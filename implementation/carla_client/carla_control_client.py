import carla
import random
import sys
import pygame
import threading
import queue

from implementation.configuration_parameter import *
from implementation.carla_client.carla_steering_algorithm import CarlaSteeringAlgorithm
from implementation.monitor.carla_monitor import CarlaMonitor
from implementation.carla_client.user_control_window import UserControlWindow
from implementation.carla_client.simulation_state import SimulationState
from implementation.carla_client.speed_controller import VehiclePIDController
from implementation.knowledge.base_attribute import Weather

class CarlaControlClient(object):

    def __init__(self):
        self.client: carla.Client = None
        self.carla_world: carla.World = None
        self.settings: carla.Settings = None
        self.raining: bool = False

        self.pygame_clock = pygame.time.Clock()
        self.window_control_queue = queue.Queue()
        self.user_control_window: UserControlWindow = UserControlWindow(self.window_control_queue)

        self.leader_vehicle: carla.Vehicle = None
        self.ego_vehicle: carla.Vehicle = None
        self.carla_steering_algorithm_ego: CarlaSteeringAlgorithm = None
        self.carla_steering_algorithm_leader: CarlaSteeringAlgorithm = None
        self.carla_ego_control = carla.VehicleControl()
        self.leader_control = carla.VehicleControl()

        self.leader_speed_controller: VehiclePIDController = None
        self.ego_speed_controller: VehiclePIDController = None

        self.carla_monitor: CarlaMonitor = None
        self.simulation_state: SimulationState = None

        self.user_control_window_thread: threading.Thread = None

        self.scene_camera = None

        self.setup_simulation()
        self.game_loop()

    def setup_simulation(self):

        self.initialize_carla_client()
        self.spawn_leader()
        self.spawn_ego_vehicle()
        self.setup_camera()
        self.move_spectator()
        self.carla_world.tick()  # tick once, to update actors

        self.setup_lead_vehicle()
        self.setup_ego_vehicle()

        self.carla_monitor = CarlaMonitor(self.carla_world, self.ego_vehicle, self.leader_vehicle)

        default_simulation_state = SimulationState()
        self.simulation_state = default_simulation_state

        self.user_control_window_thread = threading.Thread(target=self.user_control_window_thread_execution)
        self.user_control_window_thread.start()

    def user_control_window_thread_execution(self):
        self.user_control_window.window_loop()

    def initialize_carla_client(self) -> None:
        """Creates the carla client, loads the world and sets the simulation to synchronous mode.
        :return:
        """

        self.client = carla.Client(CARLA_SERVER_HOST, CARLA_SERVER_PORT)
        self.client.load_world("Town05")
        self.carla_world = self.client.get_world()

        self.settings = self.carla_world.get_settings()
        self.settings.synchronous_mode = True
        self.settings.fixed_delta_seconds = 1 / CARLA_SERVER_FPS

        self.carla_world.apply_settings(self.settings)

    def setup_lead_vehicle(self) -> None:
        """Creates the steering algorithm for the leader vehicle"""
        args_long_dict = {
            'K_P': LEADER_CONTROLLER_KP,
            'K_D': LEADER_CONTROLLER_KD,
            'K_I': LEADER_CONTROLLER_KI,
            'dt': 1/CARLA_SERVER_FPS
        }
        self.carla_steering_algorithm_leader = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.leader_vehicle)
        self.leader_speed_controller = VehiclePIDController(self.leader_vehicle, args_long_dict)

    def setup_ego_vehicle(self) -> None:
        args_long_dict = {
            'K_P': EGO_CONTROLLER_KP,
            'K_D': EGO_CONTROLLER_KD,
            'K_I': EGO_CONTROLLER_KI,
            'dt': 1 / CARLA_SERVER_FPS
        }
        self.carla_steering_algorithm_ego = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.ego_vehicle)
        self.ego_speed_controller = VehiclePIDController(self.ego_vehicle, args_long_dict)

    def move_spectator(self) -> None:
        """Sets the spectator to the same transform as the scene camera."""

        spectator = self.carla_world.get_spectator()
        if spectator is not None and self.scene_camera is not None:
            spectator.set_transform(self.scene_camera.get_transform())

    def setup_camera(self) -> None:
        blueprint = self.create_camera_blueprint()
        transform = carla.Transform(
            carla.Location(SCENE_CAMERA_LOCATION_X, SCENE_CAMERA_LOCATION_Y, SCENE_CAMERA_LOCATION_Z),
            carla.Rotation(SCENE_CAMERA_ROTATION_PITCH, SCENE_CAMERA_ROTATION_YAW, SCENE_CAMERA_ROTATION_ROLL)
        )
        self.scene_camera = self.carla_world.spawn_actor(blueprint, transform, self.ego_vehicle)

    def create_camera_blueprint(self) -> carla.ActorBlueprint:
        """Creates the carla blueprint for the scene camera
        :return carla.Blueprint
        """
        bp_library = self.carla_world.get_blueprint_library()
        blueprint = bp_library.find("sensor.camera.rgb")
        blueprint.set_attribute('image_size_x', str(PYGAME_WINDOW_WIDTH))
        blueprint.set_attribute('image_size_y', str(PYGAME_WINDOW_HEIGHT))

        blueprint.set_attribute('role_name', "scene_camera")
        blueprint.set_attribute('blur_amount', str(0.5))
        blueprint.set_attribute('motion_blur_intensity', str(0.225))
        blueprint.set_attribute('motion_blur_max_distortion', str(0.175))
        blueprint.set_attribute('motion_blur_min_object_screen_size', str(0.05))
        return blueprint

    def update_weather(self, weather: Weather) -> None:
        """Changes the weather between sunshine and rain
        :return:
        """
        new_weather = carla.WeatherParameters()
        if weather == Weather.SUNSHINE:
            new_weather = carla.WeatherParameters.ClearNoon
        elif weather == Weather.RAIN:
            new_weather = carla.WeatherParameters(precipitation=100.0, wetness=100.0)
        elif weather == Weather.FOG:
            new_weather = carla.WeatherParameters(fog_density=100.0, fog_distance=0.0,fog_falloff=0.5)
        self.carla_world.set_weather(new_weather)


    def spawn_leader(self) -> None:

        blueprint_library = self.carla_world.get_blueprint_library()
        spawn_point = carla.Transform()
        spawn_point.location.x = LEADER_SPAWN_LOCATION_X
        spawn_point.location.y = LEADER_SPAWN_LOCATION_Y
        spawn_point.location.z = LEADER_SPAWN_LOCATION_Z

        spawn_point.rotation.yaw = LEADER_SPAWN_ROTATION_YAW

        blueprint = blueprint_library.find('vehicle.bmw.grandtourer')
        print(blueprint)

        while self.leader_vehicle is None:
            self.leader_vehicle = self.carla_world.try_spawn_actor(blueprint, spawn_point)

    def spawn_ego_vehicle(self) -> None:
        blueprint_library = self.carla_world.get_blueprint_library()
        spawn_point = carla.Transform()
        spawn_point.location.x = EGO_SPAWN_LOCATION_X
        spawn_point.location.y = EGO_SPAWN_LOCATION_Y
        spawn_point.location.z = EGO_SPAWN_LOCATION_Z

        spawn_point.rotation.yaw = EGO_SPAWN_ROTATION_YAW

        blueprint = blueprint_library.find('vehicle.audi.a2')

        while self.ego_vehicle is None:
            self.ego_vehicle = self.carla_world.try_spawn_actor(blueprint, spawn_point)

    def game_loop(self) -> None:

        running = True
        while running:
            try:

                if not self.window_control_queue.empty():
                    event = self.window_control_queue.get(False)
                    if isinstance(event, SimulationState):
                        old_weather = self.simulation_state.weather
                        self.simulation_state = event
                        if self.simulation_state.weather is not old_weather:
                            self.update_weather(self.simulation_state.weather)
                    elif event == "EXIT":
                        self.exit_client()
                    elif event == "RESET":
                        self.reset_simulation()

                self.pygame_clock.tick_busy_loop(CARLA_SERVER_FPS)
                self.carla_world.tick()
                world_snapshot = self.carla_world.get_snapshot()

                self.carla_monitor.run_step(world_snapshot.timestamp, self.simulation_state)

                if MOVE_SPECTATOR:
                    self.move_spectator()

                if self.carla_steering_algorithm_leader is not None and self.leader_vehicle is not None:
                    self.leader_control = self.leader_speed_controller.run_step(self.simulation_state.leader_speed)
                    self.leader_control.steer = self.carla_steering_algorithm_leader.goToNextTargetLocation()

                    if self.simulation_state.other_perform_emergency_brake:
                        self.leader_vehicle.set_light_state(carla.VehicleLightState.Brake)
                        self.leader_control.throttle = 0.0
                        self.leader_control.brake = 1.0

                    self.leader_vehicle.apply_control(self.leader_control)
                if DEBUG_MODE:
                    print("Leader vehicle control: ", self.leader_control)

                if self.carla_steering_algorithm_ego is not None and self.ego_vehicle is not None:
                    self.carla_ego_control = self.ego_speed_controller.run_step(self.simulation_state.speed_limit)
                    self.carla_ego_control.steer = self.carla_steering_algorithm_ego.goToNextTargetLocation()
                    self.ego_vehicle.apply_control(self.carla_ego_control)
                if DEBUG_MODE:
                    print("Ego vehicle control: ", self.carla_ego_control)

            except KeyboardInterrupt:
                running = False
                self.exit_client()
            except Exception as error:
                running = False
                import traceback
                print(error)
                traceback.print_exc()

        self.exit_client()

    def reset_simulation(self) -> None:
        if self.scene_camera is not None:
            self.scene_camera.destroy()
            self.scene_camera = None
        if self.ego_vehicle is not None:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None
        if self.leader_vehicle is not None:
            self.leader_vehicle.destroy()
            self.leader_vehicle = None
        if self.carla_steering_algorithm_leader is not None:
            self.carla_steering_algorithm_leader = None
        if self.carla_steering_algorithm_ego is not None:
            self.carla_steering_algorithm_ego = None
        if self.carla_monitor is not None:
            self.carla_monitor = None

        self.setup_simulation()

    def exit_client(self) -> None:

        print("Exiting client")

        self.reset_simulation()

        if self.user_control_window is not None:
            self.user_control_window.exit_window()
            self.user_control_window_thread.join()
            self.user_control_window_thread = None
            self.user_control_window = None

        settings = self.carla_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0  # set to 0 -> server default
        settings.no_rendering_mode = False
        self.carla_world.apply_settings(settings)
        self.carla_world.wait_for_tick()


if __name__ == '__main__':
    carla_control_client = None
    try:
        carla_control_client = CarlaControlClient()
    except Exception as e:
        print("Error executing Control Client.", e)
        import traceback
        import logging
        traceback.print_exc()
        logging.error(e)
    except KeyboardInterrupt as e:
        if carla_control_client:
            carla_control_client.exit_client()
        sys.exit()
    finally:
        if carla_control_client:
            carla_control_client.exit_client()
