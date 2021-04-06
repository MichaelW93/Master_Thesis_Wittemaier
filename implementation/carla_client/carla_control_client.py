import threading
import queue

from typing import List, Optional

from implementation.configuration_parameter import *
from implementation.carla_client.data_provider import DataProvider
from implementation.carla_client.user_control_window import UserControlWindow
from implementation.carla_client.manual_control import *
from implementation.data_classes import SimulationState
from implementation.platoon_controller.knowledge.base_attribute import Weather
from implementation.vehicle.vehicles import LeaderVehicle, ManagedVehicle


class CarlaControlClient(object):

    def __init__(self):
        self.carla_client: carla.Client = None
        self.carla_world: carla.World = None
        self.carla_map: carla.Map = None
        self.settings: carla.Settings = None
        self.raining: bool = False

        self.pygame_clock = pygame.time.Clock()
        self.window_control_queue = queue.Queue()
        self.user_control_window: UserControlWindow = UserControlWindow(self.window_control_queue)
        self.manual_control_window: ManualControlWindow = None

        self.leader_vehicle: LeaderVehicle = None
        self.ego_vehicle: carla.Vehicle = None
        self.managed_vehicles: List[Optional[ManagedVehicle]] = []

        self.data_provider: DataProvider = None
        self.simulation_state: SimulationState = None

        self.user_control_window_thread: threading.Thread = None

        self.scene_camera = None
        self.leader_camera = None

        self.setup_simulation()
        self.game_loop()

    def setup_simulation(self):

        self.initialize_carla_client()
        self.spawn_leader()
        self.spawn_managed_vehicle()
        self.data_provider = DataProvider(self.ego_vehicle, self.leader_vehicle, self.carla_world)
        self.scene_camera = self.setup_camera(self.ego_vehicle)
        self.leader_camera = self.setup_camera(self.leader_vehicle)
        self.move_spectator()
        self.carla_world.tick()  # tick once, to update actors

        self.setup_lead_vehicle()
        self.setup_ego_vehicle()

        default_simulation_state = SimulationState()
        self.simulation_state = default_simulation_state

        self.manual_control_window = ManualControlWindow(self.carla_world, self.carla_map, self.leader_vehicle,
                                                         self.leader_camera, self.carla_client)

        self.user_control_window_thread = threading.Thread(target=self.user_control_window_thread_execution)
        self.user_control_window_thread.start()

    def user_control_window_thread_execution(self):
        self.user_control_window.window_loop()

    def initialize_carla_client(self) -> None:
        """Creates the carla client, loads the world and sets the simulation to synchronous mode.
        :return:
        """

        self.carla_client = carla.Client(CARLA_SERVER_HOST, CARLA_SERVER_PORT)
        self.carla_client.load_world("Town05")
        self.carla_world = self.carla_client.get_world()
        self.carla_map = self.carla_world.get_map()

        self.settings = self.carla_world.get_settings()
        self.settings.synchronous_mode = True
        self.settings.fixed_delta_seconds = 1 / CARLA_SERVER_FPS

        self.carla_world.apply_settings(self.settings)

    def move_spectator(self) -> None:
        """Sets the spectator to the same transform as the scene camera."""

        spectator = self.carla_world.get_spectator()
        if spectator is not None and self.scene_camera is not None:
            spectator.set_transform(self.scene_camera.get_transform())

    def setup_camera(self, vehicle: carla.Vehicle) -> carla.Sensor:
        blueprint = self.create_camera_blueprint()
        transform = carla.Transform(
            carla.Location(SCENE_CAMERA_LOCATION_X, SCENE_CAMERA_LOCATION_Y, SCENE_CAMERA_LOCATION_Z),
            carla.Rotation(SCENE_CAMERA_ROTATION_PITCH, SCENE_CAMERA_ROTATION_YAW, SCENE_CAMERA_ROTATION_ROLL)
        )
        camera = self.carla_world.spawn_actor(blueprint, transform, vehicle)
        return camera

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
        self.leader_vehicle = LeaderVehicle(self.carla_world)
        spawn_point = carla.Transform()
        spawn_point.location.x = LEADER_SPAWN_LOCATION_X
        spawn_point.location.y = LEADER_SPAWN_LOCATION_Y
        spawn_point.location.z = LEADER_SPAWN_LOCATION_Z
        spawn_point.rotation.yaw = LEADER_SPAWN_ROTATION_YAW
        self.leader_vehicle.spawn_vehicle(spawn_point, "vehicle.bmw.grandtourer")
        self.leader_vehicle.setup_vehicle()

    def spawn_managed_vehicle(self, number_of_managed_vehicles) -> None:
        spawn_point_offset = 0
        for i in range(number_of_managed_vehicles):
            self.managed_vehicles.append(ManagedVehicle(self.carla_world))
            spawn_point = carla.Transform()
            spawn_point.location.x = MANAGED_VEHICLE_SPAWN_LOCATION_X + spawn_point_offset
            spawn_point.location.y = MANAGED_VEHICLE_SPAWN_LOCATION_Y
            spawn_point.location.z = EGO_SPAWN_LOCATION_Z

            spawn_point.rotation.yaw = EGO_SPAWN_ROTATION_YAW
            self.managed_vehicles[i].spawn_vehicle(spawn_point, "vehicle.audi.a2")
            spawn_point_offset += 20

    def lane_change_right(self):
        current_waypoint = self.carla_map.get_waypoint(self.leader_vehicle.get_location, project_to_road = True)
        waypoint_right = current_waypoint.get_right_lane()
        target_waypoint = waypoint_right.next(20.0)
        self.carla_steering_algorithm_leader.set_next_waypoint(target_waypoint)

    def lane_change_left(self):
        current_waypoint = self.carla_map.get_waypoint(self.leader_vehicle.get_location, project_to_road = True)
        waypoint_left = current_waypoint.get_left_lane()
        target_waypoint = waypoint_left.next(20.0)
        self.carla_steering_algorithm_leader.set_next_waypoint(target_waypoint)

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

                monitor_input_data = DataProvider.collect_data(world_snapshot.timestamp, self.simulation_state)

                self.carla_monitor.run_step(monitor_input_data)

                if MOVE_SPECTATOR:
                    self.move_spectator()

                manual_vehicle_control = self.manual_control_window.tick(self.pygame_clock)
                print("Manual control command: ", manual_vehicle_control)

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
        if self.leader_camera is not None:
            self.leader_camera.destroy()
            self.leader_camera = None
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
