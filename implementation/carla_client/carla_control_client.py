import threading
import queue
import random

from typing import List, Optional, Tuple
from matplotlib import pyplot as pyp

from implementation.configuration_parameter import *
from implementation.carla_client.data_provider import DataProvider
from implementation.carla_client.user_control_window import UserControlWindow
from implementation.carla_client.manual_control import *
from implementation.data_classes import SimulationState
from implementation.carla_client.scenarios import *
from implementation.platoon_controller.knowledge.base_attribute import Weather
from implementation.vehicle.vehicles import *
from implementation.util import *
from implementation.carla_client.communication_handler import CommunicationHandler


class CarlaControlClient(object):

    def __init__(self):
        self.carla_client: carla.Client = None
        self.carla_world: carla.World = None
        self.carla_map: carla.Map = None
        self.settings: carla.Settings = None
        self.raining: bool = False

        self.pygame_clock = pygame.time.Clock()
        self.window_control_queue = queue.Queue()
        self.manual_control_window: ManualControlWindow = None

        self.leader_vehicle: LeaderVehicle = None
        self.managed_vehicles: List[Optional[ManagedVehicle]] = []
        self.environment_vehicles: List[Optional[EnvironmentVehicle]] = []

        self.data_provider: DataProvider = None
        self.communication_handler: CommunicationHandler = CommunicationHandler()
        self.simulation_state: SimulationState = SimulationState()

        self.user_control_window_thread: threading.Thread = None

        self.leader_speeds: List[float] = []
        self.follower_1_speed: List[float] = []
        self.follower_2_speed: List[float] = []
        self.elapsed_seconds: List[float] = []
        self.follower_1_distance: List[float] = []
        self.follower_2_distance: List[float] = []

        self.max_x = 0
        self.min_x = 0
        self.max_y = 0
        self.min_y = 0

        self.scene_camera = None
        self.leader_camera = None

        self.current_scenario: Optional[Scenario] = None

        self.counter = 0

        self.setup_simulation()

        follower_id: List[int] = []
        for vehicle in self.managed_vehicles:
            follower_id.append(vehicle.ego_vehicle.id)
        for vehicle in self.environment_vehicles:
            follower_id.append(vehicle.ego_vehicle.id)
        self.user_control_window: UserControlWindow = UserControlWindow(self.window_control_queue,
                                                                        self.leader_vehicle.ego_vehicle.id,
                                                                        follower_id)
        self.user_control_window_thread = threading.Thread(target=self.user_control_window_thread_execution)
        self.user_control_window_thread.start()

        self.game_loop()

    def setup_simulation(self):

        self.initialize_carla_client()
        self.spawn_leader()
        self.spawn_managed_vehicles()
        self.data_provider = DataProvider(self.managed_vehicles, self.leader_vehicle,
                                          self.carla_world, self.simulation_state)
        transform = carla.Transform(
            carla.Location(SCENE_CAMERA_LOCATION_X, SCENE_CAMERA_LOCATION_Y, SCENE_CAMERA_LOCATION_Z),
            carla.Rotation(SCENE_CAMERA_ROTATION_PITCH, SCENE_CAMERA_ROTATION_YAW, SCENE_CAMERA_ROTATION_ROLL)
        )
        self.scene_camera = self.setup_camera(self.managed_vehicles[-1].ego_vehicle, transform,
                                              str(PYGAME_WINDOW_WIDTH), str(PYGAME_WINDOW_HEIGHT))
        transform = carla.Transform(
            carla.Location(LEADER_CAMERA_LOCATION_X, LEADER_CAMERA_LOCATION_Y, LEADER_CAMERA_LOCATION_Z),
            carla.Rotation(LEADER_CAMERA_ROTATION_PITCH, LEADER_CAMERA_ROTATION_YAW, LEADER_CAMERA_ROTATION_ROLL)
        )
        self.leader_camera = self.setup_camera(self.leader_vehicle.ego_vehicle, transform,
                                               str(MANUAL_CONTROL_WINDOW_WIDTH), str(MANUAL_CONTROL_WINDOW_HEIGHT))
        self.move_spectator()
        self.carla_world.tick()  # tick once, to update actors

        for i in range(len(self.managed_vehicles)):
            self.managed_vehicles[i].setup_vehicle(i)

        self.setup_front_vehicles()

        default_simulation_state = SimulationState()
        self.simulation_state = default_simulation_state

        self.manual_control_window = ManualControlWindow(self.carla_world, self.carla_map,
                                                         self.leader_vehicle.ego_vehicle,
                                                         self.leader_camera, self.carla_client)

    def user_control_window_thread_execution(self):
        self.user_control_window.window_loop()

    def setup_front_vehicles(self):
        for i in range(len(self.managed_vehicles)):
            if i == 0:
                self.managed_vehicles[i].front_vehicles = [self.leader_vehicle]
                self.managed_vehicles[i].front_vehicle_is_leader = True
            elif i == 1:
                self.managed_vehicles[i].front_vehicles = [self.managed_vehicles[0]]
            else:
                self.managed_vehicles[i].front_vehicles = self.managed_vehicles[0:(i-1)]
            print("Front vehicles: ", self.managed_vehicles[i].front_vehicles)


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

    def setup_camera(self, vehicle: carla.Vehicle, camera_transform: carla.Transform, image_width: str, image_height: str) -> carla.Sensor:
        blueprint = self.create_camera_blueprint(image_width, image_height)
        camera = self.carla_world.spawn_actor(blueprint, camera_transform, vehicle)
        return camera

    def create_camera_blueprint(self, image_width: str, image_height: str) -> carla.ActorBlueprint:
        """Creates the carla blueprint for the scene camera
        :return carla.Blueprint
        """
        bp_library = self.carla_world.get_blueprint_library()
        blueprint = bp_library.find("sensor.camera.rgb")
        blueprint.set_attribute('image_size_x', image_width)
        blueprint.set_attribute('image_size_y', image_height)

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
        self.leader_vehicle = LeaderVehicle(self.carla_world, self.communication_handler)
        spawn_point = carla.Transform()
        spawn_point.location.x = LEADER_SPAWN_LOCATION_X
        spawn_point.location.y = LEADER_SPAWN_LOCATION_Y
        spawn_point.location.z = LEADER_SPAWN_LOCATION_Z
        spawn_point.rotation.yaw = LEADER_SPAWN_ROTATION_YAW
        self.leader_vehicle.spawn_vehicle(spawn_point, "vehicle.bmw.grandtourer")
        self.leader_vehicle.setup_vehicle()
        self.carla_world.tick()
        self.communication_handler.vehicles[self.leader_vehicle.ego_vehicle.id] = self.leader_vehicle

    def spawn_managed_vehicles(self) -> None:
        spawn_point_offset = 0
        for i in range(NUMBER_OF_MANAGED_VEHICLES):
            vehicle = ManagedVehicle(self.carla_world, f"Follower_{i}", self.communication_handler)
            self.managed_vehicles.append(vehicle)
            spawn_point = carla.Transform()
            spawn_point.location.x = MANAGED_VEHICLE_SPAWN_LOCATION_X + spawn_point_offset
            spawn_point.location.y = MANAGED_VEHICLE_SPAWN_LOCATION_Y
            spawn_point.location.z = MANAGED_VEHICLE_SPAWN_LOCATION_Z

            spawn_point.rotation.yaw = MANAGED_VEHICLE_SPAWN_ROTATION_YAW
            blueprint = random.choice(["vehicle.audi.a2", "vehicle.audi.etron", "vehicle.audi.tt", "vehicle.charger2020.charger2020"])
            self.managed_vehicles[i].spawn_vehicle(spawn_point, "vehicle.audi.tt")
            self.managed_vehicles[i].leader_vehicle = self.leader_vehicle
            print("Managed vehicle spawned")
            spawn_point_offset -= 20
            self.carla_world.tick()
            self.communication_handler.vehicles[vehicle.ego_vehicle.id] = vehicle

    def spawn_environment_vehicle(self, transform: carla.Transform) -> EnvironmentVehicle:
        environment_vehicle = EnvironmentVehicle(self.carla_world, "Cut_In_Vehicle_1", self.communication_handler)
        self.environment_vehicles.append(environment_vehicle)
        environment_vehicle.spawn_vehicle(transform, "vehicle.audi.tt", "0,255,0", "Cut_In_Vehicle_1")
        environment_vehicle.setup_vehicle()
        self.carla_world.tick()
        self.communication_handler.vehicles[environment_vehicle.ego_vehicle.id] = environment_vehicle
        return environment_vehicle

    def __run_cut_in_scenario(self):
        spawn_point = carla.Transform()
        spawn_point.location.x = 19
        spawn_point.location.y = -205
        spawn_point.location.z = 2

        spawn_point.rotation.yaw = 180
        scenario_vehicle = self.spawn_environment_vehicle(spawn_point)
        self.current_scenario = CutInScenario(scenario_vehicle, self.leader_vehicle, self.managed_vehicles[0], self.carla_world)

    def game_loop(self) -> None:

        running = True
        while running:
            try:

                if not self.window_control_queue.empty():
                    event = self.window_control_queue.get(False)
                    if isinstance(event, SimulationState):
                        old_weather = self.simulation_state.weather
                        self.simulation_state = self.update_simulation_state(event)
                        if DEBUG_MODE:
                            print(self.simulation_state.__str__())
                        if self.simulation_state.weather is not old_weather:
                            self.update_weather(self.simulation_state.weather)
                    elif event == "CUT_IN":
                        self.__run_cut_in_scenario()
                    elif event == "REMOVE":
                        self.__remove_environment_vehicles()
                    elif event == "EXIT":
                        self.exit_client()
                    elif event == "RESET":
                        self.reset_simulation()

                self.pygame_clock.tick_busy_loop(CARLA_SERVER_FPS)
                self.carla_world.tick()
                world_snapshot = self.carla_world.get_snapshot()

                if MOVE_SPECTATOR:
                    self.move_spectator()

                timestamp = world_snapshot.timestamp
                self.communication_handler.run_step(self.simulation_state)
                manual_vehicle_control = self.manual_control_window.tick(self.pygame_clock)
                self.leader_vehicle.run_step(manual_vehicle_control, self.simulation_state, timestamp)
                for vehicle_number in range(len(self.managed_vehicles)):
                    self.managed_vehicles[vehicle_number].run_step(timestamp,
                                                                   self.simulation_state.weather,
                                                                   self.simulation_state.speed_limit)

                if self.current_scenario is not None:
                    self.current_scenario.run_step(self.simulation_state)

                # Plotting and other random information
                """Get max map coordinates"""
                location = self.leader_vehicle.ego_vehicle.get_location()
                if location.x > self.max_x:
                    self.max_x = location.x
                elif location.x < self.min_x:
                    self.min_x = location.x

                if location.y > self.max_y:
                    self.max_y = location.y
                elif location.y < self.min_y:
                    self.min_y = location.y

                if self.counter >= 5:
                    self.leader_speeds.append(velocity_to_speed(self.leader_vehicle.ego_vehicle.get_velocity()))
                    self.follower_1_speed.append(velocity_to_speed(self.managed_vehicles[0].get_velocity()))
                    self.follower_2_speed.append(velocity_to_speed(self.managed_vehicles[1].get_velocity()))

                    self.elapsed_seconds.append(world_snapshot.timestamp.elapsed_seconds)
                    self.counter = 0
                else:
                    self.counter += 1

                if world_snapshot.timestamp.elapsed_seconds > 10:
                    #self.plot_vehicle_speed()
                    pass
            except KeyboardInterrupt:
                running = False
                self.exit_client()
            except Exception as error:
                running = False
                import traceback
                print(error)
                traceback.print_exc()

        self.exit_client()

    def update_simulation_state(self, simulation_state: SimulationState):

        if simulation_state.connection_strength == 0:
            simulation_state.leader_acceleration_available = False
            simulation_state.leader_speed_available = False
            for i in range(len(self.managed_vehicles)):
                simulation_state.followers_distance_available[i] = False
                simulation_state.vehicles_acceleration_available[i] = False
                simulation_state.vehicles_speed_available[i] = False
        return simulation_state

    def plot_vehicle_speed(self):

        print(f"max x: {self.max_x} \n"
              f"min x: {self.min_x} \n"
              f"max y: {self.max_y} \n"
              f"min y: {self.min_y} \n")
        fig1, ax = pyp.subplots(2)
        ax[0].plot(self.elapsed_seconds, self.leader_speeds, label="leader_vehicle")
        ax[0].plot(self.elapsed_seconds, self.follower_1_speed, label="Follower 1")
        ax[0].plot(self.elapsed_seconds, self.follower_2_speed, label="Follower 2")
        ax[0].set_xlabel("Time (s)")
        ax[0].set_ylabel("Speed (m/s)")
        ax[0].set_title("Speed comparison")
        ax[0].legend()

        ax[1].plot(self.elapsed_seconds, self.follower_1_distance, label="Follower 1 distance")
        ax[1].plot(self.elapsed_seconds, self.follower_2_distance, label="Follower 2 distance")
        ax[1].set_xlabel("Time (s)")
        ax[1].set_ylabel("Distance (m)")
        ax[1].legend()
        ax[1].set_title("Distance comparison")

        pyp.show()


    def reset_simulation(self) -> None:
        if self.scene_camera is not None:
            self.scene_camera.destroy()
            self.scene_camera = None
            print("Destroyed scene camera")
        if self.leader_camera is not None:
            self.leader_camera.destroy()
            self.leader_camera = None
            print("Destroyed leader camera")
        for vehicle in self.managed_vehicles:
            if vehicle is not None:
                if vehicle.ego_vehicle is not None:
                    vehicle.destroy()
                    print("Managed vehicle destroyed")
        self.managed_vehicles = []

        if self.leader_vehicle is not None:
            self.leader_vehicle.ego_vehicle.destroy()
            self.leader_vehicle = None

        self.__remove_environment_vehicles()

        #self.setup_simulation()

    def __remove_environment_vehicles(self):

        for vehicle in self.environment_vehicles:
            vehicle.destroy()
        self.environment_vehicles = []

    def exit_client(self) -> None:

        print("Exiting client")

        self.plot_vehicle_speed()

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
