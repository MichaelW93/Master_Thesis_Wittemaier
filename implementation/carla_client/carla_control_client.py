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
        self.managed_vehicles: List[Optional[ManagedVehicle]] = []

        self.data_provider: DataProvider = None
        self.simulation_state: SimulationState = SimulationState()

        self.user_control_window_thread: threading.Thread = None

        self.scene_camera = None
        self.leader_camera = None

        self.setup_simulation()
        self.game_loop()

    def setup_simulation(self):

        self.initialize_carla_client()
        self.spawn_leader()
        self.spawn_managed_vehicles()
        self.data_provider = DataProvider(self.managed_vehicles, self.leader_vehicle,
                                          self.carla_world, self.simulation_state)
        self.scene_camera = self.setup_camera(self.ego_vehicle)
        self.leader_camera = self.setup_camera(self.leader_vehicle)
        self.move_spectator()
        self.carla_world.tick()  # tick once, to update actors

        self.leader_vehicle.setup_vehicle()
        for vehicle in self.managed_vehicles:
            vehicle.setup_vehicle()

        default_simulation_state = SimulationState()
        self.simulation_state = default_simulation_state

        self.manual_control_window = ManualControlWindow(self.carla_world, self.carla_map, self.leader_vehicle,
                                                         self.leader_camera, self.carla_client)

        self.user_control_window_thread = threading.Thread(target=self.user_control_window_thread_execution)
        self.user_control_window_thread.start()

    def user_control_window_thread_execution(self):
        self.user_control_window.window_loop()

    def setup_front_vehicles(self):
        for i in range(len(self.managed_vehicles)):
            if i == 0:
                self.managed_vehicles[i].front_vehicles = []
            else:
                self.managed_vehicles[i].front_vehicles = self.managed_vehicles[0: i-1]


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

    def spawn_managed_vehicles(self) -> None:
        spawn_point_offset = 0
        for i in range(NUMBER_OF_MANAGED_VEHICLES):
            self.managed_vehicles.append(ManagedVehicle(self.carla_world))
            spawn_point = carla.Transform()
            spawn_point.location.x = MANAGED_VEHICLE_SPAWN_LOCATION_X + spawn_point_offset
            spawn_point.location.y = MANAGED_VEHICLE_SPAWN_LOCATION_Y
            spawn_point.location.z = MANAGED_VEHICLE_SPAWN_LOCATION_Z

            spawn_point.rotation.yaw = MANAGED_VEHICLE_SPAWN_ROTATION_YAW
            self.managed_vehicles[i].spawn_vehicle(spawn_point, "vehicle.audi.a2")
            self.managed_vehicles[i].leader_vehicle = self.leader_vehicle
            spawn_point_offset += 20

    def game_loop(self) -> None:

        running = True
        while running:
            try:

                if not self.window_control_queue.empty():
                    event = self.window_control_queue.get(False)
                    if isinstance(event, SimulationState):
                        old_weather = self.simulation_state.weather
                        self.simulation_state = self.update_simulation_state(event)
                        if self.simulation_state.weather is not old_weather:
                            self.update_weather(self.simulation_state.weather)
                    elif event == "EXIT":
                        self.exit_client()
                    elif event == "RESET":
                        self.reset_simulation()

                self.pygame_clock.tick_busy_loop(CARLA_SERVER_FPS)
                self.carla_world.tick()
                world_snapshot = self.carla_world.get_snapshot()

                vehicles_monitor_input_data = DataProvider.collect_data(world_snapshot.timestamp, self.simulation_state)
                if MOVE_SPECTATOR:
                    self.move_spectator()

                manual_vehicle_control = self.manual_control_window.tick(self.pygame_clock)
                print("Manual control command: ", manual_vehicle_control)
                self.leader_vehicle.run_step(manual_vehicle_control, self.simulation_state)

                for vehicle_number in range(len(self.managed_vehicles)):
                    self.managed_vehicles[vehicle_number].run_step(
                        vehicles_monitor_input_data[vehicle_number], self.simulation_state)

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
                simulation_state.managed_vehicle_distance_available[i] = False
                simulation_state.managed_vehicle_acceleration_available[i] = False
                simulation_state.managed_vehicle_speed_available[i] = False
                simulation_state.managed_vehicle_braking_light_available[i] = False
        return simulation_state

    def reset_simulation(self) -> None:
        if self.scene_camera is not None:
            self.scene_camera.destroy()
            self.scene_camera = None
        if self.leader_camera is not None:
            self.leader_camera.destroy()
            self.leader_camera = None
        for vehicle in self.managed_vehicles:
            if vehicle is not None:
                if vehicle.ego_vehicle is not None:
                    vehicle.destroy()
        self.managed_vehicles = []

        if self.leader_vehicle is not None:
            self.leader_vehicle.ego_vehicle.destroy()
            self.leader_vehicle = None

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
