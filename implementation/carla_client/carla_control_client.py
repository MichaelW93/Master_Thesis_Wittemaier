import threading
import queue
import random

from typing import List, Optional, Tuple
from matplotlib import pyplot as pyp

from implementation.configuration_parameter import *
#from implementation.carla_client.data_provider import DataProvider
from implementation.carla_client.user_control_window import UserControlWindow
from implementation.carla_client.manual_control import *
from implementation.data_classes import SimulationState
from implementation.carla_client.scenarios import *
from implementation.platoon_controller.knowledge.base_attribute import Weather
from implementation.vehicle.vehicles import *
from implementation.util import *
from implementation.carla_client.communication_handler import CommunicationHandler
from implementation.carla_client.scenario_controller import ScenarioController
from implementation.DecisionTree.TreeTrainer import TreeTrainer
from implementation.vehicle.controller import ControllerType


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

        #self.data_provider: DataProvider = None
        self.communication_handler: CommunicationHandler = CommunicationHandler()
        self.simulation_state: SimulationState = SimulationState()
        self.scenario_controller = ScenarioController(self, self.carla_world, self.communication_handler)
        self.user_control_window_thread: threading.Thread = None

        self.scene_camera = None
        self.leader_camera = None
        self.setup_simulation()

        self.current_scenario = None

        self.counter = 0

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
        self.simulation_state = SimulationState()
        self.communication_handler.setup()
        self.spawn_leader()
        self.spawn_managed_vehicles()
        self.spawn_environment_vehicle()
        self.scenario_controller.setup(self.leader_vehicle, self.managed_vehicles)

        transform = carla.Transform(
            carla.Location(SCENE_CAMERA_LOCATION_X, SCENE_CAMERA_LOCATION_Y, SCENE_CAMERA_LOCATION_Z),
            carla.Rotation(SCENE_CAMERA_ROTATION_PITCH, SCENE_CAMERA_ROTATION_YAW, SCENE_CAMERA_ROTATION_ROLL)
        )
        if len(self.managed_vehicles) > 0:
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

        self.setup_other_vehicles()
        self.manual_control_window = ManualControlWindow(self.carla_world, self.carla_map,
                                                         self.leader_vehicle.ego_vehicle,
                                                         self.leader_camera, self.carla_client)

    def user_control_window_thread_execution(self):
        self.user_control_window.window_loop()

    def setup_other_vehicles(self):
        for current_vehicle in self.managed_vehicles:
            current_vehicle.other_vehicles.append(self.leader_vehicle)

            for vehicle in self.managed_vehicles:
                if vehicle.ego_vehicle.id != current_vehicle.ego_vehicle.id:
                    current_vehicle.other_vehicles.append(vehicle)


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
        self.leader_vehicle.spawn_vehicle(spawn_point, "vehicle.audi.tt")
        self.leader_vehicle.setup_vehicle()
        self.carla_world.tick()
        self.simulation_state.vehicles_data_available[self.leader_vehicle.ego_vehicle.id] = True
        self.communication_handler.vehicles[self.leader_vehicle.ego_vehicle.id] = self.leader_vehicle

    def spawn_managed_vehicles(self) -> None:
        spawn_point_offset = 0
        for i in range(NUMBER_OF_MANAGED_VEHICLES):
            vehicle = ManagedVehicle(self.carla_world, f"Follower_{i}", self.communication_handler, self.carla_map)
            self.managed_vehicles.append(vehicle)
            spawn_point = carla.Transform()
            spawn_point.location.x = MANAGED_VEHICLE_SPAWN_LOCATION_X + spawn_point_offset
            spawn_point.location.y = MANAGED_VEHICLE_SPAWN_LOCATION_Y
            spawn_point.location.z = MANAGED_VEHICLE_SPAWN_LOCATION_Z

            spawn_point.rotation.yaw = MANAGED_VEHICLE_SPAWN_ROTATION_YAW
            blueprint = random.choice(["vehicle.audi.a2", "vehicle.audi.etron", "vehicle.audi.tt", "vehicle.charger2020.charger2020"])
            self.managed_vehicles[i].spawn_vehicle(spawn_point, "vehicle.audi.tt")
            self.managed_vehicles[i].leader_vehicle = self.leader_vehicle
            self.simulation_state.vehicles_data_available[vehicle.ego_vehicle.id] = True
            print("Managed vehicle spawned")
            spawn_point_offset -= 10
            self.carla_world.tick()
            self.communication_handler.vehicles[vehicle.ego_vehicle.id] = vehicle

    def spawn_environment_vehicle(self) -> EnvironmentVehicle:
        spawn_point = carla.Transform()
        spawn_point.location.x = MANAGED_VEHICLE_SPAWN_LOCATION_X
        spawn_point.location.y = MANAGED_VEHICLE_SPAWN_LOCATION_Y - 3
        spawn_point.location.z = MANAGED_VEHICLE_SPAWN_LOCATION_Z

        spawn_point.rotation.yaw = 0

        environment_vehicle = EnvironmentVehicle(self.carla_world, "Cut In Vehicle 1", self.communication_handler, self.leader_vehicle)
        self.environment_vehicles.append(environment_vehicle)
        environment_vehicle.spawn_vehicle(spawn_point, "vehicle.audi.tt", "0,255,0")
        environment_vehicle.setup_vehicle()
        self.environment_vehicles.append(environment_vehicle)
        for vehicle in self.managed_vehicles:
            vehicle.other_vehicles.append(environment_vehicle)
        return environment_vehicle

    def __run_cut_in_scenario(self):

        self.environment_vehicles[1].switch_lane_right()
        return
        spawn_point = carla.Transform()
        spawn_point.location.x = 19
        spawn_point.location.y = -205
        spawn_point.location.z = 2

        spawn_point.location.x = MANAGED_VEHICLE_SPAWN_LOCATION_X
        spawn_point.location.y = MANAGED_VEHICLE_SPAWN_LOCATION_Y - 3
        spawn_point.location.z = MANAGED_VEHICLE_SPAWN_LOCATION_Z

        spawn_point.rotation.yaw = 0
        scenario_vehicle = self.spawn_environment_vehicle(spawn_point)
        self.current_scenario = CutInScenario(self.leader_vehicle, self.managed_vehicles[0],
                                              self.carla_world, scenario_vehicle)

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

                if not MANUAL_SCENARIO_MODE:
                    self.simulation_state = self.scenario_controller.run_step()

                timestamp = world_snapshot.timestamp
                self.communication_handler.run_step(self.simulation_state)
                manual_vehicle_control = None
                if self.current_scenario is not None:
                    self.current_scenario.run_step(self.simulation_state)
                if self.manual_control_window is not None:
                    manual_vehicle_control = self.manual_control_window.tick(self.pygame_clock)
                if self.leader_vehicle is not None:
                    self.leader_vehicle.run_step(manual_vehicle_control, self.simulation_state, timestamp)
                for vehicle_number in range(len(self.managed_vehicles)):
                    if self.managed_vehicles[vehicle_number] is not None:
                        self.managed_vehicles[vehicle_number].run_step(timestamp,
                                                                       self.simulation_state.weather,
                                                                       self.simulation_state.speed_limit,
                                                                       self.simulation_state)
                for vehicle_number in range(len(self.environment_vehicles)):
                    if self.environment_vehicles[vehicle_number] is not None:
                        self.environment_vehicles[vehicle_number].run_step(self.simulation_state.environment_vehicles_target_speed)
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
                simulation_state.vehicles_data_available[i] = False
        for vehicle in self.managed_vehicles:
            vehicle.data_collector.record_data = simulation_state.record_data
            if vehicle.platoon_controller.knowledge.current_controller == ControllerType.DISTANCE:
                vehicle.controller.k1 = simulation_state.k1
                vehicle.controller.k2 = simulation_state.k2
                vehicle.controller.k3 = simulation_state.k3
            elif vehicle.platoon_controller.knowledge.current_controller == ControllerType.BRAKE:
                vehicle.controller.k1 = simulation_state.k1
                vehicle.controller.k2 = simulation_state.k2
                vehicle.controller.k3 = simulation_state.k3
            if vehicle.role_name == "Follower_1":
                if simulation_state.controller_follower_1 == "CACC":
                    vehicle.platoon_controller.set_manual_plan(Plan.SWITCH_TO_CACC)
                elif simulation_state.controller_follower_1 == "ACC":
                    vehicle.platoon_controller.set_manual_plan(Plan.SWITCH_TO_ACC)
                elif simulation_state.controller_follower_1 == "BREAK":
                    vehicle.platoon_controller.set_manual_plan(Plan.SWITCH_TO_BRAKE)
                elif simulation_state.controller_follower_1 == "SPEED":
                    vehicle.platoon_controller.set_manual_plan(Plan.SWITCH_TO_SPEED)

        return simulation_state

    def restart_simulation(self) -> None:

        self.reset_simulation()
        self.setup_simulation()

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
        self.scenario_controller.reset()
        self.communication_handler.reset()
        if self.manual_control_window is not None:
            self.manual_control_window.destroy()
            self.manual_control_window = None

    def __remove_environment_vehicles(self):
        for vehicle in self.environment_vehicles:
            for managed_vehicle in self.managed_vehicles:
                if vehicle in managed_vehicle.other_vehicles:
                    managed_vehicle.other_vehicles.remove(vehicle)
            vehicle.destroy()
        self.environment_vehicles = []
        self.current_scenario = None

    def exit_client(self) -> None:

        print("Exiting client")

        # self.plot_vehicle_speed()

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
        #tree_trainer = TreeTrainer()
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
