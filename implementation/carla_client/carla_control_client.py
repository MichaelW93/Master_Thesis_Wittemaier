import carla
import random

from implementation.configuration_parameter import *
from implementation.carla_agent.navigation.basic_agent import BasicAgent


class CarlaControlClient(object):

    def __init__(self):
        self.client = None
        self.carla_world = None
        self.settings = None
        self.raining = False

        self.leader_vehicle = None
        self.ego_vehicle = None
        self.leader_control_agent = None

        self.initialize_carla_client()

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

        self.traffic_manager = self.client.get_trafficmanager(TRAFFIC_MANAGER_PORT)
        self.traffic_manager_port = self.traffic_manager.get_port()

    def setup_lead_vehicle(self, vehicle: carla.Vehicle) -> None:

        self.leader_control_agent = BasicAgent(self.leader_vehicle, target_speed=60)
        self.leader_control_agent.proximity_tlight_threshold = -1 # ignores traffic lights

        help_points = [
            carla.Location(63, -190),
            carla.Location(160, -167),
            carla.Location(55, 191),
            carla.Location(-37, 191)
        ]

        self.leader_control_agent.set_destination(carla.Location(70, -186), 2, help_points)


    def set_weather(self, weather: carla.WeatherPreset) -> None:
        self.carla_world.set_weather(weather)

    def switch_weather(self) -> None:
        if self.raining:
            self.set_weather(carla.WeatherParameters.ClearNoon)
        else:
            self.set_weather(carla.WeatherParameters.WetNoon)

    def spawn_leader(self) -> None:
        blueprint_library = self.carla_world.get_blueprint_library()
        spawn_point = carla.Transform()
        spawn_point.location.x = 29.0
        spawn_point.location.y = -190.0
        spawn_point.location.z = 1.0

        spawn_point.rotation.yaw = 0.0

        blueprint = random.choice(blueprint_library.filter('vehicle.bmw.*'))

        while self.leader_vehicle is None:
            self.leader_vehicle = self.carla_world.try_spawn_actor(blueprint, spawn_point)

    def spawn_ego_vehicle(self) -> None:
        blueprint_library = self.carla_world.get_blueprint_library()
        spawn_point = carla.Transform()
        spawn_point.location.x = 39.0
        spawn_point.location.y = -190.0
        spawn_point.location.z = 1.0

        spawn_point.rotation.yaw = 0.0

        blueprint = random.choice(blueprint_library.filter('vehicle.audi.*'))

        while self.ego_vehicle is None:
            self.ego_vehicle = self.carla_world.try_spawn_actor(blueprint, spawn_point)

    def game_loop(self) -> None:

        while True:
            world_snapshot = self.carla_world.tick()