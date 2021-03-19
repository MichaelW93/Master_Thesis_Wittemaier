import carla

from implementation.knowledge.base_attribute import *


class EnvironmentKnowledge(object):

    def __init__(self):

        self.timestamp: carla.Timestamp = None
        self.connection_strength: float = None
        self.ego_acceleration: tuple = None
        self.ego_distance: float = None
        self.ego_velocity: tuple = None

        self.other_acceleration: tuple = None
        self.other_emergency_brake: bool = None
        self.other_braking_light: bool = None
        self.other_velocity: tuple = None

        self.speed_limit: float = None
        self.weather: Weather = None
