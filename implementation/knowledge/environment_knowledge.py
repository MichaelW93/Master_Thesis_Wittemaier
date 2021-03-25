import carla

from implementation.knowledge.base_attribute import *


class EnvironmentKnowledge(object):

    def __init__(self):

        self.timestamp: carla.Timestamp = None
        self.connection_strength: float = None
        self.ego_acceleration: tuple = None
        self.ego_distance: float = None
        self.ego_speed: tuple = None

        self.other_acceleration: tuple = None
        self.other_emergency_brake: bool = None
        self.other_braking_light: bool = None
        self.other_speed: tuple = None

        self.speed_limit: float = None
        self.weather: Weather = None

    def __str__(self) -> str:
        string = (
            f"Timestamp: {self.timestamp.elapsed_seconds} \n " \
            f"Connection strength: {self.connection_strength} \n " \
            f"Ego acceleration: {self.ego_acceleration} \n " \
            f"Ego distance: {self.ego_distance} \n " \
            f"Ego speed: {self.ego_speed[0] * 3.6} \n " \
            f"Leader acceleration: {self.other_acceleration} \n " \
            f"Leader speed: {self.other_speed[0] * 3.6} \n " \
            f"Leader emergency brake: {self.other_emergency_brake} \n " \
            f"Leader braking light: {self.other_braking_light} \n " \
            f"Speed limit: {self.speed_limit} \n " \
            f"Weather: {self.weather}"
            )
        return string
