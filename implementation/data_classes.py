import carla

from typing import Optional, List

from implementation.platoon_controller.knowledge.base_attribute import *
from implementation.util import *


class EnvironmentKnowledge(object):

    def __init__(self):

        self.timestamp: Optional[carla.Timestamp] = None
        self.connection_strength: Optional[float] = None
        self.ego_acceleration: Optional[tuple] = None
        self.ego_distance: Optional[float] = None
        self.ego_speed: Optional[tuple] = None

        self.front_vehicles_speed: List[Optional[float]] = []
        self.front_vehicles_acceleration: List[Optional[float]] = []

        self.leader_acceleration: Optional[tuple] = None
        self.leader_emergency_brake: Optional[bool] = None
        self.leader_braking_light: Optional[bool] = None
        self.leader_speed: Optional[tuple] = None

        self.speed_limit: Optional[float] = None
        self.weather: Optional[Weather] = None

    def __str__(self) -> str:
        string = (
            f"Timestamp: {self.timestamp.elapsed_seconds} \n "
            f"Connection strength: {self.connection_strength} \n "
            f"ego acceleration: {self.ego_acceleration} \n "
            f"ego distance: {self.ego_distance} \n "
            f"ego speed: {self.ego_speed[0] * 3.6} \n "
            f"front vehicles speed: {self.front_vehicles_speed} \n "
            f"front vehicles acceleration: {self.front_vehicles_acceleration} \n "
            f"Leader acceleration: {self.leader_acceleration} \n "
            f"Leader speed: {self.leader_speed[0] * 3.6} \n "
            f"Leader emergency brake: {self.leader_emergency_brake} \n "
            f"Leader braking light: {self.leader_braking_light} \n "
            f"Speed limit: {self.speed_limit} \n "
            f"Weather: {self.weather}"
            )
        return string


class SimulationState(object):

    def __init__(self):
        self.managed_vehicle_speed_available: List[bool] = initialize_array(True, NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicle_acceleration_available: List[bool] = initialize_array(True, NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicle_distance_available: List[bool] = initialize_array(True, NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicle_braking_light_available: List[bool] = initialize_array(True, NUMBER_OF_MANAGED_VEHICLES)

        self.leader_speed_available: bool = True
        self.leader_acceleration_available: bool = True
        self.leader_braking_light_available: bool = True
        self.leader_perform_emergency_brake: bool = False

        self.leader_target_speed: float = 60.0

        self.speed_limit: float = 60.0
        self.connection_strength: float = 100.0
        self.weather: Weather = Weather(1)


class MonitorInputData(object):

    def __init__(self, timestamp: carla.Timestamp):
        self.timestamp = timestamp

        self.connection_strength: Optional[float] = None
        self.speed_limit: Optional[float] = None
        self.weather: Weather = Weather(1)

        self.managed_vehicle_acceleration: Optional[carla.IMUMeasurement] = None
        self.managed_vehicle_distance: Optional[carla.ObstacleDetectionEvent] = None
        self.managed_vehicle_speed: Optional[float] = None
        self.front_vehicle_braking_light: Optional[bool] = None

        self.front_vehicles_speed: List[Optional[float]] = []
        self.front_vehicles_acceleration: List[Optional[float]] = []

        self.leader_acceleration: Optional[float] = None
        self.leader_speed: Optional[float] = None
        self.leader_emergency_brake: Optional[bool] = None