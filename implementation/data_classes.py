import carla

from typing import Optional, List, Tuple, TYPE_CHECKING, Union

from implementation.platoon_controller.knowledge.base_attribute import *
from implementation.util import *

if TYPE_CHECKING:
    from implementation.vehicle.controller import Controller


class EnvironmentKnowledge(object):

    def __init__(self):

        self.timestamp: Optional[carla.Timestamp] = carla.Timestamp()
        self.timestamp.elapsed_seconds = 0.0
        self.time_to_last_step: float = 0.05
        self.connection_strength: Optional[float] = 0
        self.ego_acceleration: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
        self.ego_distance: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
        self.ego_speed: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
        self.ego_name: str = ""

        self.front_vehicles_speed: List[Tuple[Optional[float], FailureType]] = []
        self.front_vehicles_acceleration: List[Tuple[Optional[float], FailureType]] = []

        self.leader_acceleration: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
        self.leader_speed: Tuple[Optional[float], FailureType] = (0, FailureType.omission)

        self.speed_limit: Optional[float] = 60
        self.weather: Optional[Weather] = Weather.SUNSHINE

    def __str__(self) -> str:
        front_vehicles_string = ""
        newline = "\n"
        for i in range(len(self.front_vehicles_speed)):
            front_vehicles_string += f"front vehicle {i} speed: {self.__convert_to_kmh(self.front_vehicles_speed[i][0])} km/h {self.front_vehicles_speed[i][1]} {newline}" \
                                     f"front vehicle {i} acceleration: {self.front_vehicles_acceleration[i]} {newline}"
        string = (
            f"Vehicle name: {self.ego_name} {newline}"
            f"Timestamp: {self.timestamp.elapsed_seconds} {newline}"
            f"Connection strength: {self.connection_strength} {newline}"
            f"ego acceleration: {self.ego_acceleration} {newline}"
            f"ego distance: {self.ego_distance} {newline}"
            f"ego speed: {self.__convert_to_kmh(self.ego_speed[0])} km/h {self.ego_speed[1]}{newline}"
            f"{front_vehicles_string}"
            f"Leader acceleration: {self.leader_acceleration} {newline}"
            f"Leader speed: {self.__convert_to_kmh(self.leader_speed[0])} km/h {self.leader_speed[1]} {newline}"
            f"Speed limit: {self.speed_limit} {newline}"
            f"Weather: {self.weather} {newline}"
            )
        return string

    @staticmethod
    def __convert_to_kmh(speed):
        if speed is None:
            return None
        else:
            return speed * 3.6


class SimulationState(object):

    def __init__(self):
        self.managed_vehicle_speed_to_other_available: List[bool] = initialize_list(True, NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicle_acceleration_to_other_available: List[bool] = initialize_list(True, NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicle_ego_distance_available: List[bool] = initialize_list(True, NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicle_front_vehicle_braking_light_available: List[bool] = initialize_list(True, NUMBER_OF_MANAGED_VEHICLES)

        self.leader_speed_available: bool = True
        self.leader_acceleration_available: bool = True
        self.leader_braking_light_available: bool = True
        self.leader_perform_emergency_brake: bool = False

        self.leader_target_speed: float = 60.0
        self.environment_vehicles_target_speed: float = 60

        self.speed_limit: float = 60.0
        self.connection_strength: float = 100.0
        self.weather: Weather = Weather(1)

    def __str__(self) -> str:
        newline = '\n'
        managed_vehicles_string = ""
        for i in range(len(self.managed_vehicle_ego_distance_available)):
            managed_vehicles_string += f"managed vehicle {i} speed available: {self.managed_vehicle_speed_to_other_available[i]} {newline}" \
                                       f"managed vehicle {i} acceleration available: {self.managed_vehicle_acceleration_to_other_available[i]} {newline}" \
                                       f"managed vehicle {i} distance available: {self.managed_vehicle_ego_distance_available[i]} {newline}" \
                                       f"managed vehicle {i} braking light available: {self.managed_vehicle_front_vehicle_braking_light_available[i]} {newline}"
        string = f"Current simulation state:{newline} " \
                 f"{managed_vehicles_string} " \
                 f"leader speed available: {self.leader_speed_available}{newline}" \
                 f"leader acceleration available: {self.leader_acceleration_available}{newline}" \
                 f"leader braking_light available: {self.leader_braking_light_available}{newline}" \
                 f"leader perform emergency brake: {self.leader_perform_emergency_brake}{newline}" \
                 f"leader target speed: {self.leader_target_speed} {newline}" \
                 f"speed limit: {self.speed_limit} {newline}" \
                 f"connection_strength: {self.connection_strength} {newline}" \
                 f"weather: {self.weather} {newline}"
        return string


class MonitorInputData(object):

    def __init__(self, timestamp: carla.Timestamp):
        self.timestamp: carla.Timestamp = timestamp

        self.connection_strength: Optional[float] = None
        self.speed_limit: Optional[float] = None
        self.weather: Weather = Weather(1)

        self.ego_vehicle_acceleration: Optional[carla.IMUMeasurement] = None
        self.ego_vehicle_distance: Union[carla.ObstacleDetectionEvent, float, None] = None
        self.ego_vehicle_speed: Optional[float] = None
        self.front_vehicle_braking_light: Optional[bool] = None
        self.ego_vehicle_role_name: str = ""

        self.front_vehicles_speed: List[Optional[float]] = []
        self.front_vehicles_acceleration: List[Optional[float]] = []

        self.leader_acceleration: Optional[float] = None
        self.leader_speed: Optional[float] = None
        self.leader_emergency_brake: Optional[bool] = None

    def __str__(self) -> str:
        front_vehicle_string = ""
        for i in range(len(self.front_vehicles_speed)):
            front_vehicle_string += f"front vehicle {i} speed: {self.__convert_to_kmh(self.front_vehicles_speed[i])} km/h\n" \
                                    f"front vehicle {i} acceleration: {self.front_vehicles_acceleration[i]}\n"

        string = f"Monitor input data:\n" \
                 f"Vehicle name: {self.ego_vehicle_role_name}\n" \
                 f"timestamp: {self.timestamp}\n" \
                 f"connection_strength: {self.connection_strength}\n" \
                 f"speed_limit: {self.speed_limit}km/h\n" \
                 f"weather: {self.weather}\n" \
                 f"ego vehicle speed: {self.__convert_to_kmh(self.ego_vehicle_speed)} km/h\n" \
                 f"ego vehicle acceleration: {self.ego_vehicle_acceleration}\n" \
                 f"ego vehicle distance: {self.ego_vehicle_distance}\n" \
                 f"front vehicle braking light: {self.front_vehicle_braking_light}\n" \
                 f"leader speed: {self.__convert_to_kmh(self.leader_speed)} km/h\n" \
                 f"leader acceleration: {self.leader_acceleration}\n" \
                 f"leader emergency brake: {self.leader_emergency_brake}\n" \
                 f"{front_vehicle_string}\n"

        return string

    @staticmethod
    def __convert_to_kmh(speed):
        if speed is None:
            return None
        else:
            return speed * 3.6


class SystemState(object):

    def __init__(self):
        self.controller: Optional["Controller"] = None


class Plan(Enum):
    CACC_CONTROLLER = 1


class FailureType(Enum):

    no_failure = 1
    omission = 2
    faulty_value = 3
    delay = 4
    faulty_delayed = 5
    no_front_vehicle = 6
    wrong_front_vehicle = 7


class AdaptationTechnique(Enum):
    NONE = 1
    STRUCTURAL = 2
    PARAMETER = 3

