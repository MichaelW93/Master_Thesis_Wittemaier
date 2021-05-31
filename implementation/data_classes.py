import carla

from typing import Optional, List, Tuple, TYPE_CHECKING, Union, Dict
from dataclasses import dataclass, field

from implementation.platoon_controller.knowledge.base_attribute import *
from implementation.util import *

if TYPE_CHECKING:
    from implementation.vehicle.controller import Controller


class Plan(Enum):
    SWITCH_TO_CACC = 1
    SWITCH_TO_ACC = 2
    SWITCH_TO_SPEED = 3
    SWITCH_TO_BRAKE = 4
    ADAPT_TARGET_SPEED = 5
    ADAPT_TARGET_DISTANCE = 6
    NO_CHANGE = 7


class FailureType(Enum):

    no_failure = 1
    omission = 2
    faulty_value = 3
    delay = 4
    faulty_delayed = 5
    no_front_vehicle = 6
    wrong_front_vehicle = 7


class AdaptationTechnique(Enum):
    NO_ADAPTATION = 1
    STRUCTURAL = 2
    PARAMETER = 3
    CONTEXT = 4


@dataclass()
class OtherVehicle:
    id: int
    platoonable: bool = False
    is_leader: bool = False
    is_front_vehicle: bool = False
    measured_speed: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
    measured_acceleration: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
    measured_distance: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
    speed: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
    acceleration: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
    throttle: float = 0
    brake: float = 0
    steering: float = 0


@dataclass()
class EnvironmentKnowledge(object):
    timestamp: Optional[carla.Timestamp] = carla.Timestamp()
    timestamp.elapsed_seconds = 0.0
    time_to_last_step: float = 0.05
    communication_delay: Optional[float] = 0
    ego_acceleration: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
    ego_distance: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
    ego_speed: Tuple[Optional[float], FailureType] = (0, FailureType.omission)
    ego_name: str = ""

    leader_id: int = -1
    front_vehicle_id: int = -1
    other_vehicles: Dict[int, OtherVehicle] = field(default_factory=dict)

    speed_limit: Optional[float] = 60
    weather: Optional[Weather] = Weather.SUNSHINE

    """
    def __str__(self) -> str:
        front_vehicles_string = ""
        newline = "\n"
        for i in range(len(self.front_vehicles_speed)):
            front_vehicles_string += f"front vehicle {i} speed: {self.__convert_to_kmh(self.front_vehicles_speed[i][0])} km/h {self.front_vehicles_speed[i][1]} {newline}" \
                                     f"front vehicle {i} acceleration: {self.front_vehicles_acceleration[i]} {newline}"
        string = (
            f"Vehicle name: {self.ego_name} {newline}"
            f"Timestamp: {self.timestamp.elapsed_seconds} {newline}"
            f"Connection strength: {self.communication_delay} {newline}"
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
    """

    @staticmethod
    def __convert_to_kmh(speed):
        if speed is None:
            return None
        else:
            return speed * 3.6


class SimulationState(object):

    def __init__(self):
        self.vehicles_speed_available: Dict[int, bool] = {}
        self.vehicles_acceleration_available: Dict[int, bool] = {}
        self.followers_distance_available: Dict[int, bool] = {}

        self.leader_target_speed: float = 60.0
        self.environment_vehicles_target_speed: float = 60

        self.speed_limit: float = 60.0
        self.connection_strength: float = 100.0
        self.weather: Weather = Weather(1)

    def __str__(self) -> str:
        newline = '\n'
        managed_vehicles_string = ""
        i = 0
        string = f"Current simulation state:{newline} " \
                 f"{self.vehicles_speed_available} {newline}" \
                 f"{self.vehicles_acceleration_available} {newline}" \
                 f"{self.followers_distance_available} {newline}" \
                 f"leader target speed: {self.leader_target_speed} {newline}" \
                 f"speed limit: {self.speed_limit} {newline}" \
                 f"connection_strength: {self.connection_strength} {newline}" \
                 f"weather: {self.weather} {newline}"
        return string


@dataclass()
class CommunicationData:
    vehicle_id: int = -1
    timestamp: carla.Timestamp = None
    leader_id: int = -1
    front_id: int = -1
    speed: float = 0
    acceleration: float = 0
    steering: float = 0
    throttle: float = 0
    brake: float = 0


class SystemState(object):

    def __init__(self):
        self.controller: Optional["Controller"] = None

