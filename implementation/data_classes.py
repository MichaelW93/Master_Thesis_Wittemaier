import carla

from typing import Optional, List, Tuple, TYPE_CHECKING, Union, Dict
from dataclasses import dataclass, field

from implementation.platoon_controller.knowledge.base_attribute import *
from implementation.util import *

if TYPE_CHECKING:
    from implementation.vehicle.controller import Controller


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


@dataclass()
class CommunicationData:
    vehicle_id: int = -1
    timestamp: carla.Timestamp = None
    leader_id: int = -1
    front_id: int = -1
    speed: float = -1
    acceleration: float = -1


class SystemState(object):

    def __init__(self):
        self.controller: Optional["Controller"] = None

