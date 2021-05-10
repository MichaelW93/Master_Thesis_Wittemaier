import carla
import math

from typing import Optional, Tuple, TYPE_CHECKING, Union, List, Dict

from implementation.platoon_controller.knowledge.base_attribute import *
from implementation.data_classes import *

if TYPE_CHECKING:
    from implementation.vehicle.vehicles import ManagedVehicle
    from implementation.data_classes import CommunicationData


class Monitor(object):

    def __init__(self, knowledge, ego_vehicle) -> None:
        self.leader_acceleration = None
        self.ego_acceleration = None
        self.ego_distance = None
        self.new_distance_received = False
        self.knowledge = knowledge
        self.ego_vehicle: "ManagedVehicle" = ego_vehicle
        self.communication_data: Dict[int, "CommunicationData"] = {}

    def run_step(self, timestamp: carla.Timestamp, weather: carla.WeatherParameters, speed_limit: float):

        sensor_data = self.ego_vehicle.get_sensor_data()
        ego_distance_data, ego_acc_data = self.process_sensor_data(sensor_data)
        ego_velocity = self.ego_vehicle.get_velocity()
        ego_speed = self.calculate_vehicle_speed(ego_velocity)

        # Get all currently available communication_data
        self.communication_data: Dict[int, CommunicationData] = {}
        self.communication_data = self.ego_vehicle.comm_handler.get_all_vehicle_data()

        # Get own front vehicle
        self.check_platoon(ego_distance_data)

        environment_knowledge = self.create_environment_knowledge(ego_distance_data,
                                                                  ego_acc_data,
                                                                  ego_speed,
                                                                  timestamp,
                                                                  weather,
                                                                  speed_limit)
        self.knowledge.store_environment_knowledge(environment_knowledge)
        comm_data = self.built_communication_data(timestamp, environment_knowledge.ego_speed[0], environment_knowledge.ego_acceleration[0])
        self.ego_vehicle.send_communication_data(comm_data)

        if DEBUG_MODE:
            print(environment_knowledge.__str__())
        if self.ego_vehicle.role_name == "Follower_1":
            print("Knowledge, other vehicles: ", self.knowledge.other_vehicles)
            print("Environment_knowledge: ", environment_knowledge)
        return environment_knowledge

    def built_communication_data(self, timestamp: carla.Timestamp, speed: float, acceleration: float) -> "CommunicationData":
        data = CommunicationData()
        data.front_id = self.knowledge.front_vehicle_id
        data.leader_id = self.knowledge.leader_id
        data.timestamp = timestamp
        data.vehicle_id = self.ego_vehicle.ego_vehicle.id
        data.speed = speed
        data.acceleration = acceleration

        return data

    def check_platoon(self, data: Union[carla.ObstacleDetectionEvent, int]):
        """Returns the Id of the current front vehicle, if one exists"""
        if data == -1:
            # No front vehicle detected
            self.knowledge.front_vehicle_id = -1
            self.knowledge.leader_id = -1
            self.knowledge.other_vehicles = {}
        else:
            front_id = data.other_actor.id
            self.knowledge.leader_id = front_id
            self.knowledge.front_vehicle_id = front_id
            if front_id in self.knowledge.other_vehicles:
                self.knowledge.other_vehicles[front_id].is_front_vehicle = True
                self.knowledge.other_vehicles[front_id].is_leader = True
            else:
                vehicle = OtherVehicle(is_front_vehicle=True, is_leader=True, id=front_id)
                self.knowledge.other_vehicles[front_id] = vehicle

        front_id = self.knowledge.front_vehicle_id
        while front_id != -1:
            if front_id in self.communication_data:
                # Platoonable vehicle
                front_data: CommunicationData = self.communication_data[front_id]
                if front_id in self.knowledge.other_vehicles:
                    self.knowledge.other_vehicles[front_id].platoonable = True
                if front_data.front_id == -1:
                    # Vehicle has no front_vehicle --> is Leading vehicle
                    if front_id in self.knowledge.other_vehicles:
                        # Vehicle is already known
                        self.knowledge.leader_id = front_id
                        self.knowledge.other_vehicles[front_id].is_leader = True
                        front_id = -1
                    else:
                        # Vehicle is not known --> create a new one
                        vehicle = OtherVehicle(id=front_id, platoonable=True, is_leader=True)
                        self.knowledge.other_vehicles[front_id] = vehicle
                else:
                    # go to the next vehicle
                    if front_id in self.knowledge.other_vehicles:
                        self.knowledge.other_vehicles[front_id].is_leader = False
                    front_id = front_data.front_id
            else:
                # Vehicle is not platoonable
                if front_id in self.knowledge.other_vehicles:
                    # Vehicle is already known --> just update its information
                    self.knowledge.leader_id = front_id
                    self.knowledge.other_vehicles[front_id].is_leader = True
                else:
                    # vehicle is not known --> create a new one
                    vehicle = OtherVehicle(id=front_id, platoonable=False, is_leader=True)
                    self.knowledge.other_vehicles[front_id] = vehicle
                front_id = -1

    def check_communication_data(self, comm_data: "CommunicationData", previous_timestamp):
        acc_data = self.__check_acceleration_for_failures


    def process_sensor_data(self, sensor_data: List[carla.SensorData]) -> \
        Tuple[Union[carla.ObstacleDetectionEvent, int], carla.IMUMeasurement]:
        imu_data = None
        distance_data = None

        for data in sensor_data:
            if isinstance(data, carla.IMUMeasurement):
                imu_data = data
            elif isinstance(data, carla.ObstacleDetectionEvent):
                distance_data = data

        # No front vehicle in range
        if distance_data is None:
            distance_data = -1

        return distance_data, imu_data

    def create_environment_knowledge(self, ego_distance: carla.ObstacleDetectionEvent,
                                     ego_acc: carla.IMUMeasurement,
                                     ego_speed: float,
                                     timestamp: carla.Timestamp,
                                     weather: carla.WeatherParameters,
                                     speed_limit: float) -> EnvironmentKnowledge:

        #print("Monitor input data at monitor:", monitor_input_data)

        environment_knowledge = EnvironmentKnowledge()
        previous_environment_knowledge: EnvironmentKnowledge = self.knowledge.get_current_environment_knowledge()

        environment_knowledge.ego_name = self.ego_vehicle.role_name
        environment_knowledge.timestamp = timestamp
        if len(self.communication_data) > 0:
            environment_knowledge.communication_delay = timestamp.elapsed_seconds - list(self.communication_data.values())[0].timestamp.elapsed_seconds
        environment_knowledge.time_to_last_step = timestamp.elapsed_seconds - \
                                                  previous_environment_knowledge.timestamp.elapsed_seconds

        environment_knowledge.ego_speed = self.__check_speed_for_failures(
            ego_speed,
            previous_environment_knowledge.ego_speed,
            previous_environment_knowledge.ego_acceleration,
            previous_environment_knowledge.timestamp,
            timestamp)

        environment_knowledge.ego_acceleration = self.__check_acceleration_for_failures(
            self.__process_acceleration(ego_acc),
            timestamp,
            previous_environment_knowledge.timestamp)

        for _, vehicle in self.knowledge.other_vehicles.items():
            if vehicle.id in self.communication_data:
                vehicle_comm_data = self.communication_data[vehicle.id]
                vehicle_acc = self.__check_acceleration_for_failures(
                    vehicle_comm_data.acceleration, timestamp,
                    previous_environment_knowledge.timestamp)

                environment_knowledge.other_vehicles[vehicle.id] = vehicle

                if self.knowledge.front_vehicle_id in previous_environment_knowledge.other_vehicles:
                    vehicle_speed = self.__check_speed_for_failures(
                        vehicle_comm_data.speed,
                        previous_environment_knowledge.other_vehicles[vehicle.id].speed,
                        previous_environment_knowledge.other_vehicles[vehicle.id].acceleration,
                        previous_environment_knowledge.timestamp,
                        timestamp)
                    environment_knowledge.other_vehicles[vehicle.id].speed = vehicle_speed
                environment_knowledge.other_vehicles[vehicle.id].acceleration = vehicle_acc

        if self.knowledge.front_vehicle_id in previous_environment_knowledge.other_vehicles:
            environment_knowledge.ego_distance = self.__check_distance_for_failure(
                ego_distance,
                previous_environment_knowledge.ego_speed,
                previous_environment_knowledge.ego_acceleration,
                previous_environment_knowledge.other_vehicles[self.knowledge.front_vehicle_id].acceleration,
                previous_environment_knowledge.other_vehicles[self.knowledge.front_vehicle_id].speed,
                previous_environment_knowledge.ego_distance,
                timestamp,
                previous_environment_knowledge.timestamp)

        environment_knowledge.speed_limit = speed_limit
        environment_knowledge.weather = weather
        environment_knowledge.front_vehicle_id = self.knowledge.front_vehicle_id
        environment_knowledge.leader_id = self.knowledge.leader_id

        return environment_knowledge

    @staticmethod
    def __process_acceleration(sensor_data: Optional[carla.IMUMeasurement]) -> Optional[float]:

        if sensor_data is not None:
            limits = (-99.9, 99.9)
            acceleration = max(limits[0], min(limits[1], sensor_data.accelerometer.x))
            return acceleration
        else:
            return None

    @staticmethod
    def __check_speed_for_failures(speed_data: Optional[float],
                                   previous_speed: Tuple[Optional[float], FailureType],
                                   previous_acceleration: Tuple[Optional[float], FailureType],
                                   previous_timestamp: carla.Timestamp,
                                   current_timestamp: carla.Timestamp) \
            -> Tuple[Optional[float], FailureType]:
        """"""
        """Omission Check"""
        if speed_data is None:
            speed = (None, FailureType.omission)
            return speed

        speed_threshold = speed_data * SPEED_THRESHOLD_FACTOR
        if None not in {previous_speed[0], previous_acceleration[0]}:
            """All variables are available"""
            expected_speed = previous_speed[0] + previous_acceleration[0] * (current_timestamp.elapsed_seconds -
                                                                                 previous_timestamp.elapsed_seconds)
        else:
            expected_speed = speed_data

        """Check for delay"""
        if (0.05 - DELAY_THRESHOLD) <= (current_timestamp.elapsed_seconds - previous_timestamp.elapsed_seconds) \
                <= (0.05 + DELAY_THRESHOLD):
            if (expected_speed - speed_threshold) <= speed_data <= (expected_speed + speed_threshold):
                speed = (speed_data, FailureType.no_failure)
                return speed
            else:
                speed = (speed_data, FailureType.faulty_value)
                return speed
        else:
            if (expected_speed - speed_threshold) <= speed_data <= (expected_speed + speed_threshold):
                speed = (speed_data, FailureType.delay)
                return speed
            else:
                speed = (speed_data, FailureType.faulty_delayed)
                return speed

    @staticmethod
    def __check_acceleration_for_failures(acceleration: Optional[float],
                                          current_timestamp: carla.Timestamp,
                                          previous_timestamp: carla.Timestamp) \
            -> Tuple[Optional[float], FailureType]:
        """"""
        """Check for omission"""
        if acceleration is None:
            acceleration_data = (None, FailureType.omission)
            return acceleration_data

        """Check for delay"""
        if (0.05 - DELAY_THRESHOLD) <= (current_timestamp.elapsed_seconds - previous_timestamp.elapsed_seconds) <= (
                0.05 + DELAY_THRESHOLD):
            acceleration_data = (acceleration, FailureType.no_failure)
        else:
            acceleration_data = (acceleration, FailureType.delay)

        """Check for faulty value"""
        if MAX_DECELERATION <= acceleration <= MAX_ACCELERATION:
            if acceleration_data[1] == FailureType.no_failure:
                acceleration_data = (acceleration, FailureType.no_failure)
            else:
                acceleration_data = (acceleration, FailureType.delay)
        else:
            if acceleration_data[1] == FailureType.no_failure:
                acceleration_data = (acceleration, FailureType.faulty_value)
            else:
                acceleration_data = (acceleration, FailureType.faulty_delayed)
        return acceleration_data

    def __check_distance_for_failure(self, sensor_data: Union[carla.ObstacleDetectionEvent, float, None],
                                     ego_vehicle_previous_speed: Tuple[Optional[float], FailureType],
                                     ego_vehicle_previous_acceleration: Tuple[Optional[float], FailureType],
                                     front_vehicle_previous_acceleration: Tuple[Optional[float], FailureType],
                                     front_vehicle_previous_speed: Tuple[Optional[float], FailureType],
                                     previous_distance: Tuple[Optional[float], FailureType],
                                     current_timestamp: carla.Timestamp,
                                     previous_timestamp: carla.Timestamp)\
            -> Tuple[Optional[float], FailureType]:

        if sensor_data is None:
            ego_distance = (None, FailureType.omission)
            return ego_distance

        if sensor_data == -1:
            ego_distance = (-1, FailureType.no_front_vehicle)
            return ego_distance

        # TODO check for new vehicle

        delta_time = current_timestamp.elapsed_seconds - previous_timestamp.elapsed_seconds
        v_fv: float = front_vehicle_previous_speed[0]
        a_fv: float = front_vehicle_previous_acceleration[0]

        v_ev: float = ego_vehicle_previous_speed[0]
        a_ev: float = ego_vehicle_previous_acceleration[0]

        p_dist = previous_distance[0]

        if None not in {v_fv, a_fv, v_ev, a_ev, p_dist}:
            expected_distance = (p_dist + ((v_fv + 0.5 * a_fv * delta_time) - (v_ev + 0.5 * a_ev * delta_time)) * delta_time)
        else:
            expected_distance = sensor_data.distance

        if (0.05 - DELAY_THRESHOLD) <= (current_timestamp.elapsed_seconds - previous_timestamp.elapsed_seconds) <= (
                0.05 + DELAY_THRESHOLD):
            if (expected_distance - DISTANCE_THRESHOLD_FACTOR) <= sensor_data.distance <= \
                    (expected_distance + DISTANCE_THRESHOLD_FACTOR):
                ego_distance = (sensor_data.distance, FailureType.no_failure)
                return ego_distance
            else:
                ego_distance = (sensor_data.distance, FailureType.faulty_value)
                return ego_distance
        else:
            if (expected_distance * (1 - DISTANCE_THRESHOLD_FACTOR)) <= sensor_data.distance <= \
                    (expected_distance * (1 + DISTANCE_THRESHOLD_FACTOR)):
                ego_distance = (sensor_data.distance, FailureType.delay)
                return ego_distance
            else:
                ego_distance = (sensor_data.distance, FailureType.faulty_delayed)
                return ego_distance

    def __process_other_braking_light(self, other_braking_light: Optional[bool]) -> bool:
        if other_braking_light is not None:
            return other_braking_light
        else:
            print("No braking light available")
            # TODO


    @staticmethod
    def calculate_vehicle_speed(velocity: carla.Vector3D) -> float:
        """Takes an carla velocity vector and transforms it into an speed value [m/s]
        :param velocity: velocity vector
        :type velocity: carla.Vector3D
        :return: speed of vector in [m/s]
        :rtype: float
        """

        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

    def destroy(self):
        pass