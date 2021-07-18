import carla
import math
import statistics
from typing import Optional, Tuple, TYPE_CHECKING, Union, List, Dict

from implementation.platoon_controller.knowledge.base_attribute import *
from implementation.data_classes import *

import random

if TYPE_CHECKING:
    from implementation.vehicle.vehicles import ManagedVehicle
    from implementation.data_classes import CommunicationData


class Monitor(object):

    def __init__(self, knowledge, ego_vehicle, carla_map) -> None:
        self.leader_acceleration = None
        self.ego_acceleration = None
        self.ego_distance = None
        self.new_distance_received = False
        self.carla_map = carla_map
        self.knowledge = knowledge
        self.ego_vehicle: "ManagedVehicle" = ego_vehicle
        self.communication_data: Dict[int, "CommunicationData"] = {}
        self.environment_knowledge = None

    def run_step(self, timestamp: carla.Timestamp, weather: carla.WeatherParameters, speed_limit: float):

        sensor_data = self.ego_vehicle.get_sensor_data()
        ego_distance_data, ego_acc_data = self.process_sensor_data(sensor_data)
        ego_velocity = self.ego_vehicle.get_velocity()
        ego_speed = self.calculate_vehicle_speed(ego_velocity)

        # Get all currently available communication_data
        self.communication_data: Dict[int, CommunicationData] = {}
        self.communication_data = self.ego_vehicle.comm_handler.get_all_vehicle_data()

        # Get own front vehicle
        ego_distance_data = self.process_distance()
        self.check_platoon(ego_distance_data)

        environment_knowledge = self.create_environment_knowledge(ego_distance_data,
                                                                  ego_acc_data,
                                                                  ego_speed,
                                                                  timestamp,
                                                                  weather,
                                                                  speed_limit)
        self.knowledge.store_environment_knowledge(environment_knowledge)

        if DEBUG_MODE:
            print(environment_knowledge.__str__())

        return environment_knowledge

    def check_platoon(self, data: Tuple[int, int]):
        """Returns the Id of the current front vehicle, if one exists"""
        other_vehicles = {}
        if data[0] == -1:
            # No front vehicle detected
            self.knowledge.front_vehicle_id = -1
            self.knowledge.leader_id = -1
            self.knowledge.other_vehicles = other_vehicles
            print("No front vehicle detected")
            return
        else:
            # Create front vehicle
            front_id = data[1]
            self.knowledge.leader_id = front_id
            self.knowledge.front_vehicle_id = front_id
            vehicle = OtherVehicle(is_front_vehicle=True, is_leader=True, id=front_id)
            other_vehicles[front_id] = vehicle

        front_id = self.knowledge.front_vehicle_id
        while front_id != -1:
            if front_id in self.communication_data:
                # Platoonable vehicle
                front_data: CommunicationData = self.communication_data[front_id]
                if front_id in other_vehicles:
                    other_vehicles[front_id].platoonable = True
                if front_data is None or front_data.front_id == -1 or front_data.front_id is None:
                    # Vehicle has no front_vehicle --> is Leading vehicle
                    if front_id in other_vehicles:
                        # Vehicle is already known
                        self.knowledge.leader_id = front_id
                        other_vehicles[front_id].is_leader = True
                        front_id = -1
                    else:
                        # Vehicle is not known --> create a new one
                        vehicle = OtherVehicle(id=front_id, platoonable=True, is_leader=True)
                        other_vehicles[front_id] = vehicle
                else:
                    # go to the next vehicle
                    if front_id in other_vehicles:
                        other_vehicles[front_id].is_leader = False
                    front_id = front_data.front_id
            else:
                # Vehicle is not platoonable
                if front_id in other_vehicles:
                    # Vehicle is already known --> just update its information
                    self.knowledge.leader_id = front_id
                    other_vehicles[front_id].is_leader = True
                    other_vehicles[front_id].platoonable = False
                else:
                    # vehicle is not known --> create a new one
                    vehicle = OtherVehicle(id=front_id, platoonable=False, is_leader=True)
                    other_vehicles[front_id] = vehicle
                front_id = -1
        self.knowledge.other_vehicles = other_vehicles

    def process_distance(self) -> Tuple[int, int]:

        ego_location = self.ego_vehicle.ego_vehicle.get_location()

        dist_to_front = 1000
        front_id = -1

        ego_name = self.ego_vehicle.role_name
        ego_lane_id = self.carla_map.get_waypoint(self.ego_vehicle.ego_vehicle.get_location()).lane_id

        for vehicle in self.ego_vehicle.other_vehicles:

            veh_loc = vehicle.ego_vehicle.get_location()

            dist = math.sqrt((ego_location.x - veh_loc.x) ** 2 +
                             (ego_location.y - veh_loc.y) ** 2 +
                             (ego_location.z - veh_loc.z) ** 2)
            if ego_name == "Follower_0":
                if vehicle.role_name == "Follower_1":
                    dist = 200
            if vehicle.role_name == "Cut In Vehicle 1":
                vehicle_lane_id = self.carla_map.get_waypoint(vehicle.ego_vehicle.get_location()).lane_id
                print("got vehicles lane id")
                if ego_lane_id != vehicle_lane_id:
                    print("not on the same lane")
                    dist = 200

            if dist < dist_to_front:
                dist_to_front = dist
                front_id = vehicle.ego_vehicle.id

        dist_to_front = dist_to_front - (self.ego_vehicle.ego_vehicle.bounding_box.extent.x * 2) + 0.5

        if dist_to_front > 100 or dist_to_front < 0:
            dist_to_front = -1
            front_id = -1

        return dist_to_front, front_id


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

        environment_knowledge = EnvironmentKnowledge()
        self.environment_knowledge = environment_knowledge
        previous_environment_knowledge: EnvironmentKnowledge = self.knowledge.get_current_environment_knowledge()

        environment_knowledge.ego_name = self.ego_vehicle.role_name
        environment_knowledge.timestamp = timestamp
        if len(self.communication_data) > 0:
            environment_knowledge.communication_delay = timestamp.elapsed_seconds - list(self.communication_data.values())[0].timestamp.elapsed_seconds
        environment_knowledge.time_to_last_step = timestamp.elapsed_seconds - \
                                                  previous_environment_knowledge.timestamp.elapsed_seconds

        environment_knowledge.ego_speed_tuple = self.__check_speed_for_failures(
                                                    ego_speed,
                                                    previous_environment_knowledge.ego_speed_tuple,
                                                    previous_environment_knowledge.ego_acceleration_tuple,
                                                    previous_environment_knowledge.timestamp,
                                                    timestamp)

        environment_knowledge.ego_acceleration_tuple = self.__check_acceleration_for_failures(
                                                            self.__process_acceleration(ego_acc),
                                                            timestamp,
                                                            previous_environment_knowledge.timestamp)

        environment_knowledge.speed_limit = speed_limit
        environment_knowledge.speed_over_limit = ego_speed * 3.6 - environment_knowledge.speed_limit
        environment_knowledge.weather = weather
        environment_knowledge.front_vehicle_id = self.knowledge.front_vehicle_id
        environment_knowledge.leader_id = self.knowledge.leader_id

        for _, vehicle in self.knowledge.other_vehicles.items():
            # add known vehicles to the current env_knowledge
            environment_knowledge.other_vehicles[vehicle.id] = vehicle
            if vehicle.id in self.communication_data:
                # if comm data available, add information to env_knowledge after safety check
                vehicle_comm_data = self.communication_data[vehicle.id]
                if vehicle.id in previous_environment_knowledge.other_vehicles:
                    previous_speed_tuple = previous_environment_knowledge.other_vehicles[vehicle.id].speed_tuple
                    previous_acc_tuple = previous_environment_knowledge.other_vehicles[vehicle.id].acceleration_tuple
                else:
                    previous_speed_tuple = (None, FailureType.no_failure)
                    previous_acc_tuple = (None, FailureType.no_failure)
                vehicle_speed = self.__check_speed_for_failures(
                                    vehicle_comm_data.speed,
                                    previous_speed_tuple,
                                    previous_acc_tuple,
                                    previous_environment_knowledge.timestamp,
                                    timestamp)
                vehicle_acc = self.__check_acceleration_for_failures(
                                    vehicle_comm_data.acceleration, timestamp,
                                    previous_environment_knowledge.timestamp)

                vehicle.speed_tuple = vehicle_speed
                vehicle.acceleration_tuple = vehicle_acc
                vehicle.steering = vehicle_comm_data.steering
                vehicle.brake = vehicle_comm_data.brake
                vehicle.throttle = vehicle_comm_data.throttle

                if vehicle.is_front_vehicle:
                    if self.knowledge.front_vehicle_id in previous_environment_knowledge.other_vehicles:
                        previous_front_vehicle = previous_environment_knowledge.other_vehicles[vehicle.id]

                        environment_knowledge.ego_distance_tuple = self.__check_distance_for_failure(
                            ego_distance,
                            previous_environment_knowledge.ego_speed_tuple,
                            previous_environment_knowledge.ego_acceleration_tuple,
                            previous_front_vehicle.acceleration_tuple,
                            previous_front_vehicle.speed_tuple,
                            previous_environment_knowledge.ego_distance_tuple,
                            timestamp,
                            previous_environment_knowledge.timestamp)
                    else:
                        # Test not possible due to missing previous knowledge -> Occurs only for one timestep -> Not safety critical
                        environment_knowledge.ego_distance_tuple = (ego_distance[0], FailureType.no_failure)

                    if vehicle.speed_tuple[1] == FailureType.no_failure:
                        environment_knowledge.speed_diff_to_front = vehicle.speed_tuple[0] - ego_speed
                        environment_knowledge.front_over_limit = (vehicle.speed_tuple[0]*3.6) - environment_knowledge.speed_limit

                    #if vehicle.platoonable:
                    #    self.knowledge.timegap = 0.5
                    #else:
                    #    self.knowledge.timegap = 1.5

                if vehicle.is_leader:
                    if vehicle.speed_tuple[1] == FailureType.no_failure:
                        environment_knowledge.speed_diff_to_leader = vehicle.speed_tuple[0] - ego_speed

            else:
                # no comm data available
                vehicle.steering = 0
                vehicle.throttle = -1
                vehicle.brake = -1

                environment_knowledge.ego_distance_tuple = (ego_distance[0], FailureType.no_failure)

                if vehicle.is_front_vehicle:
                    if self.knowledge.front_vehicle_id in previous_environment_knowledge.other_vehicles:
                        previous_front_vehicle = previous_environment_knowledge.other_vehicles[vehicle.id]

                        vehicle.measured_acceleration_tuple = self.__process_measured_acceleration(
                            previous_environment_knowledge.ego_distance_tuple[0],
                            environment_knowledge.ego_distance_tuple,
                            previous_environment_knowledge.ego_speed_tuple[0],
                            environment_knowledge.ego_speed_tuple[0],
                            environment_knowledge.time_to_last_step)

                        if previous_front_vehicle.speed_tuple[1] == FailureType.no_failure:
                            previous_speed_tuple = previous_front_vehicle.speed_tuple
                        else:
                            previous_speed_tuple = previous_front_vehicle.measured_speed_tuple
                        vehicle.measured_speed_tuple = self.__process_measured_speed(vehicle.measured_acceleration_tuple,
                                                                                     previous_speed_tuple,
                                                                                     environment_knowledge.time_to_last_step)
                        if vehicle.measured_speed_tuple[1] == FailureType.no_failure:
                            environment_knowledge.speed_diff_to_front = vehicle.measured_speed_tuple[0] - ego_speed
                            environment_knowledge.front_over_limit = vehicle.measured_speed_tuple[0] * 3.6 - environment_knowledge.speed_limit

                        if vehicle.is_leader:
                            environment_knowledge.speed_diff_to_leader = environment_knowledge.speed_diff_to_front

        if environment_knowledge.ego_distance_tuple[1] == FailureType.no_front_vehicle:
            environment_knowledge.front_over_limit = 0
            environment_knowledge.speed_diff_to_front = 100
            environment_knowledge.speed_diff_to_leader = 100

        environment_knowledge.desired_distance = self.__get_min_distance(ego_speed) + self.knowledge.timegap * ego_speed
        if self.ego_vehicle.role_name == "Follower_0":
            print("Desired distance = ", environment_knowledge.desired_distance)
            print("Current set timegap: ", self.knowledge.timegap)

        if environment_knowledge.desired_distance is not None and environment_knowledge.desired_distance != 0 and \
                environment_knowledge.ego_distance_tuple[1] == FailureType.no_failure:
            environment_knowledge.distance_error = abs(environment_knowledge.ego_distance_tuple[0] / environment_knowledge.desired_distance) - 1
        else:
            environment_knowledge.distance_error = 0

        environment_knowledge = self.check_max_values(environment_knowledge)

        return environment_knowledge

    @staticmethod
    def __get_min_distance(ego_speed) -> float:
        if 0 < ego_speed < 5:
            return 2
        elif 5 < ego_speed < 9:
            d = 2 - 0.5 * (ego_speed - 5)
            return d
        else:
            return 0

    def check_max_values(self, env_knowledge: EnvironmentKnowledge) -> EnvironmentKnowledge:
        max_throttle = 0
        max_brake = 0
        max_dec = 0
        max_acc = 0
        other_vehicles = env_knowledge.other_vehicles

        for vehicle_data in other_vehicles.values():
            if vehicle_data.throttle is not None:
                if vehicle_data.throttle > max_throttle:
                    max_throttle = vehicle_data.throttle
            if vehicle_data.brake is not None:
                if vehicle_data.brake > max_brake:
                    max_brake = vehicle_data.brake
            if vehicle_data.acceleration_tuple[0] is not None:
                if vehicle_data.acceleration_tuple[0] < max_dec:
                    max_dec = vehicle_data.acceleration_tuple[0]
            elif vehicle_data.measured_acceleration_tuple[0]:
                if vehicle_data.measured_acceleration_tuple[0] < max_dec:
                    max_dec = vehicle_data.measured_acceleration_tuple[0]
            if vehicle_data.acceleration_tuple[0] is not None:
                if vehicle_data.acceleration_tuple[0] > max_acc:
                    max_acc = vehicle_data.acceleration_tuple[0]
            elif vehicle_data.measured_acceleration_tuple[0]:
                if vehicle_data.measured_acceleration_tuple[0] > max_acc:
                    max_acc = vehicle_data.measured_acceleration_tuple[0]

        env_knowledge.max_acc = max_acc
        env_knowledge.max_dec = max_dec
        env_knowledge.max_throttle = max_throttle
        env_knowledge.max_brake = max_brake

        return env_knowledge

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
                speed = (speed_data, FailureType.no_failure)
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
            acceleration_data = (-1, FailureType.omission)
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

        if sensor_data[0] == -1:
            ego_distance = (-1, FailureType.no_front_vehicle)
            print("monitor did not detect a front vehicle")
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
            expected_distance = sensor_data[0]

        if (0.05 - DELAY_THRESHOLD) <= (current_timestamp.elapsed_seconds - previous_timestamp.elapsed_seconds) <= (
                0.05 + DELAY_THRESHOLD):
            if (expected_distance - DISTANCE_THRESHOLD_FACTOR) <= sensor_data[0] <= \
                    (expected_distance + DISTANCE_THRESHOLD_FACTOR):
                ego_distance = (sensor_data[0], FailureType.no_failure)
                return ego_distance
            else:
                ego_distance = (sensor_data[0], FailureType.faulty_value)
                return ego_distance
        else:
            if (expected_distance * (1 - DISTANCE_THRESHOLD_FACTOR)) <= sensor_data[0] <= \
                    (expected_distance * (1 + DISTANCE_THRESHOLD_FACTOR)):
                ego_distance = (sensor_data[0], FailureType.delay)
                return ego_distance
            else:
                ego_distance = (sensor_data[0], FailureType.faulty_delayed)
                return ego_distance

    def __process_measured_speed(self, measured_acceleration_tuple: Tuple[Optional[float], FailureType],
                                 previous_speed_tuple: Tuple[Optional[float], FailureType], timegap: float) \
            -> Tuple[Optional[float], FailureType]:
        """
        old_values: List["EnvironmentKnowledge"] = self.knowledge.get_multiple_steps(5)
        old_distance = []
        if old_values is not None:
            for old_knowledge in old_values:
                if old_knowledge is not None:
                    if old_knowledge.ego_distance_tuple[0] != -1:
                        old_distance.append(old_knowledge.ego_distance_tuple[0])
        if current_distance[1] == FailureType.no_failure:
            old_distance.append(current_distance[0])
        #if len(old_distance) == 0:
        #    previous_distance = 0
        #else:
        #    previous_distance = statistics.mean(old_distance)
        if ego_speed is not None and previous_distance is not None and timegap is not None:
            speed = ego_speed + (current_distance[0] - previous_distance)/timegap
        else:
            speed = None
        other_speed = (speed, FailureType.no_failure)
        return other_speed
        """

        data_dict = self.ego_vehicle.comm_handler.vehicles_data[1]
        vehicle_data = data_dict[self.knowledge.front_vehicle_id]
        speed_tuple = (vehicle_data.speed, FailureType.no_failure)
        return speed_tuple

        if measured_acceleration_tuple[1] == FailureType.no_failure and previous_speed_tuple[1] == FailureType.no_failure:
            fail_type = FailureType.no_failure
        else:
            fail_type = FailureType.faulty_value

        speed_front = previous_speed_tuple[0] + (measured_acceleration_tuple[0] * timegap)
        speed_front_tuple = (speed_front, fail_type)
        return speed_front_tuple

    def __process_measured_acceleration(self, previous_dist: float, current_dist_tuple: Tuple[Optional[float], FailureType],
                                        previous_ego_speed: float, current_ego_speed: float, timestep) -> \
            Tuple[Optional[float], FailureType]:
        """
        acc = (None, FailureType.omission)
        old_values = self.knowledge.get_multiple_steps(5)
        old_speeds = []
        if old_values is not None:
            for old_knowledge in old_values:
                if old_knowledge is not None and self.knowledge.front_vehicle_id in old_knowledge.other_vehicles:
                    if old_knowledge.other_vehicles[self.knowledge.front_vehicle_id].measured_speed_tuple[1] == FailureType.no_failure:
                        old_speeds.append(old_knowledge.other_vehicles[self.knowledge.front_vehicle_id].measured_speed_tuple[0])
        #if len(old_speeds) == 0:
        #    previous_speed = 0
        #else:
        #    previous_speed = statistics.mean(old_speeds)
        if current_speed is not None and previous_speed is not None and timegap is not None:
            acc = (((current_speed - previous_speed) / timegap), FailureType.no_failure)
        return acc
        """
        data_dict = self.ego_vehicle.comm_handler.vehicles_data[1]
        vehicle_data = data_dict[self.knowledge.front_vehicle_id]
        acceleration_tuple = (vehicle_data.acceleration, FailureType.no_failure)
        return acceleration_tuple
        if current_dist_tuple[1] == FailureType.no_failure:
            fail_type = FailureType.no_failure
        else:
            fail_type = FailureType.faulty_value

        delta_d = previous_dist - current_dist_tuple[0]
        delta_v_ego = previous_ego_speed - current_ego_speed
        acc_ego = delta_v_ego / timestep
        acc_comb = (0.5 * delta_d) * (timestep ** 2)
        acc_front = acc_comb - acc_ego

        acc_front_tuple = (acc_front, fail_type)
        return acc_front_tuple

    @staticmethod
    def __process_other_braking_light(other_braking_light: Optional[bool]) -> bool:
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