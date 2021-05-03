import carla
import math

from typing import Optional, Tuple, TYPE_CHECKING, Union

from implementation.platoon_controller.knowledge.base_attribute import *
from implementation.data_classes import EnvironmentKnowledge, MonitorInputData, FailureType

if TYPE_CHECKING:
    from implementation.vehicle.vehicles import ManagedVehicle


class Monitor(object):

    def __init__(self, knowledge, ego_vehicle) -> None:
        self.leader_acceleration = None
        self.ego_acceleration = None
        self.ego_distance = None
        self.new_distance_received = False
        self.knowledge = knowledge
        self.ego_vehicle: "ManagedVehicle" = ego_vehicle

    def run_step(self, monitor_input_data: MonitorInputData):

        environment_knowledge = self.create_environment_knowledge(monitor_input_data)
        self.knowledge.store_environment_knowledge(environment_knowledge)
        if DEBUG_MODE:
            print(environment_knowledge.__str__())
        return environment_knowledge

    def create_environment_knowledge(self, monitor_input_data: MonitorInputData) -> EnvironmentKnowledge:

        #print("Monitor input data at monitor:", monitor_input_data)
        previous_environment_knowledge: EnvironmentKnowledge = self.knowledge.get_current_environment_knowledge()
        environment_knowledge = EnvironmentKnowledge()
        environment_knowledge.ego_name = self.ego_vehicle.role_name
        environment_knowledge.timestamp = monitor_input_data.timestamp
        environment_knowledge.connection_strength = monitor_input_data.connection_strength
        environment_knowledge.time_to_last_step = monitor_input_data.timestamp.elapsed_seconds - \
                                                previous_environment_knowledge.timestamp.elapsed_seconds

        environment_knowledge.ego_speed = self.__check_speed_for_failures(
            monitor_input_data.ego_vehicle_speed,
            previous_environment_knowledge.ego_speed,
            previous_environment_knowledge.ego_acceleration,
            previous_environment_knowledge.timestamp,
            monitor_input_data.timestamp)

        environment_knowledge.ego_acceleration = self.__check_acceleration_for_failures(
            self.__process_acceleration(monitor_input_data.ego_vehicle_acceleration),
            monitor_input_data.timestamp,
            previous_environment_knowledge.timestamp)

        if self.ego_vehicle.front_vehicle_is_leader:
            front_vehicle_previous_speed = previous_environment_knowledge.leader_speed
            front_vehicle_previous_acceleration = previous_environment_knowledge.leader_acceleration
        else:
            if len(previous_environment_knowledge.front_vehicles_speed) > 0:
                front_vehicle_previous_speed = previous_environment_knowledge.front_vehicles_speed[-1]
            else:
                front_vehicle_previous_speed = (None, FailureType.omission)

            if len(previous_environment_knowledge.front_vehicles_acceleration) > 0:
                front_vehicle_previous_acceleration = previous_environment_knowledge.front_vehicles_acceleration[-1]
            else:
                front_vehicle_previous_acceleration = (None, FailureType.omission)

        environment_knowledge.ego_distance = self.__check_distance_for_failure(
            monitor_input_data.ego_vehicle_distance,
            previous_environment_knowledge.ego_speed,
            previous_environment_knowledge.ego_acceleration,
            front_vehicle_previous_acceleration,
            front_vehicle_previous_speed,
            previous_environment_knowledge.ego_distance,
            monitor_input_data.timestamp,
            previous_environment_knowledge.timestamp)

        environment_knowledge.leader_speed = self.__check_speed_for_failures(
            monitor_input_data.leader_speed,
            previous_environment_knowledge.leader_speed,
            previous_environment_knowledge.leader_acceleration,
            previous_environment_knowledge.timestamp,
            monitor_input_data.timestamp
        )

        environment_knowledge.leader_acceleration = self.__check_acceleration_for_failures(
            monitor_input_data.leader_acceleration,
            monitor_input_data.timestamp,
            previous_environment_knowledge.timestamp
        )

        if not self.ego_vehicle.front_vehicle_is_leader:
            for i in range(len(self.ego_vehicle.front_vehicles)):
                if len(previous_environment_knowledge.front_vehicles_speed) > 0:
                    front_vehicle_speed = self.__check_speed_for_failures(
                        monitor_input_data.front_vehicles_speed[i],
                        previous_environment_knowledge.front_vehicles_speed[i],
                        previous_environment_knowledge.front_vehicles_acceleration[i],
                        previous_environment_knowledge.timestamp,
                        monitor_input_data.timestamp
                    )
                else:
                    front_vehicle_speed = (0, FailureType.omission)
                environment_knowledge.front_vehicles_speed.append(front_vehicle_speed)

                front_vehicle_acceleration = self.__check_acceleration_for_failures(
                    monitor_input_data.front_vehicles_acceleration[i],
                    monitor_input_data.timestamp,
                    previous_environment_knowledge.timestamp
                )
                environment_knowledge.front_vehicles_acceleration.append(front_vehicle_acceleration)

        environment_knowledge.speed_limit = monitor_input_data.speed_limit
        environment_knowledge.weather = monitor_input_data.weather

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