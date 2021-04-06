import carla
import math

from typing import Optional, Tuple

from implementation.knowledge.environment_knowledge import EnvironmentKnowledge
from implementation.knowledge.base_attribute import *
from implementation.carla_client.simulation_state import SimulationState
from implementation.monitor.monitor_input_data import MonitorInputData


class CarlaMonitor(object):

    def __init__(self, carla_world: carla.World, ego_vehicle: carla.Vehicle, leader_vehicle: carla.Vehicle) -> None:
        self.ego_vehicle = ego_vehicle
        self.leader_vehicle = leader_vehicle
        self.carla_world = carla_world

        self.leader_acceleration = None
        self.ego_acceleration = None
        self.ego_distance = None
        self.new_distance_received = False

    def run_step(self, monitor_input_data: MonitorInputData):

        if self.ego_vehicle is not None and self.leader_vehicle is not None:
            environment_knowledge = self.create_environment_knowledge(monitor_input_data)

            print(environment_knowledge.__str__())
            if DEBUG_MODE:
                print(environment_knowledge.__str__())

    def create_environment_knowledge(self, monitor_input_data: MonitorInputData) -> EnvironmentKnowledge:

        environment_knowledge = EnvironmentKnowledge()
        environment_knowledge.timestamp = monitor_input_data.timestamp
        environment_knowledge.connection_strength = monitor_input_data.connection_strength
        environment_knowledge.ego_speed = self.__process_ego_speed(monitor_input_data.ego_speed)
        environment_knowledge.ego_distance = self.__process_ego_distance(monitor_input_data.ego_distance)
        environment_knowledge.ego_acceleration = self.__process_ego_acceleration(monitor_input_data.ego_acceleration)
        environment_knowledge.other_speed = self.__process_other_speed(monitor_input_data.other_speed)
        environment_knowledge.other_acceleration = self.__process_other_acceleration(monitor_input_data.other_acceleration)
        environment_knowledge.other_braking_light = self.__process_other_braking_light(monitor_input_data.other_braking_light)
        environment_knowledge.other_emergency_brake = monitor_input_data.other_emergency_brake

        environment_knowledge.speed_limit = monitor_input_data.speed_limit
        environment_knowledge.weather = self.__process_weather()

        return environment_knowledge

    def __process_ego_speed(self, ego_speed_data) -> Optional[Tuple[float, float]]:
        if ego_speed_data is not None:
            ego_speed = (ego_speed_data, EGO_SPEED_RANGE)
            return ego_speed
        else:
            print("No ego speed available")
            # TODO

    def __process_ego_acceleration(self, sensor_data: carla.IMUMeasurement) -> Optional[Tuple[float, float]]:
        if sensor_data is not None:
            limits = (-99.9, 99.9)
            current_ego_acceleration = max(limits[0], min(limits[1], sensor_data.accelerometer.x))
            ego_acceleration = (current_ego_acceleration, EGO_ACCELERATION_RANGE)
            return ego_acceleration
        else:
            print("No ego acceleration available")
            # TODO

    def __process_ego_distance(self, sensor_data: carla.ObstacleDetectionEvent) -> Optional[Tuple[float, float]]:

        if sensor_data is not None:
            if sensor_data.other_actor.id == self.leader_vehicle.id:
                return sensor_data.distance
            else:
                return None
        else:
            print("No distance data received")
            # TODO

    def __process_other_acceleration(self, sensor_data: carla.IMUMeasurement) -> Optional[Tuple[float, float]]:
        if sensor_data is not None:
            limits = (-99.9, 99.9)
            current_other_acceleration = max(limits[0], min(limits[1], sensor_data.accelerometer.x))
            leader_acceleration = (current_other_acceleration, OTHER_ACCELERATION_RANGE)
            return leader_acceleration
        else:
            print("No other acceleration available")
            # TODO

    def __process_other_braking_light(self, other_braking_light: Optional[bool]) -> bool:
        if other_braking_light is not None:
            return other_braking_light
        else:
            print("No braking light available")
            # TODO

    def __process_other_speed(self, other_speed) -> Optional[Tuple[float, float]]:
        if other_speed is not None:
            other_speed = (other_speed, OTHER_VELOCITY_RANGE)
            return other_speed
        else:
            print("No other speed available")
            # TODO

    def __process_weather(self) -> Weather:
        weather = self.carla_world.get_weather()
        if weather.precipitation > 60.0:
            return Weather.RAIN
        elif weather.wetness > 60.0:
            return Weather.RAIN
        else:
            return Weather.SUNSHINE

    def reset(self) -> None:
        if self.ego_vehicle is not None:
            self.ego_vehicle = None
        if self.leader_vehicle is not None:
            self.leader_vehicle = None
        if self.carla_world is not None:
            self.carla_world = None

    def calculate_vehicle_speed(self, velocity: carla.Vector3D) -> float:
        """Takes an carla velocity vector and transforms it into an speed value [m/s]
        :param velocity: velocity vector
        :type velocity: carla.Vector3D
        :return: speed of vector in [m/s]
        :rtype: float
        """

        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

    def destroy(self):
        pass