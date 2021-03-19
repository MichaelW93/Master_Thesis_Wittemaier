import carla

from implementation.knowledge.environment_knowledge import EnvironmentKnowledge
from implementation.knowledge.base_attribute import *


class CarlaMonitor(object):

    def __init__(self, carla_world: carla.World, ego_vehicle: carla.Vehicle, leader_vehicle: carla.Vehicle) -> None:
        self.ego_vehicle = ego_vehicle
        self.leader_vehicle = leader_vehicle
        self.carla_world = carla_world

    def run_step(self, timestamp: carla.Timestamp, connection_strength: float,
                 leader_emergency_brake: bool, speed_limit: float):
        environment_knowledge = self.create_environment_knowledge(timestamp, connection_strength,
                                                                  leader_emergency_brake, speed_limit)

        if DEBUG_MODE:
            print(environment_knowledge)

    def create_environment_knowledge(self, timestamp: carla.Timestamp, connection_strength: float,
                 leader_emergency_brake: bool, speed_limit: float) -> EnvironmentKnowledge:

        environment_knowledge = EnvironmentKnowledge()
        environment_knowledge.timestamp = timestamp
        environment_knowledge.connection_strength = connection_strength

        environment_knowledge.ego_velocity = self.__process_ego_velocity()
        environment_knowledge.ego_acceleration = self.__process_ego_acceleration()
        environment_knowledge.ego_distance = self.__process_ego_distance()

        environment_knowledge.other_acceleration = self.__process_other_acceleration()
        environment_knowledge.other_velocity = self.__process_other_velocity()
        environment_knowledge.other_braking_light = self.__process_other_braking_light()
        environment_knowledge.other_emergency_brake = leader_emergency_brake

        environment_knowledge.speed_limit = speed_limit
        environment_knowledge.weather = self.__process_weather()

        return environment_knowledge

    def __process_ego_velocity(self) -> tuple:

        current_ego_velocity = self.ego_vehicle.get_velocity()
        ego_speed = (self.calculate_vehicle_speed(current_ego_velocity), EGO_SPEED_RANGE)
        return ego_speed

    def __process_ego_acceleration(self) -> tuple:

        current_ego_acceleration = self.ego_vehicle.get_accelaration()
        ego_acceleration = (current_ego_acceleration, EGO_ACCELERATION_RANGE)
        return ego_acceleration

    def __process_ego_distance(self) -> float:

        distance = 0

        return distance
        pass

    def __process_other_acceleration(self) -> tuple:
        current_ego_acceleration = self.leader_vehicle.get_accelaration()
        leader_acceleration = (current_ego_acceleration, EGO_ACCELERATION_RANGE)
        return leader_acceleration

    def __process_other_braking_light(self) -> bool:
        light_state = self.leader_vehicle.get_light_state()
        braking_light = light_state.brake
        return braking_light

    def __process_other_velocity(self) -> tuple:
        other_current_velocity = self.leader_vehicle.get_velocity()
        other_speed = (self.calculate_vehicle_speed(other_current_velocity), OTHER_VELOCITY_RANGE)
        return other_speed

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
        """Takes an carla velocity vector and transforms it into an speed value [m/]
        :param velocity: velocity vector
        :type velocity: carla.Vector3D
        :return: speed of vector in [m/s]
        :rtype: float
        """

        speed = velocity.x * 2 + velocity.y * 2 + velocity.z * 2
        return speed

    def destroy(self):
        pass