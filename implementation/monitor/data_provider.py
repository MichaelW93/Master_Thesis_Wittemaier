import carla
import math
from typing import Optional
from implementation.configuration_parameter import *
from implementation.carla_client.simulation_state import SimulationState
from implementation.monitor.monitor_input_data import MonitorInputData
from implementation.knowledge.base_attribute import *


class DataProvider(object):

    def __init__(self, ego_vehicle, leader_vehicle, carla_world: carla.World, simulation_state: SimulationState):
        self.carla_world = carla_world
        self.delay = 50.0 # in ms
        self.ego_vehicle = ego_vehicle
        self.leader_vehicle = leader_vehicle

        self.simulation_state: SimulationState = simulation_state

        self.new_distance_received = False

        self.ego_imu_sensor_data = None
        self.ego_speed = None
        self.ego_obstacle_sensor_data = None
        self.other_braking_light = None

        # TODO find a better solution
        self.other_acceleration_data = [None, None, None, None, None, None, None, None, None, None]
        self.other_speed_data = [None, None, None, None, None, None, None, None, None, None]
        self.other_emergency_brake_data = [None, None, None, None, None, None, None, None, None, None]

    def calculate_delay_in_simulation_tick(self) -> Optional[int]:
        if self.simulation_state.connection_strength == 100:
            self.delay = 50
        elif self.simulation_state.connection_strength == 75:
            self.delay = 100
        elif self.simulation_state.connection_strength == 50:
            self.delay = 200
        elif self.simulation_state.connection_strength == 25:
            self.delay = 500
        else:
            self.delay = -1
            return None
        delay_in_simulation_ticks = self.delay/((1/CARLA_SERVER_FPS)*1000) # convert to ms
        return int(round(delay_in_simulation_ticks))

    def collect_data(self, timestamp: carla.Timestamp, simulation_state: SimulationState) -> MonitorInputData:
        self.simulation_state = simulation_state
        monitor_input_data = MonitorInputData(timestamp)
        monitor_input_data.speed_limit = simulation_state.speed_limit
        monitor_input_data.weather = self.__process_weather()
        monitor_input_data.connection_strength = simulation_state.connection_strength

        monitor_input_data.ego_speed = self.calculate_vehicle_speed(self.ego_vehicle.get_velocity())
        monitor_input_data.ego_acceleration = self.ego_imu_sensor_data

        if self.new_distance_received:
            monitor_input_data.ego_distance = self.ego_obstacle_sensor_data
            self.new_distance_received = False
        elif self.simulation_state.ego_distance_available:
            monitor_input_data.ego_distance.distance = -1
        else:
            monitor_input_data.ego_distance = None
        monitor_input_data.other_braking_light = self.__update_other_braking_light()

        if self.simulation_state.connection_strength != 0:
            delay_in_simulation_ticks = self.calculate_delay_in_simulation_tick()
            monitor_input_data.other_speed = self.other_speed_data[delay_in_simulation_ticks]
            monitor_input_data.other_acceleration = self.other_acceleration_data[delay_in_simulation_ticks]
            self.update_other_emergency_brake(simulation_state)
            monitor_input_data.other_emergency_brake = self.other_emergency_brake_data[delay_in_simulation_ticks]
        else:
            """ No connection between the two vehicles """
            monitor_input_data.other_speed = None
            monitor_input_data.other_emergency_brake = None
            monitor_input_data.other_acceleration = None

        return monitor_input_data

    def update_ego_acceleration(self, sensor_data: carla.IMUMeasurement) -> None:
        if self.simulation_state.ego_acceleration_available:
            self.ego_imu_sensor_data = sensor_data
        else:
            self.ego_imu_sensor_data = None

    def update_other_acceleration(self, sensor_data: carla.IMUMeasurement) -> None:

        for i, data in reversed(list(enumerate(self.other_acceleration_data))):
            if i == 0:
                if self.simulation_state.other_acceleration_available:
                    self.other_acceleration_data[i] = sensor_data
                else:
                    self.other_acceleration_data[i] = None
            else:
                self.other_acceleration_data[i] = self.other_acceleration_data[i - 1]

    def update_other_speed(self) -> None:
        for i, data in reversed(list(enumerate(self.other_speed_data))):
            if i == 0:
                if self.simulation_state.other_speed_available:
                    self.other_speed_data[i] = self.calculate_vehicle_speed(self.leader_vehicle.get_velocity())
                else:
                    self.other_speed_data[i] = None
            else:
                self.other_speed_data[i] = self.other_speed_data[i - 1]

    def __update_other_braking_light(self) -> Optional[bool]:
        if self.simulation_state.other_braking_light_available:
            light_state = self.leader_vehicle.get_light_state()
            braking_light = light_state.Brake
            return braking_light
        else:
            return None

    def update_ego_obstacle_distance(self, sensor_data: carla.ObstacleDetectionEvent) -> None:
        if self.simulation_state.ego_distance_available:
            self.new_distance_received = True
            self.ego_obstacle_sensor_data = sensor_data
        else:
            self.ego_obstacle_sensor_data = None

    def update_other_emergency_brake(self, simulation_state: SimulationState) -> None:
        for i, data in reversed(list(enumerate(self.other_emergency_brake_data))):
            if i == 0:
                if self.simulation_state.other_emergency_brake_available:
                    self.other_emergency_brake_data[i] = simulation_state.other_perform_emergency_brake
                else:
                    self.other_emergency_brake_data[i] = None
            else:
                self.other_emergency_brake_data[i] = simulation_state.other_perform_emergency_brake[i - 1]

    def __process_weather(self) -> Weather:
        weather = self.carla_world.get_weather()
        if weather.precipitation > 60.0:
            return Weather.RAIN
        elif weather.wetness > 60.0:
            return Weather.RAIN
        else:
            return Weather.SUNSHINE

    def calculate_vehicle_speed(self, velocity: carla.Vector3D) -> float:
        """Takes an carla velocity vector and transforms it into an speed value [m/s]
        :param velocity: velocity vector
        :type velocity: carla.Vector3D
        :return: speed of vector in [m/s]
        :rtype: float
        """

        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
