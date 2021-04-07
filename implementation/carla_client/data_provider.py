import carla
import math
from typing import Optional, List, Tuple
from implementation.data_classes import SimulationState, MonitorInputData
from implementation.platoon_controller.knowledge.base_attribute import *
from implementation.vehicle.vehicles import ManagedVehicle, LeaderVehicle
from implementation.util import *


class DataProvider(object):

    def __init__(self, managed_vehicles: List[Optional[ManagedVehicle]], leader_vehicle: LeaderVehicle,
                 carla_world: carla.World, simulation_state: SimulationState):

        self.carla_world = carla_world
        self.delay = 50.0 # in ms
        self.managed_vehicles: List[Optional[ManagedVehicle]] = managed_vehicles
        self.leader_vehicle: LeaderVehicle = leader_vehicle

        self.simulation_state: SimulationState = simulation_state

        self.leader_acceleration_data: List[Optional[float]] = initialize_array(None, 10)
        self.leader_speed_data: List[Optional[float]] = initialize_array(None, 10)

        self.managed_vehicles_acceleration_data: List[List[Optional[carla.IMUMeasurement]]] = \
            initialize_array(initialize_array(None, 10), NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicles_speed_data: List[List[Optional[float]]] = \
            initialize_array(initialize_array(None, 10), NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicles_distance: List[List[Optional[carla.ObstacleDetectionEvent]]] = \
            initialize_array(initialize_array(None, 10), NUMBER_OF_MANAGED_VEHICLES)

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

    def collect_data(self, timestamp: carla.Timestamp, simulation_state: SimulationState) -> List[MonitorInputData]:

        delay_in_simulation_ticks = self.calculate_delay_in_simulation_tick()
        self.update_leader_speed(self.calculate_vehicle_speed(self.leader_vehicle.ego_vehicle.get_velocity()))
        self.update_leader_acceleration(self.leader_vehicle.get_sensor_data()[0])
        self.update_managed_vehicles_speed()
        self.update_managed_vehicles_sensor_data()

        self.simulation_state = simulation_state

        vehicles_monitor_input_data = []

        for i in range(len(self.managed_vehicles)):
            vehicle = self.managed_vehicles[i]
            monitor_input_data: MonitorInputData = MonitorInputData(timestamp)
            """Simulation data"""
            monitor_input_data.speed_limit = simulation_state.speed_limit
            monitor_input_data.weather = self.__process_weather()
            monitor_input_data.connection_strength = simulation_state.connection_strength

            """Ego vehicle data"""
            if self.simulation_state.managed_vehicle_speed_available[i]:
                monitor_input_data.ego_speed = self.calculate_vehicle_speed(vehicle.get_velocity())
            # Acceleration and distance
            monitor_input_data = self.process_sensor_data(vehicle.get_sensor_data(), monitor_input_data, i)

            """Leader vehicle data"""
            if self.simulation_state.leader_speed_available:
                monitor_input_data.leader_speed = self.leader_speed_data[delay_in_simulation_ticks]
            if self.simulation_state.leader_acceleration_available:
                monitor_input_data.leader_acceleration = self.leader_acceleration_data[delay_in_simulation_ticks]

            """Front vehicle data"""
            for front_vehicle_number in range(len(vehicle.front_vehicles)):
                monitor_input_data.front_vehicles_speed.append(
                    self.managed_vehicles_speed_data[front_vehicle_number][delay_in_simulation_ticks])
                monitor_input_data.front_vehicles_acceleration.append(
                    self.managed_vehicles_acceleration_data[front_vehicle_number][delay_in_simulation_ticks])

            vehicles_monitor_input_data.append(monitor_input_data)
        return vehicles_monitor_input_data

    def update_managed_vehicles_sensor_data(self):
        for i in range(len(self.managed_vehicles)):
            imu_data, distance_data = self.extract_sensor_data(self.managed_vehicles[i].get_sensor_data())
            for j, data in reversed(list(enumerate(self.managed_vehicles_acceleration_data[i]))):
                if j == 0:
                    if self.simulation_state.managed_vehicle_acceleration_available[i]:
                        self.managed_vehicles_acceleration_data[i][j] = imu_data
                    else:
                        self.managed_vehicles_acceleration_data[i][j] = None
                else:
                    self.managed_vehicles_acceleration_data[i][j] = self.managed_vehicles_acceleration_data[i][j - 1]

            for j, data in reversed(list(enumerate(self.managed_vehicles_distance[i]))):
                if j == 0:
                    if self.simulation_state.managed_vehicle_distance_available[i]:
                        if imu_data is None:
                            self.managed_vehicles_distance[i][j] = -1
                        else:
                            self.managed_vehicles_distance[i][j] = distance_data
                    else:
                        self.managed_vehicles_distance[i][j] = self.managed_vehicles_distance[i][j - 1]

    def update_managed_vehicles_speed(self):

        for i in range(len(self.managed_vehicles)):
            speed = self.calculate_vehicle_speed(self.managed_vehicles[i].get_velocity())

            for j, data in reversed(list(enumerate(self.managed_vehicles_speed_data[i]))):
                if j == 0:
                    if self.simulation_state.managed_vehicle_speed_available[i]:
                        self.managed_vehicles_speed_data[i][j] = speed
                    else:
                        self.managed_vehicles_speed_data[i][j] = None
                else:
                    self.managed_vehicles_speed_data[i][j] = self.managed_vehicles_speed_data[i][j-1]

    def update_leader_speed(self, speed: float) -> None:
        for i, data in reversed(list(enumerate(self.leader_speed_data))):
            if i == 0:
                if self.simulation_state.leader_speed_available:
                    self.leader_speed_data[i] = speed
                else:
                    self.leader_speed_data[i] = None
            else:
                self.leader_speed_data[i] = self.leader_speed_data[i-1]

    def update_leader_acceleration(self, sensor_data: carla.IMUMeasurement):
        limits = (-99.9, 99.9)
        leader_acceleration = max(limits[0], min(limits[1], sensor_data.accelerometer.x))
        for i, data in reversed(list(enumerate(self.leader_acceleration_data))):
            if i == 0:
                if self.simulation_state.leader_acceleration_available:
                    self.leader_acceleration_data[i] = leader_acceleration
                else:
                    self.leader_acceleration_data[i] = None
            else:
                self.leader_acceleration_data[i] = self.leader_acceleration_data[i - 1]

    @staticmethod
    def extract_sensor_data(sensor_data: List[Optional[carla.SensorData]]) -> \
            Tuple[Optional[carla.IMUMeasurement], Optional[carla.ObstacleDetectionEvent]]:
        imu_data = None
        distance_data = None

        for data in sensor_data:
            if isinstance(data, carla.IMUMeasurement):
                imu_data = data
            elif isinstance(data, carla.ObstacleDetectionEvent):
                distance_data = None

        return imu_data, distance_data

    def process_sensor_data(self, sensor_data: List [carla.SensorData], monitor_input_data: MonitorInputData, vehicle_number: int) -> MonitorInputData:
        for data in sensor_data:
            if isinstance(data, carla.IMUMeasurement):
                if self.simulation_state.managed_vehicle_acceleration_available[vehicle_number]:
                    monitor_input_data.ego_acceleration = data
            elif isinstance(data, carla.ObstaceDetectionEvent):
                if self.simulation_state.managed_vehicle_distance_available[vehicle_number]:
                    monitor_input_data.ego_distance = data

        """No vehicle in front detected"""
        if monitor_input_data.ego_distance is None:
            if self.simulation_state.managed_vehicle_distance_available[vehicle_number]:
                monitor_input_data.ego_distance = -1

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
                    self.other_emergency_brake_data[i] = simulation_state.leader_perform_emergency_brake
                else:
                    self.other_emergency_brake_data[i] = None
            else:
                self.other_emergency_brake_data[i] = simulation_state.leader_perform_emergency_brake[i - 1]

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
