import carla
import math
from typing import Optional, List, Tuple, Union
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

        self.leader_acceleration_data: List[Optional[float]] = initialize_list(None, 10)
        self.leader_speed_data: List[Optional[float]] = initialize_list(None, 10)

        self.managed_vehicles_acceleration_data: List[List[Optional[carla.IMUMeasurement]]] = \
            initialize_list_of_lists(None, 10, NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicles_speed_data: List[List[Optional[float]]] = \
            initialize_list_of_lists(None, 10, NUMBER_OF_MANAGED_VEHICLES)
        self.managed_vehicles_distance: List[List[Optional[carla.ObstacleDetectionEvent]]] = \
            initialize_list_of_lists(None, 10, NUMBER_OF_MANAGED_VEHICLES)

        self.test_list = initialize_list_of_lists(None, 10, NUMBER_OF_MANAGED_VEHICLES)
        self.test_counter_1 = 0
        self.test_counter_2 = 0

        self.max_acceleration = 0
        self.max_deceleration = 0

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
        self.update_leader_acceleration(self.leader_vehicle.get_sensor_data())
        self.update_managed_vehicles_speed()
        self.update_managed_vehicles_sensor_data()

        self.simulation_state = simulation_state

        vehicles_monitor_input_data = []

        for i in range(len(self.managed_vehicles)):
            vehicle = self.managed_vehicles[i]
            monitor_input_data: MonitorInputData = MonitorInputData(timestamp)
            """Simulation data"""
            monitor_input_data.speed_limit = self.simulation_state.speed_limit
            monitor_input_data.weather = self.__process_weather()
            monitor_input_data.connection_strength = self.delay
            monitor_input_data.ego_vehicle_role_name = vehicle.role_name

            """Ego vehicle data"""
            monitor_input_data.ego_vehicle_speed = self.calculate_vehicle_speed(vehicle.get_velocity())
            # Acceleration and distance
            monitor_input_data.ego_vehicle_distance, monitor_input_data.ego_vehicle_acceleration\
                = self.process_sensor_data(vehicle.get_sensor_data(), i)

            """Leader vehicle data"""
            if self.simulation_state.leader_speed_available:
                monitor_input_data.leader_speed = self.leader_speed_data[delay_in_simulation_ticks]
            if self.simulation_state.leader_acceleration_available:
                monitor_input_data.leader_acceleration = self.leader_acceleration_data[delay_in_simulation_ticks]

            """Front vehicle data"""
            if not vehicle.front_vehicle_is_leader:
                for front_vehicle_number in range(len(vehicle.front_vehicles)):
                    monitor_input_data.front_vehicles_speed.append(
                        self.managed_vehicles_speed_data[front_vehicle_number][delay_in_simulation_ticks])
                    if self.managed_vehicles_acceleration_data[front_vehicle_number][delay_in_simulation_ticks] is not None:
                        monitor_input_data.front_vehicles_acceleration.append(
                            self.managed_vehicles_acceleration_data[front_vehicle_number][delay_in_simulation_ticks].accelerometer.x)
                    else:
                        monitor_input_data.front_vehicles_acceleration.append(None)

            vehicles_monitor_input_data.append(monitor_input_data)
            print(monitor_input_data)

        self.leader_vehicle.sensor_data = []
        for vehicle in self.managed_vehicles:
            vehicle.sensor_data = []

        return vehicles_monitor_input_data

    def update_managed_vehicles_sensor_data(self):
        for i in range(len(self.managed_vehicles)):
            imu_data, distance_data = self.extract_sensor_data(self.managed_vehicles[i].get_sensor_data())
            for j, data in reversed(list(enumerate(self.managed_vehicles_acceleration_data[i]))):
                if j == 0:
                    if self.simulation_state.managed_vehicle_acceleration_to_other_available[i]:
                        self.managed_vehicles_acceleration_data[i][j] = imu_data
                    else:
                        self.managed_vehicles_acceleration_data[i][j] = None
                else:
                    self.managed_vehicles_acceleration_data[i][j] = self.managed_vehicles_acceleration_data[i][j - 1]

            for j, data in reversed(list(enumerate(self.managed_vehicles_distance[i]))):
                if j == 0:
                    if self.simulation_state.managed_vehicle_ego_distance_available[i]:
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
                    if self.simulation_state.managed_vehicle_speed_to_other_available[i]:
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

    def update_leader_acceleration(self, sensor_data_array: carla.IMUMeasurement):
        if len(sensor_data_array) >= 1:
            sensor_data = sensor_data_array[0]
        else:
            return
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

        if leader_acceleration < 0:
            if leader_acceleration < self.max_deceleration:
                self.max_deceleration = leader_acceleration
        else:
            if leader_acceleration > self.max_acceleration:
                self.max_acceleration = leader_acceleration

        print("Max acceleration = ", self.max_acceleration, " Max deceleration = ", self.max_deceleration)

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

    def process_sensor_data(self, sensor_data: List [carla.SensorData], vehicle_number: int) -> \
            Tuple[Union[carla.ObstacleDetectionEvent, int], carla.IMUMeasurement]:

        ego_distance = None
        ego_acceleration = None

        for data in sensor_data:
            if isinstance(data, carla.IMUMeasurement):
                ego_acceleration = data
            elif isinstance(data, carla.ObstacleDetectionEvent):
                ego_distance = data

        """No vehicle in front detected"""
        if ego_distance is None:
            if self.simulation_state.managed_vehicle_ego_distance_available[vehicle_number]:
                ego_distance = -1
                ego_distance = carla.ObstacleDetectionEvent()
                ego_distance.distance = 50

        return ego_distance, ego_acceleration

    def __process_weather(self) -> Weather:
        weather = self.carla_world.get_weather()
        if weather.precipitation > 60.0:
            return Weather.RAIN
        elif weather.wetness > 60.0:
            return Weather.RAIN
        else:
            return Weather.SUNSHINE

    @staticmethod
    def calculate_vehicle_speed(velocity: carla.Vector3D) -> float:
        """Takes an carla velocity vector and transforms it into an speed value [m/s]
        :param velocity: velocity vector
        :type velocity: carla.Vector3D
        :return: speed of vector in [m/s]
        :rtype: float
        """

        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
