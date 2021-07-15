import carla
import random

from typing import Optional, List, TYPE_CHECKING, Union
from implementation.vehicle.carla_steering_algorithm import CarlaSteeringAlgorithm
from implementation.vehicle.controller import SpeedController, DistanceController, BrakeController, PIDController
from implementation.configuration_parameter import *
from implementation.data_classes import EnvironmentKnowledge, SimulationState, CommunicationData, FailureType, OtherVehicle
from implementation.platoon_controller.platoon_controller import PlatoonController
from implementation.platoon_controller.monitor.monitor import Monitor
from implementation.util import *
from implementation.DecisionTree.DataCollector import DataCollector

if TYPE_CHECKING:
    from implementation.carla_client.communication_handler import CommunicationHandler


class Vehicle(object):

    def __init__(self, carla_world, communication_handler: "CommunicationHandler") -> None:
        self.carla_world: carla.World = carla_world
        self.ego_vehicle: Optional[carla.Vehicle] = None
        self.imu_sensor: Optional[carla.Sensor] = None
        self.sensor_data: List[Optional[carla.SensorData]] = []
        self.control: Optional[carla.VehicleControl] = None
        self.role_name: str = ""
        self.comm_handler: "CommunicationHandler" = communication_handler

        self.sensors: List[carla.Sensor] = []

    def spawn_vehicle(self, spawn_location: carla.Location, vehicle_type: str, color: str = None, role_name: Optional[str] = None) -> carla.Vehicle:
        blueprint_library = self.carla_world.get_blueprint_library()
        blueprint = blueprint_library.find(vehicle_type)
        if color is not None:
            blueprint.set_attribute("color", color)
            if role_name is not None:
                blueprint.set_attribute("role_name", role_name)
        while self.ego_vehicle is None:
            self.ego_vehicle = self.carla_world.try_spawn_actor(blueprint, spawn_location)
            self.carla_world.tick()
        return self.ego_vehicle

    def update_light_state(self):
        pass

    def store_sensor_data(self, data: carla.SensorData) -> None:
        """Stores data from each sensor for every simulation tick, until the data_provider collects it"""
        self.sensor_data.append(data)

    """
        ==============================================================================================
                                    getter and setter help functions
        ==============================================================================================
        """

    def get_sensor_data(self) -> List[Optional[carla.SensorData]]:
        data = self.sensor_data
        self.sensor_data = []
        return data

    def get_velocity(self) -> carla.Vector3D:
        """
        Returns the vehicles current velocity
        :return: current velocity
        :rtype: carla.Vector3D
        """
        return self.ego_vehicle.get_velocity()

    def send_communication_data(self, comm_data: CommunicationData):
        self.comm_handler.set_vehicle_data(comm_data)

    def get_speed(self):
        velocity = self.get_velocity()
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        return speed

    def destroy(self):
        for sensor in self.sensors:
            if sensor is not None and sensor.is_alive:
                print("Destroying sensor: ", sensor)
                sensor.destroy()
        self.sensors = []
        if self.ego_vehicle is not None and self.ego_vehicle.is_alive:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None
        if self.imu_sensor is not None:
            self.imu_sensor = None


class EnvironmentVehicle(Vehicle):

    def __init__(self, carla_world, role_name, comm_handler, leader):
        super(EnvironmentVehicle, self).__init__(carla_world, comm_handler)
        self.controller = None
        self.steering_controller: Optional[CarlaSteeringAlgorithm] = None
        self.role_name = role_name
        self.leading_vehicle: LeaderVehicle = leader

    def setup_vehicle(self):
        args_long_dict = {
            'K_P': LEADER_CONTROLLER_KP,
            'K_D': LEADER_CONTROLLER_KD,
            'K_I': LEADER_CONTROLLER_KI,
            'dt': 1 / CARLA_SERVER_FPS
        }
        self.steering_controller = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.ego_vehicle)
        self.controller = DistanceController(self.ego_vehicle)
        self.controller.timegap = 0.2
        self.controller.max_acceleration = 12
        self.controller.max_deceleration = -12

        blueprint_library = self.carla_world.get_blueprint_library()
        blueprint = blueprint_library.find("sensor.other.imu")
        self.imu_sensor = self.carla_world.spawn_actor(blueprint, carla.Transform(), self.ego_vehicle)
        self.imu_sensor.listen(lambda sensor_data: self.store_sensor_data(sensor_data))
        self.sensors.append(self.imu_sensor)

    def run_step(self, target_speed: float) -> None:
        """Main loop for the Vehicle. Handles the calculation of the vehicle control.
        :param target_speed: vehicles target speed in [km/h]"""

        data = self.get_sensor_data()
        data_dict = self.comm_handler.vehicles_data[1]
        if self.leading_vehicle.ego_vehicle.id in data_dict:
            leader_data: CommunicationData = data_dict[self.leading_vehicle.ego_vehicle.id]
        else:
            leader_data = CommunicationData()

        acceleration = 0
        if len(data)>0:
            if isinstance(data[0], carla.IMUMeasurement):
                limits = (-99.9, 99.9)
                acceleration = max(limits[0], min(limits[1], data[0].accelerometer.x))

        if self.controller is not None:
            env_knowledge = EnvironmentKnowledge()
            env_knowledge.ego_speed_tuple = (velocity_to_speed(self.ego_vehicle.get_velocity()), FailureType.no_failure)
            other_vehicle = OtherVehicle(self.leading_vehicle.ego_vehicle.id)
            other_vehicle.speed_tuple = (leader_data.speed, FailureType.no_failure)
            other_vehicle.acceleration_tuple = (leader_data.acceleration, FailureType.no_failure)
            other_vehicle.is_front_vehicle = True
            env_knowledge.other_vehicles[self.leading_vehicle.ego_vehicle.id] = other_vehicle
            env_knowledge.ego_distance_tuple = self.get_distance()
            env_knowledge.speed_limit = target_speed
            print(env_knowledge)
            self.control = self.controller.run_step(env_knowledge)
            print(self.control)
        if self.steering_controller is not None:
            self.control.steer = self.steering_controller.goToNextTargetLocation()
        if self.control is not None:
            self.ego_vehicle.apply_control(self.control)

        comm_data = CommunicationData()
        comm_data.leader_id = -1
        comm_data.front_id = -1
        comm_data.vehicle_id = self.ego_vehicle.id
        comm_data.acceleration = acceleration
        comm_data.speed = velocity_to_speed(self.ego_vehicle.get_velocity())
        comm_data.throttle = self.control.throttle
        comm_data.brake = self.control.brake

        self.send_communication_data(comm_data)

    def switch_lane_right(self):
        waypoint = self.carla_world.get_map().get_waypoint(self.ego_vehicle.get_location(), project_to_road=True,
                                                           lane_type=carla.LaneType.Driving)
        right_lane_waypoint = waypoint.get_right_lane()
        next_waypoint = right_lane_waypoint.next(40)
        self.steering_controller.set_next_waypoint(next_waypoint)

    def get_distance(self):
        ego_pos = self.ego_vehicle.get_location()
        leader_pos = self.leading_vehicle.ego_vehicle.get_location()

        dist = math.sqrt((ego_pos.x - leader_pos.x) ** 2 +
                         (ego_pos.y - leader_pos.y) ** 2 +
                         (ego_pos.z - leader_pos.z) ** 2)

        dist_to_front = dist - (self.ego_vehicle.bounding_box.extent.x * 2) + 0.5

        dist_tuple = (dist_to_front, FailureType.no_failure)
        return dist_tuple

    def destroy(self):
        if self.ego_vehicle is not None and self.ego_vehicle.is_alive:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None
        self.imu_sensor.destroy()


class LeaderVehicle(Vehicle):

    def __init__(self, carla_world, communication_handler):
        super(LeaderVehicle, self).__init__(carla_world, communication_handler)
        self.controller = None
        self.steering_controller: Optional[CarlaSteeringAlgorithm] = None
        self.role_name = "Leader"
        self.has_front_vehicle = False
        self.perform_emergency_brake = False
        self.perform_soft_brake = False
        self.perform_medium_brake = False
        self.target_speed = -1

    def run_step(self, manual_vehicle_control: Optional[carla.VehicleControl], simulation_state: SimulationState, timestamp: carla.Timestamp) -> None:

        data = self.get_sensor_data()

        if self.target_speed == -1:
            target_speed = simulation_state.leader_target_speed
        else:
            target_speed = self.target_speed

        acceleration = 0
        if isinstance(data[0], carla.IMUMeasurement):
            limits = (-99.9, 99.9)
            acceleration = max(limits[0], min(limits[1], data[0].accelerometer.x))

        if manual_vehicle_control is None:
            if self.steering_controller is not None and self.ego_vehicle is not None:
                self.control = self.controller.run_step(target_speed)
                self.control.steer = self.steering_controller.goToNextTargetLocation()
        else:
            if manual_vehicle_control.steer == 0:

                self.control.throttle = manual_vehicle_control.throttle
                self.control.brake = manual_vehicle_control.brake
                self.control.steer = self.steering_controller.goToNextTargetLocation()

            else:
                self.control = manual_vehicle_control

        self.perform_emergency_brake = simulation_state.emergency_brake
        self.perform_medium_brake = simulation_state.medium_brake
        self.perform_soft_brake = simulation_state.soft_brake

        if self.perform_emergency_brake:
            self.control.brake = 1.0
            self.control.throttle = 0
        if self.perform_medium_brake:
            self.control.brake = random.uniform(0.4, 0.6)
            self.control.throttle = 0
        if self.perform_soft_brake:
            self.control.brake = random.uniform(0.1, 0.2)
            self.control.throttle = 0

        comm_data = CommunicationData()
        comm_data.leader_id = -1
        comm_data.front_id = -1
        comm_data.vehicle_id = self.ego_vehicle.id
        comm_data.acceleration = acceleration
        comm_data.speed = velocity_to_speed(self.ego_vehicle.get_velocity())
        comm_data.timestamp = timestamp
        comm_data.throttle = self.control.throttle
        comm_data.brake = self.control.brake

        self.send_communication_data(comm_data)

        self.ego_vehicle.apply_control(self.control)
        if DEBUG_MODE:
            print("Leader vehicle control: ", self.control)

    def setup_vehicle(self):

        self.steering_controller = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.ego_vehicle)
        self.controller = PIDController(self.ego_vehicle)

        blueprint_library = self.carla_world.get_blueprint_library()
        blueprint = blueprint_library.find("sensor.other.imu")
        self.imu_sensor = self.carla_world.spawn_actor(blueprint, carla.Transform(), self.ego_vehicle)
        self.imu_sensor.listen(lambda sensor_data: self.store_sensor_data(sensor_data))
        self.sensors.append(self.imu_sensor)
        self.role_name = "leader"

    def destroy(self):
        super(LeaderVehicle, self).destroy()
        self.controller = None
        self.steering_controller = None


class ManagedVehicle(Vehicle):

    def __init__(self, carla_world: carla.World, role_name: str, comm_handler: "CommunicationHandler", carla_map: carla.Map):
        super(ManagedVehicle, self).__init__(carla_world, comm_handler)
        self.front_vehicles: List[Optional[Vehicle]] = []
        self.leader_vehicle: Optional[LeaderVehicle] = None
        self.controller: Union["DistanceController", "SpeedController",  "BrakeController"] = None
        self.steering_controller: Optional[CarlaSteeringAlgorithm] = None
        self.obstacle_distance_sensor: Optional[carla.Sensor] = None
        self.platoon_controller: Optional[PlatoonController] = None
        self.role_name: str = role_name
        self.front_vehicle_is_leader: bool = False
        self.has_front_vehicle = False
        self.target_speed = 0
        self.data_collector: DataCollector = None
        self.carla_map = carla_map

        self.other_vehicles: List["Vehicle"] = []

    def run_step(self, timestamp: carla.Timestamp, weather: carla.WeatherParameters, speed_limit: float, sim_state: "SimulationState") -> None:

        environment_knowledge = self.platoon_controller.run_step(timestamp, weather, speed_limit)
        self.data_collector.run_step(environment_knowledge, sim_state)

        self.control = self.controller.run_step(environment_knowledge)

        self.control.steer = self.steering_controller.goToNextTargetLocation()
        self.ego_vehicle.apply_control(self.control)

        comm_data = CommunicationData()
        comm_data.leader_id = environment_knowledge.leader_id
        comm_data.front_id = environment_knowledge.front_vehicle_id
        comm_data.vehicle_id = self.ego_vehicle.id
        comm_data.acceleration = environment_knowledge.ego_acceleration_tuple[0]
        comm_data.speed = environment_knowledge.ego_speed_tuple[0]
        comm_data.timestamp = timestamp
        comm_data.throttle = self.control.throttle
        comm_data.brake = self.control.brake
        comm_data.steering = self.control.steer

        print(f"{self.role_name}, acceleration: {comm_data.acceleration}")
        print(f"{self.role_name}, speed: {comm_data.speed}")

        self.send_communication_data(comm_data)

        if DEBUG_MODE:
            print("Managed Vehicle control: ", self.ego_vehicle.id, "\n", self.control)

    def setup_vehicle(self, vehicle_number: int) -> None:

        self.controller = SpeedController(self.ego_vehicle)
        self.steering_controller = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.ego_vehicle)


        blueprint_library = self.carla_world.get_blueprint_library()
        blueprint = blueprint_library.find("sensor.other.imu")
        self.imu_sensor = self.carla_world.spawn_actor(blueprint, carla.Transform(), self.ego_vehicle)
        self.imu_sensor.listen(lambda sensor_data: self.store_sensor_data(sensor_data))
        self.sensors.append(self.imu_sensor)

        blueprint = blueprint_library.find("sensor.other.obstacle")
        blueprint.set_attribute("distance", str(60))
        blueprint.set_attribute("only_dynamics", str(True))
        blueprint.set_attribute("hit_radius", str(10))
        if DEBUG_MODE:
            blueprint.set_attribute("debug_linetrace", str(True))

        self.obstacle_distance_sensor = self.carla_world.spawn_actor(blueprint, carla.Transform(),
                                                                           self.ego_vehicle)
        self.obstacle_distance_sensor.listen(
            lambda sensor_data: self.store_sensor_data(sensor_data))
        self.sensors.append(self.obstacle_distance_sensor)

        self.platoon_controller = PlatoonController(self,  self.carla_map)
        self.role_name = f"Follower_{vehicle_number}"
        self.target_speed: float = self.platoon_controller.knowledge.target_speed
        self.data_collector = DataCollector(self.platoon_controller.knowledge, self)

    def destroy(self):
        super(ManagedVehicle, self).destroy()
        self.data_collector.terminate()
        self.front_vehicles = []
        self.leader_vehicle = []
        self.steering_controller = None
        self.obstacle_distance_sensor = None
        self.platoon_controller.destroy()
        self.platoon_controller = None

    """
    ==============================================================================================
                                getter and setter help functions
    ==============================================================================================
    """
    def get_monitor(self) -> Monitor:
        return self.platoon_controller.monitor
