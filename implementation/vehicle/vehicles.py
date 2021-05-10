import carla

from typing import Optional, List, TYPE_CHECKING
from implementation.vehicle.carla_steering_algorithm import CarlaSteeringAlgorithm
from implementation.vehicle.controller import VehiclePIDController
from implementation.configuration_parameter import *
from implementation.data_classes import EnvironmentKnowledge, SimulationState, MonitorInputData, CommunicationData
from implementation.platoon_controller.platoon_controller import PlatoonController
from implementation.platoon_controller.monitor.monitor import Monitor
from implementation.util import *

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

    def __init__(self, carla_world, role_name, comm_handler):
        super(EnvironmentVehicle, self).__init__(carla_world, comm_handler)
        self.controller = None
        self.steering_controller: Optional[CarlaSteeringAlgorithm] = None
        self.role_name = role_name

    def setup_vehicle(self):
        args_long_dict = {
            'K_P': LEADER_CONTROLLER_KP,
            'K_D': LEADER_CONTROLLER_KD,
            'K_I': LEADER_CONTROLLER_KI,
            'dt': 1 / CARLA_SERVER_FPS
        }
        self.steering_controller = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.ego_vehicle)
        self.controller = VehiclePIDController(self, args_long_dict)

    def run_step(self, target_speed: float) -> None:
        """Main loop for the Vehicle. Handles the calculation of the vehicle control.
        :param target_speed: vehicles target speed in [km/h]"""

        if self.controller is not None:
            self.control = self.controller.run_step(target_speed=target_speed)
        if self.steering_controller is not None:
            self.control.steer = self.steering_controller.goToNextTargetLocation()
        if self.control is not None:
            self.ego_vehicle.apply_control(self.control)

    def switch_lane_right(self):
        waypoint = self.carla_world.get_map().get_waypoint(self.ego_vehicle.get_location(), project_to_road=True,
                                                           lane_type=carla.LaneType.Driving)
        right_lane_waypoint = waypoint.get_right_lane()
        next_waypoint = right_lane_waypoint.next(40)
        self.steering_controller.set_next_waypoint(next_waypoint)

    def destroy(self):
        if self.ego_vehicle is not None and self.ego_vehicle.is_alive:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None


class LeaderVehicle(Vehicle):

    def __init__(self, carla_world, communication_handler):
        super(LeaderVehicle, self).__init__(carla_world, communication_handler)
        self.controller = None
        self.steering_controller: Optional[CarlaSteeringAlgorithm] = None
        self.role_name = "Leader"
        self.has_front_vehicle = False

    def run_step(self, manual_vehicle_control: Optional[carla.VehicleControl], simulation_state: SimulationState, timestamp: carla.Timestamp) -> None:

        data = self.get_sensor_data()

        acceleration = 0
        if isinstance(data[0], carla.IMUMeasurement):
            limits = (-99.9, 99.9)
            acceleration = max(limits[0], min(limits[1], data[0].accelerometer.x))

        comm_data = CommunicationData()
        comm_data.leader_id = -1
        comm_data.front_id = -1
        comm_data.vehicle_id = self.ego_vehicle.id
        comm_data.acceleration = acceleration
        comm_data.speed = velocity_to_speed(self.ego_vehicle.get_velocity())
        comm_data.timestamp = timestamp

        self.send_communication_data(comm_data)

        if manual_vehicle_control is None:
            if self.steering_controller is not None and self.ego_vehicle is not None:
                self.control = self.controller.run_step(target_speed=simulation_state.leader_target_speed)
                self.control.steer = self.steering_controller.goToNextTargetLocation()
        else:
            if manual_vehicle_control.steer == 0:

                self.control.throttle = manual_vehicle_control.throttle
                self.control.brake = manual_vehicle_control.brake
                self.control.steer = self.steering_controller.goToNextTargetLocation()

            else:
                self.control = manual_vehicle_control

        self.ego_vehicle.apply_control(self.control)
        if DEBUG_MODE:
            print("Leader vehicle control: ", self.control)

    def setup_vehicle(self):
        args_long_dict = {
            'K_P': LEADER_CONTROLLER_KP,
            'K_D': LEADER_CONTROLLER_KD,
            'K_I': LEADER_CONTROLLER_KI,
            'dt': 1 / CARLA_SERVER_FPS
        }
        self.steering_controller = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.ego_vehicle)
        self.controller = VehiclePIDController(self, args_long_dict)

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

    def __init__(self, carla_world: carla.World, role_name: str, comm_handler: "CommunicationHandler"):
        super(ManagedVehicle, self).__init__(carla_world, comm_handler)
        self.front_vehicles: List[Optional[Vehicle]] = []
        self.leader_vehicle: Optional[LeaderVehicle] = None
        self.controller = None
        self.steering_controller: Optional[CarlaSteeringAlgorithm] = None
        self.obstacle_distance_sensor: Optional[carla.Sensor] = None
        self.platoon_controller: Optional[PlatoonController] = None
        self.role_name: str = role_name
        self.front_vehicle_is_leader: bool = False
        self.has_front_vehicle = False

    def run_step(self, timestamp: carla.Timestamp, weather: carla.WeatherParameters, speed_limit: float) -> None:

        environment_knowledge = self.platoon_controller.run_step(timestamp, weather, speed_limit)

        self.control = self.controller.run_step(environment_knowledge, environment_knowledge.speed_limit)
        self.control.steer = self.steering_controller.goToNextTargetLocation()
        self.ego_vehicle.apply_control(self.control)
        if DEBUG_MODE:
            print("Managed Vehicle control: ", self.ego_vehicle.id, "\n", self.control)

    def setup_vehicle(self, vehicle_number: int) -> None:
        args_long_dict = {
            'K_P': MANAGED_VEHICLE_CONTROLLER_KP,
            'K_D': MANAGED_VEHICLE_CONTROLLER_KD,
            'K_I': MANAGED_VEHICLE_CONTROLLER_KI,
            'dt': 1 / CARLA_SERVER_FPS
        }

        self.steering_controller = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.ego_vehicle)
        self.controller = VehiclePIDController(self, args_long_dict)

        blueprint_library = self.carla_world.get_blueprint_library()
        blueprint = blueprint_library.find("sensor.other.imu")
        self.imu_sensor = self.carla_world.spawn_actor(blueprint, carla.Transform(), self.ego_vehicle)
        self.imu_sensor.listen(lambda sensor_data: self.store_sensor_data(sensor_data))
        self.sensors.append(self.imu_sensor)

        blueprint = blueprint_library.find("sensor.other.obstacle")
        blueprint.set_attribute("distance", str(50))
        blueprint.set_attribute("only_dynamics", str(True))
        blueprint.set_attribute("hit_radius", str(2))
        if DEBUG_MODE:
            blueprint.set_attribute("debug_linetrace", str(True))

        self.obstacle_distance_sensor = self.carla_world.spawn_actor(blueprint, carla.Transform(),
                                                                           self.ego_vehicle)
        self.obstacle_distance_sensor.listen(
            lambda sensor_data: self.store_sensor_data(sensor_data))
        self.sensors.append(self.obstacle_distance_sensor)

        self.platoon_controller = PlatoonController(self)
        self.role_name = f"Follower_{vehicle_number}"

    def destroy(self):
        super(ManagedVehicle, self).destroy()
        self.front_vehicles = []
        self.leader_vehicle = []
        self.controller = None
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
