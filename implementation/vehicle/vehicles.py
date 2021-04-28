import carla

from typing import Optional, List
from implementation.vehicle.carla_steering_algorithm import CarlaSteeringAlgorithm
from implementation.vehicle.controller import VehiclePIDController
from implementation.configuration_parameter import *
from implementation.data_classes import EnvironmentKnowledge, SimulationState, MonitorInputData
from implementation.platoon_controller.platoon_controller import PlatoonController
from implementation.platoon_controller.monitor.monitor import Monitor


class Vehicle(object):

    def __init__(self, carla_world) -> None:
        self.carla_world: carla.World = carla_world
        self.ego_vehicle: Optional[carla.Vehicle] = None
        self.imu_sensor: Optional[carla.Sensor] = None
        self.sensor_data: List[Optional[carla.SensorData]] = []
        self.control: Optional[carla.VehicleControl] = None
        self.role_name: str = ""

        self.sensors: List[carla.Sensor] = []

    def spawn_vehicle(self, spawn_location: carla.Location, vehicle_type: str) -> carla.Vehicle:
        blueprint_library = self.carla_world.get_blueprint_library()
        blueprint = blueprint_library.find(vehicle_type)
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
        return self.sensor_data

    def get_velocity(self) -> carla.Vector3D:
        """
        Returns the vehicles current velocity
        :return: current velocity
        :rtype: carla.Vector3D
        """
        return self.ego_vehicle.get_velocity()

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


class LeaderVehicle(Vehicle):

    def __init__(self, carla_world):
        super(LeaderVehicle, self).__init__(carla_world)
        self.controller = None
        self.steering_controller: Optional[CarlaSteeringAlgorithm] = None
        self.role_name = "Leader"

    def run_step(self, manual_vehicle_control: Optional[carla.VehicleControl], simulation_state: SimulationState) -> None:
        if manual_vehicle_control is None:
            if self.steering_controller is not None and self.ego_vehicle is not None:
                self.control = self.controller.run_step(None, simulation_state.leader_target_speed)
                self.control.steer = self.steering_controller.goToNextTargetLocation()
        else:
            if manual_vehicle_control.steer == 0:

                self.control.throttle = manual_vehicle_control.throttle
                self.control.brake = manual_vehicle_control.brake
                self.control.steer = self.steering_controller.goToNextTargetLocation()

            else:
                self.control = manual_vehicle_control

        if simulation_state.leader_perform_emergency_brake:
            self.ego_vehicle.set_light_state(carla.VehicleLightState.Brake)
            self.control.throttle = 0.0
            self.control.brake = 1.0

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

    def __init__(self, carla_world, role_name):
        super(ManagedVehicle, self).__init__(carla_world)
        self.front_vehicles: List[Optional[Vehicle]] = []
        self.leader_vehicle: Optional[LeaderVehicle] = None
        self.controller = None
        self.steering_controller: Optional[CarlaSteeringAlgorithm] = None
        self.obstacle_distance_sensor: Optional[carla.Sensor] = None
        self.platoon_controller: Optional[PlatoonController] = None
        self.role_name: str = role_name
        self.front_vehicle_is_leader: bool = False

    def run_step(self, monitor_input: MonitorInputData, simulation_state: SimulationState) -> None:

        environment_knowledge = self.platoon_controller.run_step(monitor_input)

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
        blueprint.set_attribute("hit_radius", str(5))
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
