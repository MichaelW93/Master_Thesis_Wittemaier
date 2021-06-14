import carla
from enum import Enum
from implementation.vehicle.vehicles import EnvironmentVehicle, LeaderVehicle, ManagedVehicle
from implementation.data_classes import *
from implementation.util import *

import random


class Scenarios(Enum):

    CUT_IN_SCENARIO = 1
    LEADER_SPEEDING = 2
    EMERGENCY_BRAKE = 3
    COMMUNICATION_FAILURE = 4
    SOFT_BRAKE = 5
    MEDIUM_BRAKE = 6


class Scenario(object):

    def __init__(self):
        self.done = False

    def run_step(self, sim_state: SimulationState):
        raise NotImplementedError

    def create_random_sim_state(self) -> SimulationState:
        raise NotImplementedError

    def destroy(self):
        return


class CutInScenario(Scenario):

    def __init__(self, front_vehicle: Union[LeaderVehicle, ManagedVehicle], rear_vehicle: ManagedVehicle, carla_world: carla.World,
                 communication_handler):

        super(CutInScenario, self).__init__()
        self.carla_world = carla_world
        self.communication_handler = communication_handler
        self.ego_vehicle: EnvironmentVehicle = self.spawn_environment_vehicle()
        self.front_vehicle: LeaderVehicle = front_vehicle
        self.rear_vehicle: ManagedVehicle = rear_vehicle

        self.target_speed_reached: bool = False
        self.teleported_to_new_position: bool = False
        self.lane_change_done: bool = False
        self.waited: bool = False
        self.counter = 0

    def spawn_environment_vehicle(self) -> EnvironmentVehicle:
        spawn_point = carla.Transform()
        spawn_point.location.x = 19
        spawn_point.location.y = -205
        spawn_point.location.z = 2

        spawn_point.rotation.yaw = 180

        environment_vehicle = EnvironmentVehicle(self.carla_world, "Cut_In_Vehicle_1", self.communication_handler)
        environment_vehicle.spawn_vehicle(spawn_point, "vehicle.audi.tt", "0,255,0", "Cut_In_Vehicle_1")
        environment_vehicle.setup_vehicle()
        self.carla_world.tick()
        self.communication_handler.vehicles[environment_vehicle.ego_vehicle.id] = environment_vehicle
        return environment_vehicle

    def run_step(self, simulation_state: SimulationState):

        speed = velocity_to_speed(self.ego_vehicle.get_velocity()) * 3.6

        if self.lane_change_done:
            self.ego_vehicle.run_step(velocity_to_speed(self.front_vehicle.get_velocity()) * 3.6)
            print("Current Transform: ", self.ego_vehicle.ego_vehicle.get_transform())
            return

        if self.waited and not self.lane_change_done:
            self.ego_vehicle.switch_lane_right()
            self.lane_change_done = True
            self.ego_vehicle.run_step(velocity_to_speed(self.front_vehicle.get_velocity()) * 3.6)
            print("Current Transform: ", self.ego_vehicle.ego_vehicle.get_transform())
            return

        if self.teleported_to_new_position and not self.waited:
            self.ego_vehicle.run_step(velocity_to_speed(self.front_vehicle.get_velocity()) * 3.6)
            self.counter += 1
            print("Current Transform: ", self.ego_vehicle.ego_vehicle.get_transform())
            print("Waiting...")
            if self.counter == 100:
                self.waited = True
            return

        if self.target_speed_reached and not self.teleported_to_new_position:
            teleport_transform = self.calculate_teleport_position()
            print(teleport_transform)
            transform = carla.Transform()
            transform.location = teleport_transform.location
            transform.rotation = self.rear_vehicle.ego_vehicle.get_transform().rotation

            print("Final teleport transform: ", transform)
            self.ego_vehicle.ego_vehicle.set_transform(transform)
            self.teleported_to_new_position = True
            self.ego_vehicle.run_step(simulation_state.environment_vehicles_target_speed)
            return

        if speed >= (simulation_state.environment_vehicles_target_speed - 1) and not self.target_speed_reached:
            print("Target speed reached")
            self.ego_vehicle.run_step(simulation_state.environment_vehicles_target_speed)
            self.target_speed_reached = True
        else:
            self.ego_vehicle.run_step(simulation_state.environment_vehicles_target_speed)
            return

    def calculate_teleport_position(self) -> carla.Transform:
        transform_front: carla.Transform = self.front_vehicle.ego_vehicle.get_transform()
        transform_rear: carla.Transform = self.rear_vehicle.ego_vehicle.get_transform()
        print("Transform_front: ", transform_front)
        print("Transform_rear: ", transform_rear)

        distance = math.sqrt((transform_front.location.x - transform_rear.location.x) ** 2 +
                             (transform_front.location.y - transform_rear.location.y) ** 2)
        waypoint_rear = self.carla_world.get_map().get_waypoint(transform_rear.location)
        waypoints_left = waypoint_rear.get_left_lane().next(abs(distance)/2)
        print("Waypoint_Left: ", waypoints_left[0])
        teleport_transform = waypoints_left[0].transform
        teleport_transform.location.z = max(transform_front.location.z, transform_rear.location.z)

        return teleport_transform

    def create_random_sim_state(self) -> SimulationState:
        sim_state = SimulationState()
        sim_state.speed_limit = random.randint(60, 100)
        sim_state.leader_target_speed = sim_state.speed_limit
        sim_state.environment_vehicles_target_speed = sim_state.speed_limit
        sim_state.vehicles_speed_available[self.leader.ego_vehicle.id] = True
        sim_state.vehicles_acceleration_available[self.leader.ego_vehicle.id] = True
        for vehicle in self.managed_vehicles:
            sim_state.vehicles_speed_available[vehicle.ego_vehicle.id] = True
            sim_state.vehicles_acceleration_available[vehicle.ego_vehicle.id] = True

        sim_state.vehicles_speed_available[self.ego_vehicle.ego_vehicle.id] = True
        sim_state.vehicles_acceleration_available[self.ego_vehicle.ego_vehicle.id] = True

        return sim_state

    def destroy(self):
        self.ego_vehicle.destroy()


class EmergencyBrakeScenario(Scenario):

    def __init__(self, leader_vehicle: LeaderVehicle, managed_vehicles: List[ManagedVehicle]):

        super(EmergencyBrakeScenario, self).__init__()
        self.leader = leader_vehicle
        self.managed_vehicles = managed_vehicles
        self.target_speed: float = 60
        self.target_speed_reached: bool = False
        self.leader_stopped: bool = False
        self.waited_for_ticks: bool = False
        self.waited_for_ticks_2: bool = False
        self.done = False

        self.counter = 0
        self.counter_2 = 0

    def run_step(self, sim_state: SimulationState):

        if self.waited_for_ticks_2:
            self.done = True
            return

        if self.waited_for_ticks and not self.waited_for_ticks_2:
            if self.counter_2 >= 100:
                self.waited_for_ticks_2 = True
            self.counter_2 += 1

        if self.waited_for_ticks:
            self.leader.perform_emergency_brake = False
            return

        if self.leader_stopped and not self.waited_for_ticks:
            if self.counter >= 40:
                self.waited_for_ticks = True
            self.counter += 1

        if self.leader.get_speed() == 0 and self.target_speed_reached:
            self.leader_stopped = True
            return

        if self.target_speed_reached:
            for vehicle in self.managed_vehicles:
                vehicle.data_collector.record_data = True
                print("Record data set to True")
            self.leader.perform_emergency_brake = True
            return

        if not self.target_speed_reached:
            if (self.leader.get_speed() * 3.6) > (self.target_speed - 5):
                self.target_speed_reached = True
            else:
                self.target_speed_reached = False

            for vehicle in self.managed_vehicles:
                if (vehicle.get_speed() * 3.6) > (self.target_speed - 5):
                    self.target_speed_reached &= True
                else:
                    self.target_speed_reached &= False
            return

    def create_random_sim_state(self) -> SimulationState:
        sim_state = SimulationState()
        sim_state.speed_limit = random.randint(60, 100)
        sim_state.leader_target_speed = sim_state.speed_limit
        self.target_speed = sim_state.speed_limit

        sim_state.vehicles_speed_available[self.leader.ego_vehicle.id] = True
        sim_state.vehicles_acceleration_available[self.leader.ego_vehicle.id] = True
        for vehicle in self.managed_vehicles:
            sim_state.vehicles_speed_available[vehicle.ego_vehicle.id] = True
            sim_state.vehicles_acceleration_available[vehicle.ego_vehicle.id] = True

        return sim_state


class MediumBrakeScenario(Scenario):

    def __init__(self, leader_vehicle: LeaderVehicle, managed_vehicles: List[ManagedVehicle]):
        super(MediumBrakeScenario, self).__init__()
        self.leader = leader_vehicle
        self.managed_vehicles = managed_vehicles
        self.target_speed: float = 60
        self.target_speed_reached: bool = False
        self.leader_slowed: bool = False
        self.waited_for_ticks: bool = False
        self.waited_for_ticks_2: bool = False
        self.target_speed_after_brake = 30

        self.counter = 0
        self.counter_2 = 0

    def run_step(self, sim_state: SimulationState):
        if self.waited_for_ticks_2:
            self.done = True
            return

        if self.waited_for_ticks and not self.waited_for_ticks_2:
            if self.counter_2 >= 100:
                self.waited_for_ticks_2 = True
            self.counter_2 += 1

        if self.waited_for_ticks:
            self.leader.perform_medium_brake = False
            return

        if self.leader_slowed and not self.waited_for_ticks:
            if self.counter >= 40:
                self.waited_for_ticks = True
            self.counter += 1

        if self.leader.get_speed() <= self.target_speed_after_brake and self.target_speed_reached:
            self.leader_slowed = True
            return

        if self.target_speed_reached:
            for vehicle in self.managed_vehicles:
                vehicle.data_collector.record_data = True
                print("Record data set to True")
            self.leader.perform_medium_brake = True
            return

        if not self.target_speed_reached:
            if (self.leader.get_speed() * 3.6) > (self.target_speed - 5):
                self.target_speed_reached = True
            else:
                self.target_speed_reached = False

            for vehicle in self.managed_vehicles:
                if (vehicle.get_speed() * 3.6) > (self.target_speed - 5):
                    self.target_speed_reached &= True
                else:
                    self.target_speed_reached &= False
            return

    def create_random_sim_state(self) -> SimulationState:
        sim_state = SimulationState()
        sim_state.speed_limit = random.randint(60, 100)
        sim_state.leader_target_speed = sim_state.speed_limit
        self.target_speed = sim_state.speed_limit
        self.target_speed_after_brake = self.target_speed - 30

        sim_state.vehicles_speed_available[self.leader.ego_vehicle.id] = True
        sim_state.vehicles_acceleration_available[self.leader.ego_vehicle.id] = True
        for vehicle in self.managed_vehicles:
            sim_state.vehicles_speed_available[vehicle.ego_vehicle.id] = True
            sim_state.vehicles_acceleration_available[vehicle.ego_vehicle.id] = True

        return sim_state


class SoftBrakeScenario(Scenario):

    def __init__(self, leader_vehicle, managed_vehicles):
        super(SoftBrakeScenario, self).__init__()
        self.leader = leader_vehicle
        self.managed_vehicles = managed_vehicles
        self.target_speed: float = 60
        self.target_speed_reached: bool = False
        self.leader_slowed: bool = False
        self.waited_for_ticks: bool = False
        self.waited_for_ticks_2: bool = False
        self.target_speed_after_brake = 30

        self.counter = 0
        self.counter_2 = 0

    def run_step(self, sim_state: SimulationState):
        if self.waited_for_ticks_2:
            self.done = True
            return

        if self.waited_for_ticks and not self.waited_for_ticks_2:
            if self.counter_2 >= 100:
                self.waited_for_ticks_2 = True
            self.counter_2 += 1

        if self.waited_for_ticks:
            self.leader.perform_soft_brake = False
            return

        if self.leader_slowed and not self.waited_for_ticks:
            if self.counter >= 40:
                self.waited_for_ticks = True
            self.counter += 1

        if self.leader.get_speed() <= self.target_speed_after_brake and self.target_speed_reached:
            self.leader_slowed = True
            return

        if self.target_speed_reached:
            for vehicle in self.managed_vehicles:
                vehicle.data_collector.record_data = True
                print("Record data set to True")
            self.leader.perform_soft_brake = True
            return

        if not self.target_speed_reached:
            if (self.leader.get_speed() * 3.6) > (self.target_speed - 5):
                self.target_speed_reached = True
            else:
                self.target_speed_reached = False

            for vehicle in self.managed_vehicles:
                if (vehicle.get_speed() * 3.6) > (self.target_speed - 5):
                    self.target_speed_reached &= True
                else:
                    self.target_speed_reached &= False
            return

    def create_random_sim_state(self) -> SimulationState:
        sim_state = SimulationState()
        sim_state.speed_limit = random.randint(60, 100)
        sim_state.leader_target_speed = sim_state.speed_limit
        self.target_speed = sim_state.speed_limit
        self.target_speed_after_brake = self.target_speed - 20

        sim_state.vehicles_speed_available[self.leader.ego_vehicle.id] = True
        sim_state.vehicles_acceleration_available[self.leader.ego_vehicle.id] = True
        for vehicle in self.managed_vehicles:
            sim_state.vehicles_speed_available[vehicle.ego_vehicle.id] = True
            sim_state.vehicles_acceleration_available[vehicle.ego_vehicle.id] = True

        return sim_state


class SpeedingLeaderScenario(Scenario):

    def __init__(self, leader_vehicle: LeaderVehicle, managed_vehicles: List[ManagedVehicle]):
        super(SpeedingLeaderScenario, self).__init__()
        self.leader = leader_vehicle
        self.managed_vehicles = managed_vehicles
        self.target_speed = 60
        self.leader_target_speed: float = 70
        self.target_speed_reached: bool = False
        self.leader_target_speed_reached: bool = False
        self.waited: bool = True
        self.counter = 0

    def run_step(self, sim_state: SimulationState):

        if self.leader_target_speed_reached and not self.waited:
            if self.counter >= 100:
                self.done = True
            self.counter += 1
        if (self.leader.get_speed() * 3.6) >= (self.leader_target_speed_reached - 5) \
           and not self.leader_target_speed_reached:
            self.target_speed_reached = True

        if self.target_speed_reached:
            for vehicle in self.managed_vehicles:
                vehicle.data_collector.record_data = True
                print("Record data set to True")
            self.leader.target_speed = self.leader_target_speed

        if not self.target_speed_reached:
            if (self.leader.get_speed() * 3.6) > (self.target_speed - 5):
                self.target_speed_reached = True
            else:
                self.target_speed_reached = False

            for vehicle in self.managed_vehicles:
                if (vehicle.get_speed() * 3.6) > (self.target_speed - 5):
                    self.target_speed_reached &= True
                else:
                    self.target_speed_reached &= False
            return

    def create_random_sim_state(self) -> SimulationState:
        sim_state: SimulationState = SimulationState()
        sim_state.speed_limit = random.randint(60, 80)
        sim_state.leader_target_speed = sim_state.speed_limit
        self.leader_target_speed = sim_state.speed_limit + 10
        sim_state.vehicles_speed_available[self.leader.ego_vehicle.id] = True
        sim_state.vehicles_acceleration_available[self.leader.ego_vehicle.id] = True
        for vehicle in self.managed_vehicles:
            sim_state.vehicles_speed_available[vehicle.ego_vehicle.id] = True
            sim_state.vehicles_acceleration_available[vehicle.ego_vehicle.id] = True

        return sim_state


class CommunicationFailureScenario(Scenario):

    def __init__(self, leader_vehicle: LeaderVehicle, managed_vehicles):

        super(CommunicationFailureScenario, self).__init__()
        self.leader: LeaderVehicle = leader_vehicle
        self.managed_vehicles = managed_vehicles
        self.target_speed = 60
        self.target_speed_reached: bool = False
        self.waited: bool = False
        self.counter = 0

    def run_step(self, sim_state: SimulationState):


        if self.target_speed_reached and not self.waited:
            for vehicle in self.managed_vehicles:
                vehicle.data_collector.record_data = True
                print("Record data set to True")
            if self.counter >= 100:
                self.done = True
            self.counter += 1

        if not self.target_speed_reached:
            if (self.leader.get_speed() * 3.6) > (self.target_speed - 5):
                self.target_speed_reached = True
            else:
                self.target_speed_reached = False

            for vehicle in self.managed_vehicles:
                if (vehicle.get_speed() * 3.6) > (self.target_speed - 5):
                    self.target_speed_reached &= True
                else:
                    self.target_speed_reached &= False
            return

    def create_random_sim_state(self) -> SimulationState:
        sim_state: SimulationState = SimulationState()
        sim_state.speed_limit = random.randint(60, 80)
        sim_state.leader_target_speed = sim_state.speed_limit

        sim_state.vehicles_speed_available[self.leader.ego_vehicle.id] = bool(random.getrandbits(1))
        sim_state.vehicles_acceleration_available[self.leader.ego_vehicle.id] = bool(random.getrandbits(1))

        for vehicle in self.managed_vehicles:
            sim_state.vehicles_speed_available[vehicle.ego_vehicle.id] = bool(random.getrandbits(1))
            sim_state.vehicles_acceleration_available[vehicle.ego_vehicle.id] = bool(random.getrandbits(1))

        return sim_state


