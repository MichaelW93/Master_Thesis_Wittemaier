import carla

from implementation.vehicle.vehicles import EnvironmentVehicle, LeaderVehicle, ManagedVehicle
from implementation.data_classes import *
from implementation.util import *

class Scenario(object):
    pass


class CutInScenario(Scenario):

    def __init__(self, ego_vehicle: EnvironmentVehicle,
                 front_vehicle: Union[LeaderVehicle, ManagedVehicle],
                 rear_vehicle: ManagedVehicle,
                 carla_world: carla.World):

        self.ego_vehicle: EnvironmentVehicle = ego_vehicle
        self.front_vehicle: Union[LeaderVehicle, ManagedVehicle] = front_vehicle
        self.rear_vehicle: ManagedVehicle = rear_vehicle

        self.carla_world = carla_world

        self.target_speed_reached: bool = False
        self.teleported_to_new_position: bool = False
        self.lane_change_done: bool = False
        self.waited: bool = False
        self.counter = 0

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


