import carla


class CarlaMonitor(object):

    def __init__(self, carla_world, ego_vehicle, leader_vehicle):
        self.ego_vehicle = ego_vehicle
        self.leader_vehicle = leader_vehicle
        self.carla_world = carla_world

    def run_step(self):
        pass

    def reset(self):

        if self.ego_vehicle is not None:
            self.ego_vehicle = None
        if self.leader_vehicle is not None:
            self.leader_vehicle = None
        if self.carla_world is not None:
            self.carla_world = None

    def destroy(self):
        pass