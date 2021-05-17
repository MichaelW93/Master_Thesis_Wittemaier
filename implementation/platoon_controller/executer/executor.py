from typing import TYPE_CHECKING

from implementation.platoon_controller.knowledge.knowledge import Knowledge
from implementation.data_classes import *
from implementation.vehicle.controller import *

if TYPE_CHECKING:
    from implementation.vehicle.vehicles import *


class Executor(object):

    def __init__(self, knowledge: Knowledge, ego_vehicle: "ManagedVehicle"):
        self.knowledge: Knowledge = knowledge
        self.ego_vehicle: "ManagedVehicle" = ego_vehicle

    def run_step(self, plan: Plan):

        if plan == Plan.CACC_CONTROLLER:
            self.ego_vehicle.distance_controller = CACCController(self.ego_vehicle)
            self.knowledge.current_controller = ControllerType.CACC
        elif plan == Plan.SPEED_CONTROLLER:
            self.ego_vehicle.distance_controller = None
            self.knowledge.current_controller = ControllerType.SPEED
            self.knowledge.target_speed = self.knowledge.get_current_environment_knowledge().speed_limit
        elif plan == Plan.ACC_CONTROLLER:
            self.ego_vehicle.distance_controller = ACCController(self.ego_vehicle)
            self.knowledge.current_controller = ControllerType.ACC
