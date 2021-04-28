from typing import TYPE_CHECKING

from implementation.platoon_controller.knowledge.knowledge import Knowledge
from implementation.data_classes import *
from implementation.vehicle.controller import VehiclePIDController, DistanceController

if TYPE_CHECKING:
    from implementation.vehicle.vehicles import *


class Executor(object):

    def __init__(self, knowledge: Knowledge, ego_vehicle: "ManagedVehicle"):
        self.knowledge: Knowledge = knowledge
        self.ego_vehicle: "ManagedVehicle" = ego_vehicle

    def run_step(self, plan: Plan):

        if plan == Plan.CACC_CONTROLLER:
            self.ego_vehicle.controller = DistanceController(self.ego_vehicle)

