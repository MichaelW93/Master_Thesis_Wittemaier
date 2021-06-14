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

    def run_step(self, plan: Plan, env_knowledge:  EnvironmentKnowledge):

        if plan == Plan.SWITCH_TO_CACC:
            #self.ego_vehicle.data_collector.record_data = True
            self.ego_vehicle.controller = DistanceController(self.ego_vehicle)
            self.ego_vehicle.controller.max_deceleration = -5
            self.ego_vehicle.controller.max_acceleration = 2.75
            self.ego_vehicle.controller.timegap = 0.5
            self.knowledge.current_controller = ControllerType.DISTANCE
            self.knowledge.cont_max_acc = 2.75
            self.knowledge.cont_max_dec = -5
            self.knowledge.timegap = 0.5
            return
        elif plan == Plan.SWITCH_TO_SPEED:
            self.ego_vehicle.controller = SpeedController(self.ego_vehicle)
            self.knowledge.current_controller = ControllerType.SPEED
            self.knowledge.target_speed = self.knowledge.get_current_environment_knowledge().speed_limit
            return
        elif plan == Plan.SWITCH_TO_ACC:
            self.ego_vehicle.controller = DistanceController(self.ego_vehicle)
            self.ego_vehicle.controller.max_deceleration = -3.5
            self.ego_vehicle.controller.max_acceleration = 2
            self.ego_vehicle.controller.timegap = 1.5
            self.knowledge.current_controller = ControllerType.DISTANCE
            self.knowledge.cont_max_acc = 2
            self.knowledge.cont_max_dec = -3.5
            self.knowledge.timegap = 1.5
            return
        elif plan == Plan.SWITCH_TO_BRAKE:
            self.ego_vehicle.controller = BrakeController(self.ego_vehicle)
            self.knowledge.current_controller = ControllerType.BRAKE
            return
        elif plan == Plan.ADAPT_TARGET_SPEED:
            self.knowledge.target_speed = env_knowledge.speed_limit
            return
        elif plan == Plan.ADAPT_TARGET_DISTANCE:
            return
        elif plan == Plan.NO_CHANGE:
            return
        else:
            print("Unknown plan")
