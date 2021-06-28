from typing import TYPE_CHECKING
from implementation.platoon_controller.knowledge.knowledge import Knowledge
from implementation.vehicle.controller import *
from implementation.data_classes import AdaptationTechnique, Plan
if TYPE_CHECKING:
    from implementation.data_classes import *


class Planner(object):

    def __init__(self, knowledge: Knowledge):
        self.knowledge = knowledge

    def run_step(self, adaptation_technique: AdaptationTechnique, environment_knowledge: "EnvironmentKnowledge"):

        if adaptation_technique == AdaptationTechnique.STRUCTURAL:
            plan = self.process_structural_adaptation(environment_knowledge)
            return plan
        elif adaptation_technique == AdaptationTechnique.PARAMETER:
            return self.process_parameter_adaptation(environment_knowledge)
        elif adaptation_technique == AdaptationTechnique.CONTEXT:
            return self.process_context_adaptation(environment_knowledge)
        else:
            return Plan.NO_CHANGE

    def process_parameter_adaptation(self, environment_knowledge: "EnvironmentKnowledge"):

        if self.knowledge.current_controller == ControllerType.DISTANCE:
            if self.knowledge.cont_max_acc == 2.75:  # CACC Mode
                return Plan.SWITCH_TO_ACC
            else:
                return Plan.SWITCH_TO_CACC
        elif self.knowledge.current_controller == ControllerType.BRAKE:
            return Plan.EMERGENCY_BRAKE
        else:
            return Plan.NO_CHANGE

    def process_context_adaptation(self, env_knowledge: "EnvironmentKnowledge"):
        if self.knowledge.current_controller == ControllerType.DISTANCE:
            return Plan.EMERGENCY_BRAKE
        else:
            return Plan.NO_CHANGE

    def process_structural_adaptation(self, env_knowledge: "EnvironmentKnowledge"):

        controller = self.knowledge.current_controller
        if self.knowledge.front_vehicle_id in env_knowledge.other_vehicles:
            front_vehicle = env_knowledge.other_vehicles[self.knowledge.front_vehicle_id]
        else:
            return Plan.SWITCH_TO_SPEED

        if front_vehicle.speed_tuple[1] == FailureType.no_failure:
            speed_front = front_vehicle.speed_tuple[0]
        else:
            speed_front = front_vehicle.measured_speed_tuple[0]

        if controller == ControllerType.DISTANCE:
            # no front vehicle
            if env_knowledge.ego_distance_tuple[1] == FailureType.no_front_vehicle:
                return Plan.SWITCH_TO_SPEED
            # obey speed limit
            if env_knowledge.speed_over_limit > 3:
                return Plan.SWITCH_TO_SPEED

            if env_knowledge.distance_error < 0:
                return Plan.SWITCH_TO_BRAKE

            if env_knowledge.distance_error > 2 and env_knowledge.front_over_limit < 0:
                return Plan.SWITCH_TO_SPEED

        elif controller == ControllerType.SPEED:
            if env_knowledge.distance_error < 2 and env_knowledge.speed_over_limit < 3:
                if front_vehicle.speed_tuple[1] == FailureType.no_failure and \
                        front_vehicle.acceleration_tuple[1] == FailureType.no_failure:
                    return Plan.SWITCH_TO_CACC
                else:
                    return Plan.SWITCH_TO_ACC
            else:
                return Plan.NO_CHANGE
        elif controller == ControllerType.BRAKE:
            if front_vehicle.speed_tuple[1] == FailureType.no_failure and \
                    front_vehicle.acceleration_tuple[1] == FailureType.no_failure:
                return Plan.SWITCH_TO_CACC
            else:
                return Plan.SWITCH_TO_ACC

        return Plan.SWITCH_TO_CACC
