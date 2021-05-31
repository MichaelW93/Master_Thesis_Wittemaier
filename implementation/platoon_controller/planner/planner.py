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
            self.process_structural_adaptation(environment_knowledge)
            return Plan.SWITCH_TO_CACC
        elif adaptation_technique == AdaptationTechnique.PARAMETER:
            return self.process_parameter_adaptation()
        elif adaptation_technique == AdaptationTechnique.CONTEXT:
            return self.process_context_adaptation(environment_knowledge)
        else:
            return Plan.NO_CHANGE

    def process_parameter_adaptation(self, environment_knowledge: "EnvironmentKnowledge"):

        if self.knowledge.current_controller == ControllerType.DISTANCE:
            if self.knowledge.target_speed > environment_knowledge.speed_limit:
                return Plan.ADAPT_TARGET_SPEED
            else:
                return Plan.NO_CHANGE
        elif self.knowledge.current_controller == ControllerType.DISTANCE:
            return Plan.ADAPT_TARGET_DISTANCE
        elif self.knowledge.current_controller == ControllerType.DISTANCE:
            return Plan.ADAPT_TARGET_DISTANCE
        elif self.knowledge.current_controller == ControllerType.BRAKE:
            return Plan.SWITCH_TO_BRAKE

    def process_context_adaptation(self, env_knowledge: "EnvironmentKnowledge"):
        if self.knowledge.current_controller == ControllerType.DISTANCE:
            return Plan.SWITCH_TO_SPEED
        elif self.knowledge.current_controller == ControllerType.DISTANCE:
            return Plan.SWITCH_TO_SPEED
        elif self.knowledge.current_controller == ControllerType.SPEED:
            if env_knowledge.other_vehicles[self.knowledge.leader_id].speed[1] == FailureType.omission or \
               env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[1] == FailureType.omission or \
               env_knowledge.other_vehicles[self.knowledge.front_vehicle_id].speed[1] == FailureType.omission or \
               env_knowledge.other_vehicles[self.knowledge.front_vehicle_id].acceleration[1] == FailureType.omission:
                return Plan.SWITCH_TO_ACC
            else:
                return Plan.SWITCH_TO_CACC
        elif self.knowledge.current_controller == ControllerType.BRAKE:
            if env_knowledge.other_vehicles[self.knowledge.leader_id].speed[1] == FailureType.omission or \
               env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[1] == FailureType.omission or \
               env_knowledge.other_vehicles[self.knowledge.front_vehicle_id].speed[1] == FailureType.omission or \
               env_knowledge.other_vehicles[self.knowledge.front_vehicle_id].acceleration[1] == FailureType.omission:
                return Plan.SWITCH_TO_ACC
            else:
                return Plan.SWITCH_TO_CACC

    def process_structural_adaptation(self, env_knowledge: "EnvironmentKnowledge"):
        if self.knowledge.leader_id in env_knowledge.other_vehicles:
            if self.knowledge.current_controller == ControllerType.DISTANCE:
                if env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[0] <= -8:
                    # Leader performs emergency brake
                    return Plan.SWITCH_TO_BRAKE
            elif self.knowledge.current_controller == ControllerType.DISTANCE:
                if env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[0] <= -8:
                    # Leader performs emergency brake
                    return Plan.SWITCH_TO_BRAKE
            elif self.knowledge.current_controller == ControllerType.SPEED:
                if env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[0] <= -8:
                    # Leader performs emergency brake
                    return Plan.SWITCH_TO_BRAKE
            elif self.knowledge.current_controller == ControllerType.BRAKE:
                if env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[0] >= -8:
                    # Leader performs emergency brake
                    return Plan.SWITCH_TO_SPEED
        else:
            return

