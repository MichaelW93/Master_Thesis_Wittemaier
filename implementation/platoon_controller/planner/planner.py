from typing import TYPE_CHECKING
from implementation.platoon_controller.knowledge.knowledge import Knowledge
from implementation.vehicle.controller import *
if TYPE_CHECKING:
    from implementation.data_classes import *


class Planner(object):

    def __init__(self, knowledge: Knowledge):
        self.knowledge = knowledge

    def run_step(self, adaptation_technique: AdaptationTechnique, environment_knowledge: "EnvironmentKnowledge"):

        if adaptation_technique == AdaptationTechnique.STRUCTURAL:
            return Plan.CACC_CONTROLLER
        elif adaptation_technique == AdaptationTechnique.PARAMETER:
            pass
        else:
            pass

    def process_parameter_adaptation(self, environment_knowledge: "EnvironmentKnowledge"):

        if self.knowledge.current_controller == ControllerType.SPEED:
            self.knowledge.target_speed = environment_knowledge.speed_limit

    def process_context_adaptation(self, env_knowledge: "EnvironmentKnowledge"):
        pass

    def process_structural_adaptation(self, env_knowledge: "EnvironmentKnowledge"):

        if self.knowledge.current_controller == ControllerType.CACC:
            if env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[0] <= -8:
                # Leader performs emergency brake
                return Plan.EMERGENCY_BRAKE
            if env_knowledge.other_vehicles[self.knowledge.leader_id].speed[1] == FailureType.omission or \
               env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[1] == FailureType.omission or \
               env_knowledge.other_vehicles[self.knowledge.front_vehicle_id].speed[1] == FailureType.omission or \
               env_knowledge.other_vehicles[self.knowledge.front_vehicle_id].acceleration[1] == FailureType.omission:
                return Plan.ACC_CONTROLLER
            if env_knowledge.ego_distance[1] == FailureType.no_front_vehicle:
                self.knowledge.target_speed = env_knowledge.speed_limit
                return Plan.SPEED_CONTROLLER

        elif self.knowledge.current_controller == ControllerType.ACC:
            if env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[0] <= -8:
                # Leader performs emergency brake
                return Plan.EMERGENCY_BRAKE
            if env_knowledge.ego_distance[1] == FailureType.no_front_vehicle:
                self.knowledge.target_speed = env_knowledge.speed_limit
                return Plan.SPEED_CONTROLLER
            if env_knowledge.other_vehicles[self.knowledge.leader_id].speed[1] == FailureType.no_failure or \
                    env_knowledge.other_vehicles[self.knowledge.leader_id].acceleration[1] == FailureType.no_failure or \
                    env_knowledge.other_vehicles[self.knowledge.front_vehicle_id].speed[1] == FailureType.no_failure or \
                    env_knowledge.other_vehicles[self.knowledge.front_vehicle_id].acceleration[1] == FailureType.no_failure:
                return Plan.CACC_CONTROLLER
        elif self.knowledge.current_controller == ControllerType.SPEED:

        elif self.knowledge.current_controller == ControllerType.EMERGENCY_BRAKE:

