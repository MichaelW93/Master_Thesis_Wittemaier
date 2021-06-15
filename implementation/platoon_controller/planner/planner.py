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

    def process_context_adaptation(self, env_knowledge: "EnvironmentKnowledge"):
        if self.knowledge.current_controller == ControllerType.DISTANCE:
            return Plan.SWITCH_TO_BRAKE

    def process_structural_adaptation(self, env_knowledge: "EnvironmentKnowledge"):

        controller = self.knowledge.current_controller
        front_vehicle = env_knowledge.other_vehicles[self.knowledge.front_vehicle_id]

        if controller == ControllerType.DISTANCE:
            # obey speed limit
            if (env_knowledge.ego_speed_tuple[0] * 3.6) > env_knowledge.speed_limit + 4.5:
                return Plan.SWITCH_TO_SPEED

            # Keep safe distance
            if env_knowledge.speed_diff_to_front < - 10.0:  # S3
                return Plan.SWITCH_TO_BRAKE
            if -10 < env_knowledge.speed_diff_to_front < -5:  # S2
                if env_knowledge.ego_distance_tuple[0] > env_knowledge.desired_distance:  # Better than C1
                    return Plan.NO_CHANGE
                else:  # C1 - C3
                    return Plan.SWITCH_TO_BRAKE
            if -5 < env_knowledge.speed_diff_to_front < 0:  # S1
                if env_knowledge.ego_distance_tuple[0] < env_knowledge.desired_distance - 2:  # C2
                    return Plan.SWITCH_TO_BRAKE
            if env_knowledge.ego_distance_tuple[0] < (env_knowledge.desired_distance / 2):
                return Plan.SWITCH_TO_BRAKE

            # Performance improvements
            if env_knowledge.desired_distance * 1.5 < env_knowledge.ego_distance_tuple[0] and \
                    env_knowledge.ego_speed_tuple[0] < front_vehicle.speed_tuple[0] and \
                    env_knowledge.ego_speed_tuple[0] * 3.6 < env_knowledge.speed_limit:
                return Plan.SWITCH_TO_SPEED
        elif controller == ControllerType.SPEED:
            if env_knowledge.ego_distance_tuple[0] < env_knowledge.desired_distance * 1.5 and \
                    front_vehicle.speed_tuple[0] * 3.6 < env_knowledge.speed_limit + 3:
                if front_vehicle.speed_tuple[1] == FailureType.no_failure and \
                        front_vehicle.acceleration_tuple[1] == FailureType.no_failure:
                    return Plan.SWITCH_TO_CACC
                else:
                    return Plan.SWITCH_TO_ACC
        elif controller == ControllerType.BRAKE:
            if front_vehicle.speed_tuple[1] == FailureType.no_failure and \
                    front_vehicle.acceleration_tuple[1] == FailureType.no_failure:
                return Plan.SWITCH_TO_CACC
            else:
                return Plan.SWITCH_TO_ACC
