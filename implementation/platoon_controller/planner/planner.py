from implementation.platoon_controller.knowledge.knowledge import Knowledge
from implementation.data_classes import *


class Planner(object):

    def __init__(self, knowledge: Knowledge):
        self.knowledge = knowledge

    def run_step(self, adaptation_technique: AdaptationTechnique, environment_knowledge: EnvironmentKnowledge):

        if adaptation_technique == AdaptationTechnique.STRUCTURAL:
            return Plan.CACC_CONTROLLER
        elif adaptation_technique == AdaptationTechnique.PARAMETER:
            pass
        else:
            pass
