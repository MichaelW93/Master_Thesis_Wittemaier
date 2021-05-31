from implementation.platoon_controller.knowledge.knowledge import Knowledge
from implementation.data_classes import *

class Analyzer(object):

    def __init__(self, knowledge: Knowledge):
        self.knowledge: Knowledge = knowledge

    def run_step(self, analyzer_input: EnvironmentKnowledge) -> AdaptationTechnique:

        if (analyzer_input.ego_speed[0] * 3.6) > 56:
            return AdaptationTechnique.STRUCTURAL
        else:
            return AdaptationTechnique.NO_ADAPTATION
