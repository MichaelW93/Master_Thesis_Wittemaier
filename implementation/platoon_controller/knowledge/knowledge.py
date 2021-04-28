from implementation.platoon_controller.knowledge.logger import Logger
from implementation.data_classes import EnvironmentKnowledge, SystemState


class Knowledge(object):

    def __init__(self):
        self.logger = Logger()
        self.current_system_state: SystemState = SystemState()

    def store_environment_knowledge(self, environment_knowledge: EnvironmentKnowledge) -> None:
        self.logger.update_log(environment_knowledge)

    def get_current_environment_knowledge(self) -> EnvironmentKnowledge:
        return self.logger.get_current_simulation_step()
