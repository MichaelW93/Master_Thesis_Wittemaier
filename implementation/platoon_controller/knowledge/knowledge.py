from implementation.data_classes import *
from implementation.vehicle.vehicles import *
from implementation.util import *
from typing import List, Dict, Union


class Knowledge(object):

    def __init__(self):
        self.current_system_state: SystemState = SystemState()
        self.leader_id: int = -1
        self.front_vehicle_id: int = -1
        self.current_simulation_step: EnvironmentKnowledge = EnvironmentKnowledge()
        self.simulation_step_history: List[EnvironmentKnowledge] = initialize_list(None, 10)
        self.other_vehicles: Dict[int, Union[OtherVehicle, PlatoonVehicle]] = {}

    def get_current_simulation_step(self) -> EnvironmentKnowledge:
        if self.current_simulation_step is not None:
            return self.current_simulation_step

    def get_past_simulation_step(self, past_simulation_step: int = 1) -> EnvironmentKnowledge:
        """
        Returns the Environment Knowledge of a past simulation step. 0 is the current simulation step,
        20 the oldest available.
        :param past_simulation_step:
        :return: EnvironmentKnowledge of the selected simulation step
        """
        if self.simulation_step_history[past_simulation_step] is not None:
            return self.simulation_step_history[past_simulation_step]

    def update_log(self, environment_knowledge: EnvironmentKnowledge) -> None:
        """
        Updates the log of the environment_knowledge. Moves down all old simulation steps, puts the new
        data at index 0 and updates the current_simulation_step attribute.
        :param environment_knowledge:
        :return:
        """
        for i, knowledge in reversed(list(enumerate(self.simulation_step_history))):
            if i == 0:
                self.current_simulation_step = environment_knowledge
                self.simulation_step_history[i] = environment_knowledge
            else:
                self.simulation_step_history[i] = self.simulation_step_history[i - 1]

    def store_environment_knowledge(self, environment_knowledge: EnvironmentKnowledge) -> None:
        self.update_log(environment_knowledge)

    def get_current_environment_knowledge(self) -> EnvironmentKnowledge:
        return self.get_current_simulation_step()
