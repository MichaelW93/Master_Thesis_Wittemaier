from implementation.data_classes import *
from implementation.vehicle.vehicles import *
from implementation.util import *
from typing import List, Dict, Union, TYPE_CHECKING
from implementation.vehicle.controller import ControllerType


class Knowledge(object):

    def __init__(self):
        self.current_system_state: SystemState = SystemState()
        self.leader_id: int = -1
        self.front_vehicle_id: int = -1
        self.current_simulation_step: EnvironmentKnowledge = EnvironmentKnowledge()
        self.simulation_step_history: List[EnvironmentKnowledge] = initialize_list(None, 10)
        self.other_vehicles: Dict[int, OtherVehicle] = {}
        self.current_controller: "ControllerType" = ControllerType.SPEED
        self.target_speed: float = 60
        self.cont_max_acc: float = 5
        self.cont_max_dec: float = -5
        self.timegap: float = 0.5

    def get_current_simulation_step(self) -> EnvironmentKnowledge:
        if self.simulation_step_history[0] is not None:
            return self.simulation_step_history[0]
        else:
            tmp = EnvironmentKnowledge()
            return tmp

    def get_multiple_steps(self, number_of_steps) -> List[EnvironmentKnowledge]:

        return self.simulation_step_history[:number_of_steps - 1]

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
        self.simulation_step_history.insert(0, environment_knowledge)
        del self.simulation_step_history[-1]
        #print("List start: ", self.simulation_step_history[0])
        #print("List second place: ",self.simulation_step_history[1])

    def store_environment_knowledge(self, environment_knowledge: EnvironmentKnowledge) -> None:
        self.update_log(environment_knowledge)

    def get_current_environment_knowledge(self) -> EnvironmentKnowledge:
        return self.get_current_simulation_step()
