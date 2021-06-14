from implementation.carla_client.scenarios import *
import random
from typing import TYPE_CHECKING, Optional
if TYPE_CHECKING:
    from implementation.carla_client.carla_control_client import CarlaControlClient

class ScenarioController(object):

    def __init__(self, control_client, carla_world, comm_handler):
        self.leader = None
        self.managed_vehicles = []
        self.carla_world = carla_world
        self.current_scenario: Optional[Scenario] = None
        self.sim_state = SimulationState()
        self.control_client: "CarlaControlClient" = control_client
        self.communication_handler = comm_handler

    def setup(self, leader, managed_vehicles):
        self.leader = leader
        self.managed_vehicles = managed_vehicles

    def run_step(self):

        if self.current_scenario is None:
            self.select_scenario()
            self.sim_state = self.current_scenario.create_random_sim_state()
            print(self.sim_state)

        if self.current_scenario is not None:
            if not self.current_scenario.done:
                self.current_scenario.run_step(self.sim_state)
            else:
                self.current_scenario.destroy()
                self.control_client.restart_simulation()
                self.current_scenario = None
        return self.sim_state

    def select_scenario(self):
        rand = random.randint(1, 6)
        scenario = Scenarios(rand)

        if scenario == Scenarios.CUT_IN_SCENARIO:
            #self.current_scenario = CutInScenario(self.leader, self.managed_vehicles[0], self.carla_world)
            self.current_scenario = CommunicationFailureScenario(self.leader, self.managed_vehicles)
        elif scenario == Scenarios.LEADER_SPEEDING:
            self.current_scenario = SpeedingLeaderScenario(self.leader, self.managed_vehicles)
        elif scenario == Scenarios.EMERGENCY_BRAKE:
            self.current_scenario = EmergencyBrakeScenario(self.leader, self.managed_vehicles)
        elif scenario == Scenarios.COMMUNICATION_FAILURE:
            self.current_scenario = CommunicationFailureScenario(self.leader, self.managed_vehicles)
        elif scenario == Scenarios.SOFT_BRAKE:
            self.current_scenario = SoftBrakeScenario(self.leader, self.managed_vehicles)
        elif scenario == Scenarios.MEDIUM_BRAKE:
            self.current_scenario = MediumBrakeScenario(self.leader, self.managed_vehicles)

        print("Selected scenario: ", self.current_scenario)

    def reset(self):
        self.leader = None
        self.managed_vehicles = []

