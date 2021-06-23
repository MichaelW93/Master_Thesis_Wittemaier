from typing import TYPE_CHECKING

from implementation.data_classes import EnvironmentKnowledge
from implementation.platoon_controller.monitor.monitor import Monitor
from implementation.platoon_controller.planner.planner import Planner
from implementation.platoon_controller.analyzer.analyzer import Analyzer
from implementation.platoon_controller.executer.executor import Executor
from implementation.platoon_controller.knowledge.knowledge import Knowledge
from implementation.data_classes import Plan

if TYPE_CHECKING:
    from implementation.vehicle.vehicles import *


class PlatoonController(object):

    def __init__(self, ego_vehicle):
        self.ego_vehicle: "ManagedVehicle" = ego_vehicle
        self.knowledge: Knowledge = Knowledge()
        self.monitor: Monitor = Monitor(self.knowledge, self.ego_vehicle)
        self.analyzer: Analyzer = Analyzer(self.knowledge)
        self.planner: Planner = Planner(self.knowledge)
        self.executor: Executor = Executor(self.knowledge, self.ego_vehicle)

        self.plan: Plan = Plan.NO_CHANGE
        self.new_plan_available = False

    def run_step(self, timestamp: "carla.Timestamp", weather: "carla.WeatherParameters", speed_limit: float) -> EnvironmentKnowledge:

        environment_knowledge = self.monitor.run_step(timestamp, weather, speed_limit)
        adaptation_technique = self.analyzer.run_step(environment_knowledge)
        if not self.new_plan_available:
            self.plan = self.planner.run_step(adaptation_technique, environment_knowledge)
        else:
            self.new_plan_available = False

        print(f"{self.ego_vehicle.role_name}: \n"
              f"Current Controller: {self.knowledge.current_controller} \n"
              f"Technique from tree: {adaptation_technique}, \n"
              f"Plan: {self.plan}\n"
              f"")

        self.executor.run_step(self.plan, environment_knowledge)

        return environment_knowledge

    def set_manual_plan(self, plan):
        self.new_plan_available = True
        self.plan = plan
    def destroy(self):
        # TODO
        pass
