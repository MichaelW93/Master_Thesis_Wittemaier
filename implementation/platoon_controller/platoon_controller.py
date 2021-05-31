from typing import TYPE_CHECKING

from implementation.data_classes import EnvironmentKnowledge
from implementation.platoon_controller.monitor.monitor import Monitor
from implementation.platoon_controller.planner.planner import Planner
from implementation.platoon_controller.analyzer.analyzer import Analyzer
from implementation.platoon_controller.executer.executor import Executor
from implementation.platoon_controller.knowledge.knowledge import Knowledge

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

    def run_step(self, timestamp: "carla.Timestamp", weather: "carla.WeatherParameters", speed_limit: float) -> EnvironmentKnowledge:

        environment_knowledge = self.monitor.run_step(timestamp, weather, speed_limit)
        adaptation_technique = self.analyzer.run_step(environment_knowledge)
        plan = self.planner.run_step(adaptation_technique, environment_knowledge)
        self.executor.run_step(plan, environment_knowledge)

        return environment_knowledge

    def destroy(self):
        # TODO
        pass
