from implementation.data_classes import EnvironmentKnowledge, MonitorInputData
from implementation.platoon_controller.monitor.monitor import Monitor

class PlatoonController(object):

    def __init__(self):
        self.monitor: Monitor = Monitor()
        self.analyzer
        self.planner
        self.executor

    def run_step(self, data: MonitorInputData) -> None:

        analyzer_input = self.monitor.run_step(data)

    def destroy(self):
        # TODO
        pass
