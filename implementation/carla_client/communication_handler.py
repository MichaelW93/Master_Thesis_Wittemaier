from typing import TYPE_CHECKING, List, Dict, Union
from implementation.data_classes import CommunicationData, SimulationState
from implementation.util import *
from implementation.configuration_parameter import *
from implementation.vehicle.vehicles import ManagedVehicle, LeaderVehicle, EnvironmentVehicle


class CommunicationHandler(object):

    def __init__(self):
        self.vehicles_data: List[Dict[int, CommunicationData]] = None
        self.delay_in_simulation_ticks = 1
        self.sim_state: SimulationState = SimulationState()
        self.vehicles: Dict[int, Union[ManagedVehicle, LeaderVehicle, EnvironmentVehicle]] = None

    def setup(self):
        self.vehicles_data: List[Dict[int, CommunicationData]] = initialize_list({}, 10)
        self.delay_in_simulation_ticks = 1
        self.sim_state: SimulationState = SimulationState()
        self.vehicles: Dict[int, Union[ManagedVehicle, LeaderVehicle, EnvironmentVehicle]] = {}

    def run_step(self, sim_state: SimulationState):
        # received data is updated in the next simulation step --> always 1 tick delay
        self.sim_state = sim_state
        delay = self.calculate_delay_in_simulation_tick(self.sim_state.connection_strength)
        if delay is None:
            self.delay_in_simulation_ticks = None
        else:
            self.delay_in_simulation_ticks = delay - 1
        self.update_vehicle_data()

    def update_vehicle_data(self):
        for i in reversed(range(len(self.vehicles_data))):
            if i == 0:
                self.vehicles_data[i] = {}
            else:
                self.vehicles_data[i] = self.vehicles_data[i - 1]

    def set_vehicle_data(self, data: CommunicationData) -> None:
        data_dict = self.vehicles_data[0]
        data_dict[data.vehicle_id] = data

    def get_vehicle_data(self, vehicle_id) -> Optional[CommunicationData]:
        if self.delay_in_simulation_ticks is None:
            data_dict = self.vehicles_data[1]
            data = data_dict[vehicle_id]
            data.speed = None
            data.acceleration = None
        else:
            data_dict = self.vehicles_data[self.delay_in_simulation_ticks]
            if vehicle_id in data_dict:
                data = data_dict[vehicle_id]
                if not self.sim_state.vehicles_data_available[vehicle_id]:
                    data.speed = None
                    data.acceleration = None
                    data.throttle = None
                    data.brake = None
                    data.front_id = None
                    data.steering = None
            else:
                return
        return data

    def get_all_vehicle_data(self) -> Dict[int, CommunicationData]:
        vehicle_comm_data: Dict[int, CommunicationData] = {}
        for vehicle in self.vehicles.values():
            vehicle_comm_data[vehicle.ego_vehicle.id] = self.get_vehicle_data(vehicle.ego_vehicle.id)
        return vehicle_comm_data

    def calculate_delay_in_simulation_tick(self, connection_strength) -> Optional[int]:
        if connection_strength == 100:
            delay = 50
        elif connection_strength == 75:
            delay = 100
        elif connection_strength == 50:
            delay = 200
        elif connection_strength == 25:
            delay = 500
        else:
            delay = -1
            return None
        delay_in_simulation_ticks = delay/((1/CARLA_SERVER_FPS)*1000)  # convert to ms
        return int(round(delay_in_simulation_ticks))

    def reset(self):
        self.vehicles_data = []
        self.vehicles = {}
