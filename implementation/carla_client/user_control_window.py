import PySimpleGUI as sg

from queue import Queue
from implementation.configuration_parameter import *
from implementation.data_classes import SimulationState
from implementation.platoon_controller.knowledge.base_attribute import Weather
from typing import List

class UserControlWindow(object):
    """Creates the Pygame Window, which is used as control panel for the user to change the simulation"""

    def __init__(self, carla_control_client_queue, leader_id: int, follower_ids: List[int]):
        self.window = None
        self.layout = None
        self.carla_control_client_queue: Queue = carla_control_client_queue

        self.__initialize_layout()
        self.__initialize_window()

        self.simulation_running = True

        self.vehicle_ids = []
        # 0 --> Leader
        # 1 --> Follower 1
        # ...
        self.vehicle_ids.append(leader_id)
        self.vehicle_ids.extend(follower_ids)

    def __initialize_layout(self):

        follower_speed_checkboxes = []
        follower_acceleration_checkboxes = []
        follower_distance_checkboxes = []

        for i in range(NUMBER_OF_MANAGED_VEHICLES):
            follower_speed_checkboxes.append(
                sg.Checkbox(f"managed vehicle {i} speed available", default=True, key=f"-MANAGED_SPEED_{i}-"))
            follower_acceleration_checkboxes.append(
                sg.Checkbox(f"managed vehicle {i} acceleration available", default=True, key=f"-MANAGED_ACCELERATION_{i}-"))

        self.layout = [
            [*follower_distance_checkboxes],
            [*follower_acceleration_checkboxes],
            [*follower_speed_checkboxes],
            [sg.Checkbox("Classify", default=False, key="-CLASSIFY-")],
            [sg.Radio("NA", "TECHNIQUE", default=True, key="-NA-"),
             sg.Radio("PA", "TECHNIQUE", default=False, key="-PA-"),
             sg.Radio("SA", "TECHNIQUE", default=False, key="-SA-"),
             sg.Radio("CA", "TECHNIQUE", default=False, key="-CA-")],
            [sg.Radio("CACC", "CONTROLLER", default=True, key="-CACC-"),
             sg.Radio("ACC", "CONTROLLER", default=False, key="-ACC-"),
             sg.Radio("SPEED", "CONTROLLER", default=False, key="-SPEED-"),
             sg.Radio("BREAK", "CONTROLLER", default=False, key="-BREAK-")],
            [sg.Checkbox("leader_acceleration_available", default=True, key="-LEADER_ACCELERATION-")],
            [sg.Checkbox("leader_speed_available", default=True, key="-LEADER_SPEED-")],
            [sg.Checkbox("Record Data", default=False, key="-RECORD-")],
            [sg.InputText("60", key="-SPEED_LIMIT-"), sg.Text("Speed Limit")],
            [sg.InputText("60", key="-LEADER_TARGET_SPEED-"), sg.Text("Leader target speed")],
            [sg.InputText("60", key="-ENVIRONMENT_TARGET_SPEED-"), sg.Text("Environment vehicles target speed")],
            #[sg.Text("Distance Controller Parameter - k1, k2, k3"), sg.InputText("1", key="-K1-"), sg.InputText("0.2", key="-K2-"), sg.InputText("0.4", key="-K3-")],
            [sg.Slider(range=(1, 100), default_value=100, resolution=25,
                       orientation="horizontal", key="-CONNECTION_STRENGTH-"),
                sg.Text("Connection Strength")],
            [sg.Radio("Sunshine", "WEATHER", default=True, key="-SUNSHINE-"),
                sg.Radio("Rain", "WEATHER", default=False, key="-RAIN-"),
                sg.Radio("Fog", "WEATHER", default=False, key="-FOG-")],
            [sg.Radio("Regular driving", "BRAKE", default=True, key="-REGULAR-"),
             sg.Radio("Emergency Braking", "BRAKE", default=False, key="-EMERGENCY-"),
             sg.Radio("Medium Braking", "BRAKE", default=False, key="-MEDIUM-"),
             sg.Radio("Soft Braking", "BRAKE", default=False, key="-SOFT-")],
            [sg.Button("Update Simulation", key="-UPDATE-"),
                sg.Button("Reset Simulation", key="-RESET-"),
                sg.Button("Exit Simulation", key="-EXIT-")],
            [sg.Button("Cut-In vehicle", key="-CUT_IN-"), sg.Button("Remove Environment Vehicle", key="-REMOVE-")]
        ]

    def __initialize_window(self):
        self.window = sg.Window("User Control", self.layout)

    def window_loop(self):

        while self.simulation_running:
            event, values = self.window.read()

            if DEBUG_MODE:
                print(values)

            if event == sg.WIN_CLOSED:
                break
            elif event == "-UPDATE-":
                self.update_simulation(values)
            elif event == "-CUT_IN-":
                self.carla_control_client_queue.put("CUT_IN")
            elif event == "-REMOVE-":
                self.carla_control_client_queue.put("REMOVE")
            elif event == "-RESET-":
                self.carla_control_client_queue.put("RESET")
            elif event == "-EXIT-":
                self.carla_control_client_queue.put("EXIT")
                break
            else:
                return

        self.exit_window()

    def update_simulation(self, values):
        simulation_state = SimulationState()
        if len(self.vehicle_ids) > 1:
            for i in range(NUMBER_OF_MANAGED_VEHICLES):
                id = self.vehicle_ids[i + 1]
                simulation_state.vehicles_speed_available[id] = values[f"-MANAGED_SPEED_{i}-"]
                simulation_state.vehicles_acceleration_available[id] = values[f"-MANAGED_ACCELERATION_{i}-"]

        simulation_state.vehicles_speed_available[self.vehicle_ids[0]] = values["-LEADER_SPEED-"]
        simulation_state.vehicles_acceleration_available[self.vehicle_ids[0]] = values["-LEADER_ACCELERATION-"]
        simulation_state.leader_target_speed = float(values["-LEADER_TARGET_SPEED-"])
        simulation_state.environment_vehicles_target_speed = float(values["-ENVIRONMENT_TARGET_SPEED-"])
        simulation_state.speed_limit = float(values["-SPEED_LIMIT-"])
        simulation_state.connection_strength = float(values["-CONNECTION_STRENGTH-"])
        simulation_state.record_data = values["-RECORD-"]
        #simulation_state.k1 = float(values["-K1-"])
        #simulation_state.k2 = float(values["-K2-"])
        #simulation_state.k3 = float(values["-K3-"])
        simulation_state.classify = values["-CLASSIFY-"]
        simulation_state.no_adap = values["-NA-"]
        simulation_state.par_adap = values["-PA-"]
        simulation_state.struc_adap = values["-SA-"]
        simulation_state.com_adap = values["-CA-"]

        simulation_state.emergency_brake = values["-EMERGENCY-"]
        simulation_state.medium_brake = values["-MEDIUM-"]
        simulation_state.soft_brake = values["-SOFT-"]

        if values["-CACC-"]:
            simulation_state.controller_follower_1 = "CACC"
        elif values["-ACC-"]:
            simulation_state.controller_follower_1 = "ACC"
        elif values["-SPEED-"]:
            simulation_state.controller_follower_1 = "SPEED"
        elif values["-BREAK-"]:
            simulation_state.controller_follower_1 = "BREAK"


        if values["-SUNSHINE-"]:
            simulation_state.weather = Weather(1)
        elif values["-RAIN-"]:
            simulation_state.weather = Weather(2)
        elif values["-FOG-"]:
            simulation_state.weather = Weather(3)

        print(simulation_state)

        self.carla_control_client_queue.put(simulation_state)

    def exit_window(self):
        self.window.close()


