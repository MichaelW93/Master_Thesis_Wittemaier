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
        self.vehicle_ids.append(follower_ids)

    def __initialize_layout(self):

        follower_speed_checkboxes = []
        follower_acceleration_checkboxes = []
        follower_distance_checkboxes = []

        for i in range(NUMBER_OF_MANAGED_VEHICLES):
            follower_speed_checkboxes.append(
                sg.Checkbox(f"managed vehicle {i} speed available", default=True, key=f"-MANAGED_SPEED_{i}-"))
            follower_acceleration_checkboxes.append(
                sg.Checkbox(f"managed vehicle {i} acceleration available", default=True, key=f"-MANAGED_ACCELERATION_{i}-"))
            follower_distance_checkboxes.append(
                sg.Checkbox(f"managed vehicle {i} distance available", default=True, key=f"-MANAGED_DISTANCE_{i}-"))

        self.layout = [
            [*follower_distance_checkboxes],
            [*follower_acceleration_checkboxes],
            [*follower_speed_checkboxes],
            [sg.Checkbox("leader_acceleration_available", default=True, key="-LEADER_ACCELERATION-")],
            [sg.Checkbox("leader_speed_available", default=True, key="-LEADER_SPEED-")],
            [sg.InputText("60", key="-SPEED_LIMIT-"), sg.Text("Speed Limit")],
            [sg.InputText("60", key="-LEADER_TARGET_SPEED-"), sg.Text("Leader target speed")],
            [sg.InputText("60", key="-ENVIRONMENT_TARGET_SPEED-"), sg.Text("Environment vehicles target speed")],
            [sg.Slider(range=(1, 100), default_value=100, resolution=25,
                       orientation="horizontal", key="-CONNECTION_STRENGTH-"),
                sg.Text("Connection Strength")],
            [sg.Radio("Sunshine", "WEATHER", default=True, key="-SUNSHINE-"),
                sg.Radio("Rain", "WEATHER", default=False, key="-RAIN-"),
                sg.Radio("Fog", "WEATHER", default=False, key="-FOG-")],
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
        for i in range(NUMBER_OF_MANAGED_VEHICLES):
            simulation_state.vehicles_speed_available[self.vehicle_ids[i + 1]] = values[f"-MANAGED_SPEED_{i}-"]
            simulation_state.vehicles_acceleration_available[self.vehicle_ids[i + 1]] = values[f"-MANAGED_ACCELERATION_{i}-"]
            simulation_state.followers_distance_available[self.vehicle_ids[i+1]] = values[f"-MANAGED_DISTANCE_{i}-"]

        simulation_state.vehicles_speed_available[self.vehicle_ids[0]] = values["-LEADER_SPEED-"]
        simulation_state.vehicles_acceleration_available[self.vehicle_ids[0]] = values["-LEADER_ACCELERATION-"]
        simulation_state.leader_target_speed = float(values["-LEADER_TARGET_SPEED-"])
        simulation_state.environment_vehicles_target_speed = float(values["-ENVIRONMENT_TARGET_SPEED-"])
        simulation_state.speed_limit = float(values["-SPEED_LIMIT-"])
        simulation_state.connection_strength = float(values["-CONNECTION_STRENGTH-"])

        if values["-SUNSHINE-"]:
            simulation_state.weather = Weather(1)
        elif values["-RAIN-"]:
            simulation_state.weather = Weather(2)
        elif values["-FOG-"]:
            simulation_state.weather = Weather(3)

        self.carla_control_client_queue.put(simulation_state)

    def exit_window(self):
        self.window.close()


