import PySimpleGUI as sg

from queue import Queue
from implementation.configuration_parameter import *
from implementation.carla_client.simulation_state import SimulationState
from implementation.knowledge.base_attribute import Weather

class UserControlWindow(object):
    """Creates the Pygame Window, which is used as control panel for the user to change the simulation"""

    def __init__(self, carla_control_client_queue):
        self.window = None
        self.layout = None
        self.carla_control_client_queue: Queue = carla_control_client_queue

        self.__initialize_layout()
        self.__initialize_window()

        self.simulation_running = True

    def __initialize_layout(self):
        self.layout = [
            [sg.Checkbox("ego_speed_available", default=True, key="-EGO_SPEED-")],
            [sg.Checkbox("ego_acceleration_available", default=True, key="-EGO_ACCELERATION-")],
            [sg.Checkbox("ego_distance_available",default=True, key="-EGO_DISTANCE-")],
            [sg.Checkbox("other_acceleration_available", default=True, key="-OTHER_ACCELERATION-")],
            [sg.Checkbox("other_speed_available", default=True, key="-OTHER_SPEED-")],
            [sg.Checkbox("other_emergency_brake_available", default=True, key="-OTHER_EMERGENCY_BRAKE-")],
            [sg.Checkbox("other_braking_light_available", default=True, key="-OTHER_BRAKING_LIGHT-")],
            [sg.InputText("60", key="-SPEED_LIMIT-"), sg.Text("Speed Limit")],
            [sg.InputText("60", key="-LEADER_SPEED-"), sg.Text("Leader Speed")],
            [sg.Checkbox("Leader perform emergency brake", default=False, key="-PERFORM_EMERGENCY_BRAKE-")],
            [sg.Slider(range=(1,100), default_value=100, resolution=25,
                       orientation="horizontal", key="-CONNECTION_STRENGTH-"),
                sg.Text("Connection Strength")],
            [sg.Radio("Sunshine", "WEATHER", default=True, key="-SUNSHINE-"),
                sg.Radio("Rain", "WEATHER", default=False, key="-RAIN-"),
                sg.Radio("Fog", "WEATHER", default=False, key="-FOG-")],
            [sg.Button("Update Simulation", key="-UPDATE-"),
                sg.Button("Reset Simulation", key="-RESET-"),
                sg.Button("Exit Simulation", key="-EXIT-")]
        ]

    def __initialize_window(self):
        self.window = sg.Window("User Control", self.layout)

    def window_loop(self):

        while self.simulation_running:
            event, values = self.window.read()

            print(event)
            print(values)

            if event == sg.WIN_CLOSED:
                break
            elif event == "-UPDATE-":
                self.update_simulation(values)
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
        simulation_state.ego_speed_available = values["-EGO_SPEED-"]
        simulation_state.ego_acceleration_available = values["-EGO_ACCELERATION-"]
        simulation_state.ego_distance_available = values["-EGO_DISTANCE-"]
        simulation_state.other_speed_available = values["-OTHER_SPEED-"]
        simulation_state.other_acceleration_available = values["-OTHER_ACCELERATION-"]
        simulation_state.other_emergency_brake_available = values["-OTHER_EMERGENCY_BRAKE-"]
        simulation_state.other_braking_light_available = values["-OTHER_BRAKING_LIGHT-"]
        simulation_state.other_perform_emergency_brake = values["-PERFORM_EMERGENCY_BRAKE-"]
        simulation_state.leader_speed = values["-LEADER_SPEED-"]
        simulation_state.speed_limit = values["-SPEED_LIMIT-"]
        simulation_state.connection_strength = values["-CONNECTION_STRENGTH-"]

        if values["-SUNSHINE-"]:
            simulation_state.weather = Weather(1)
        elif values["-RAIN-"]:
            simulation_state.weather = Weather(2)
        elif values["-FOG-"]:
            simulation_state.weather = Weather(3)

        self.carla_control_client_queue.put(simulation_state)

    def exit_window(self):
        self.window.close()


