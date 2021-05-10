import carla
from collections import deque
import numpy as np
import math

from typing import TYPE_CHECKING, Optional
from implementation.configuration_parameter import *

if TYPE_CHECKING:
    from implementation.data_classes import *
    from implementation.vehicle.vehicles import Vehicle

class Controller(object):

    def __init__(self, ego_vehicle):
        self.ego_vehicle: "Vehicle" = ego_vehicle


class DistanceController(Controller):

    def __init__(self, ego_vehicle):
        super(DistanceController, self).__init__(ego_vehicle)
        self.k1: float = 1.0
        self.k2: float = 0.2
        self.k3: float = 0.4  # proportional gain
        self.k4: float = 0  # derivative gain
        self.theta: float = 0  # reduce noise

        args_long_dict = {
            'K_P': MANAGED_VEHICLE_CONTROLLER_KP,
            'K_D': MANAGED_VEHICLE_CONTROLLER_KD,
            'K_I': MANAGED_VEHICLE_CONTROLLER_KI,
            'dt': 1 / CARLA_SERVER_FPS
        }
        self.speed_controller = VehiclePIDController(self.ego_vehicle, args_long_dict)

    def run_step(self, environment_knowledge: "EnvironmentKnowledge", target_speed):
        control = self.__calculate_throttle_value(environment_knowledge)
        return control

    def __calculate_throttle_value(self, environment_knowledge: "EnvironmentKnowledge"):

        speed_front = 0
        acc = []
        for _, vehicle in environment_knowledge.other_vehicles.items():
            acc.append(vehicle.acceleration[0])
            if vehicle.is_front_vehicle:
                speed_front = vehicle.speed[0]

        speed_ego = environment_knowledge.ego_speed[0]
        distance = environment_knowledge.ego_distance[0]
        headway = 0.5
        r = 2  # distance at standstill
        timestep = environment_knowledge.time_to_last_step

        des_distance = r + headway*speed_ego

        #print("\n", self.ego_vehicle.role_name, "\n")
        #print("Desired distance: ", des_distance)
        #print("Actual distance: ", distance)

        error = distance - des_distance

        acc_ego = self.k3 * min(acc) + self.k2 * (speed_front - speed_ego) + self.k1 * error
        #print("Target accel ", acc_ego)
        #print("Ego speed: ", speed_ego * 3.6)
        if acc_ego > 0:
            ego_target_speed = ((0.5 * acc_ego * timestep * timestep + speed_ego) * 3.6) + 0.6
        else:
            ego_target_speed = ((0.5 * acc_ego * timestep * timestep + speed_ego) * 3.6)
        #print("Target speed: ", ego_target_speed)

        control = self.speed_controller.run_step(None, ego_target_speed)
        #print(control)
        return control

    def __calculate_feedforward_filter(self):
        filter_value = (self.k3 + self.k4)/(1 + self.theta)
        return filter_value


class VehiclePIDController(Controller):
    """
    VehiclePIDController is the combination of two PID controllers
    (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, ego_vehicle, args_longitudinal, max_throttle=0.75, max_brake=0.3):
        """
        Constructor method.

        :param ego_vehicle: actor to apply to local planner logic onto
        :param args_longitudinal: dictionary of arguments to set the longitudinal
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        """

        super(VehiclePIDController, self).__init__(ego_vehicle)
        self.max_brake = max_brake
        self.max_throttle = max_throttle

        self._world = self.ego_vehicle.ego_vehicle.get_world()
        self._lon_controller = PIDLongitudinalController(self.ego_vehicle.ego_vehicle, **args_longitudinal)

    def run_step(self, environment_knowledge: "Optional[EnvironmentKnowledge]" = None,
                 target_speed: Optional[float] = None) -> carla.VehicleControl:

        acceleration = self._lon_controller.run_step(target_speed)
        control = carla.VehicleControl()
        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throttle)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class PIDLongitudinalController(object):
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._dt = dt
        self._error_buffer = deque(maxlen=10)

    def run_step(self, target_speed, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

            :param target_speed: target speed in Km/h
            :param debug: boolean for debugging
            :return: throttle control
        """
        current_speed = get_speed(self._vehicle)

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

            :param target_speed:  target speed in Km/h
            :param current_speed: current speed of the vehicle in Km/h
            :return: throttle/brake control
        """

        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)