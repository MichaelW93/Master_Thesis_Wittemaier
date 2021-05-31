import carla
from collections import deque
import numpy as np
import math
from enum import Enum
from abc  import ABC
from matplotlib import pyplot as pyp
from typing import TYPE_CHECKING, Optional
from implementation.configuration_parameter import *

if TYPE_CHECKING:
    from implementation.data_classes import *
    from implementation.vehicle.vehicles import Vehicle


class ControllerType(Enum):
    DISTANCE = 1
    SPEED = 2
    BRAKE = 3


class Controller(ABC):

    def __init__(self, ego_vehicle):
        self.ego_vehicle: carla.Vehicle = ego_vehicle
        self.controller_type: ControllerType

class DistanceController(Controller):

    def __init__(self, ego_vehicle):
        super(DistanceController, self).__init__(ego_vehicle)
        self.k1: float = 1.0
        self.k2: float = 0.2
        self.k3: float = 0.4

        self.max_acceleration: float = 2.75
        self.max_deceleration: float = -5
        self.timegap: float = 0.5

        self.controller_type: ControllerType = ControllerType.DISTANCE
        self.speed_controller = PIDController(self.ego_vehicle)

    def run_step(self, environment_knowledge: "EnvironmentKnowledge"):
        target_speed = self.__calculate_throttle_value(environment_knowledge)
        return self.speed_controller.run_step(target_speed)

    def __calculate_throttle_value(self, environment_knowledge: "EnvironmentKnowledge"):

        speed_front = 0
        acc = []
        for _, vehicle in environment_knowledge.other_vehicles.items():
            acc.append(vehicle.acceleration[0])
            if vehicle.is_front_vehicle:
                speed_front = vehicle.speed[0]

        speed_ego = environment_knowledge.ego_speed[0]
        distance = environment_knowledge.ego_distance[0]
        r = self.__get_min_distance(speed_ego)  # distance at standstill
        timestep = environment_knowledge.time_to_last_step

        des_distance = r + self.timegap * speed_ego
        error = distance - des_distance
        # acc = 0

        acc_ego = self.k3 * min(acc) + self.k2 * (speed_front - speed_ego) + self.k1 * error
        if acc_ego > 0:
            acc_ego = max(self.max_acceleration, acc_ego)
        else:
            acc_ego = min(self.max_deceleration, acc_ego)
        if acc_ego > 0:
            ego_target_speed = ((0.5 * acc_ego * timestep * timestep + speed_ego) * 3.6) + 0.6
        else:
            ego_target_speed = ((0.5 * acc_ego * timestep * timestep + speed_ego) * 3.6)
        return ego_target_speed

    @staticmethod
    def __get_min_distance(ego_speed) -> float:
        if 0 < ego_speed < 5:
            return 2
        elif 5 < ego_speed < 9:
            d = 2 - 0.5 * (ego_speed - 5)
            return d
        else:
            return 0

class SpeedController(Controller):

    def __init__(self, ego_vehicle):

        self.pid_controller = PIDController(ego_vehicle)

    def run_step(self, environment_knowledge: "EnvironmentKnowledge"):

        return self.pid_controller.run_step(target_speed= environment_knowledge.speed_limit)


class PIDController(object):
    """
    VehiclePIDController is the combination of two PID controllers
    (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, ego_vehicle, max_throttle=0.75, max_brake=0.3):
        """
        Constructor method.

        :param ego_vehicle: actor to apply to local planner logic onto
        :param args_longitudinal: dictionary of arguments to set the longitudinal
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        """

        args_longitudinal = {
            'K_P': MANAGED_VEHICLE_CONTROLLER_KP,
            'K_D': MANAGED_VEHICLE_CONTROLLER_KD,
            'K_I': MANAGED_VEHICLE_CONTROLLER_KI,
            'dt': 1 / CARLA_SERVER_FPS
        }

        self.ego_vehicle = ego_vehicle
        self.max_brake = max_brake
        self.max_throttle = max_throttle
        self.controller_type: ControllerType = ControllerType.SPEED

        self._lon_controller = PIDLongitudinalController(self.ego_vehicle, **args_longitudinal)

    def run_step(self, target_speed: Optional[float] = None) -> carla.VehicleControl:

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


class BrakeController(Controller):

    def __init__(self, ego_vehicle):
        super(BrakeController, self).__init__(ego_vehicle)

    def run_step(self, env_knowledge: "EnvironmentKnowledge"):

        max_deceleration = 0
        max_brake = 0

        for vehicle in env_knowledge.other_vehicles.values():
            if vehicle.measured_acceleration[1] == FailureType.no_failure:
                if vehicle.acceleration[1] == FailureType.no_failure:
                    vehicle_dec = min(vehicle.measured_acceleration[0], vehicle.acceleration[0])
                else:
                    vehicle_dec = vehicle.measured_acceleration[0]
            else:
                if vehicle.acceleration[1] == FailureType.no_failure:
                    vehicle_dec = vehicle.acceleration[0]
                else:
                    vehicle_dec = 0
            if vehicle.brake > max_brake:
                max_brake = vehicle.brake
            if vehicle_dec < max_deceleration:
                max_deceleration = vehicle_dec

        control = carla.VehicleControl()
        if max_brake > 0.9:
            control.brake = 1
            return control
        else:
            control.brake = max_brake
            return control

        target_speed = 0.5 * max_deceleration * env_knowledge.time_to_last_step**2 + env_knowledge.ego_speed[0]



def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)