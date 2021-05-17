import carla
from collections import deque
import numpy as np
import math

from typing import TYPE_CHECKING, Optional
from implementation.configuration_parameter import *

if TYPE_CHECKING:
    from implementation.data_classes import *
    from implementation.vehicle.vehicles import Vehicle


class ControllerType(Enum):
    CACC = 1
    ACC = 2
    SPEED = 3
    EMERGENCY_BRAKE = 4


class Controller(object):

    def __init__(self, ego_vehicle):
        self.ego_vehicle: "Vehicle" = ego_vehicle
        self.controller_type: ControllerType


class CACCController(Controller):

    def __init__(self, ego_vehicle):
        super(CACCController, self).__init__(ego_vehicle)
        self.k1: float = 1.0
        self.k2: float = 0.2
        self.k3: float = 0.4  # proportional gain
        self.k4: float = 0  # derivative gain
        self.theta: float = 0  # reduce noise

        self.MAX_ACCELERATION: float = 5
        self.MAX_DECELERATION: float = -5

        self.controller_type: ControllerType = ControllerType.CACC

        args_long_dict = {
            'K_P': MANAGED_VEHICLE_CONTROLLER_KP,
            'K_D': MANAGED_VEHICLE_CONTROLLER_KD,
            'K_I': MANAGED_VEHICLE_CONTROLLER_KI,
            'dt': 1 / CARLA_SERVER_FPS
        }
        self.speed_controller = SpeedController(self.ego_vehicle, args_long_dict)

    def run_step(self, environment_knowledge: "EnvironmentKnowledge" = None):
        target_speed = self.__calculate_throttle_value(environment_knowledge)
        return target_speed

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
        if acc_ego > 0:
            acc_ego = max(self.MAX_ACCELERATION, acc_ego)
        else:
            acc_ego = min(self.MAX_DECELERATION, acc_ego)
        #print("Target accel ", acc_ego)
        #print("Ego speed: ", speed_ego * 3.6)
        if acc_ego > 0:
            ego_target_speed = ((0.5 * acc_ego * timestep * timestep + speed_ego) * 3.6) + 0.6
        else:
            ego_target_speed = ((0.5 * acc_ego * timestep * timestep + speed_ego) * 3.6)
        #print("Target speed: ", ego_target_speed)
        #print(control)
        return ego_target_speed


class ACCController(Controller):

    def __init__(self, ego_vehicle):
        super(ACCController, self).__init__(ego_vehicle)

        self.k3: float = 1
        self.k2: float = 0.2
        self.k1: float = 0.4

        self.MAX_ACCELERATION: float = 5
        self.MAX_DECELERATION: float = -5

        args_long_dict = {
            'K_P': MANAGED_VEHICLE_CONTROLLER_KP,
            'K_D': MANAGED_VEHICLE_CONTROLLER_KD,
            'K_I': MANAGED_VEHICLE_CONTROLLER_KI,
            'dt': 1 / CARLA_SERVER_FPS
        }
        self.speed_controller = SpeedController(ego_vehicle, args_long_dict)

    def run_step(self, env_knowledge: "EnvironmentKnowledge" = None) -> carla.VehicleControl:
        target_speed = self.__calculate_target_speed(env_knowledge)

        return target_speed

    def __calculate_target_speed(self, env_knowledge: "EnvironmentKnowledge") -> float:

        speed_front = env_knowledge.other_vehicles[env_knowledge.front_vehicle_id].measured_speed[0]
        acc_front = env_knowledge.other_vehicles[env_knowledge.front_vehicle_id].measured_acceleration[0]

        speed_ego = env_knowledge.ego_speed[0]
        distance = env_knowledge.ego_distance[0]
        headway = 1
        r = 2  # distance at standstill
        timestep = env_knowledge.time_to_last_step

        des_distance = r + headway * speed_ego
        error = distance - des_distance

        acc_ego = self.k3 * acc_front + self.k2 * (speed_front - speed_ego) + self.k1 * error
        if acc_ego > 0:
            acc_ego = max(self.MAX_ACCELERATION, acc_ego)
        else:
            acc_ego = min(self.MAX_DECELERATION, acc_ego)
        # print("Target accel ", acc_ego)
        # print("Ego speed: ", speed_ego * 3.6)
        if acc_ego > 0:
            ego_target_speed = ((0.5 * acc_ego * timestep * timestep + speed_ego) * 3.6) + 0.6
        else:
            ego_target_speed = ((0.5 * acc_ego * timestep * timestep + speed_ego) * 3.6)

        return ego_target_speed


class SpeedController(Controller):
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

        super(SpeedController, self).__init__(ego_vehicle)
        self.max_brake = max_brake
        self.max_throttle = max_throttle
        self.controller_type: ControllerType = ControllerType.SPEED

        self._lon_controller = PIDLongitudinalController(self.ego_vehicle.ego_vehicle, **args_longitudinal)

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


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)