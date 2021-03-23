from implementation.configuration_parameter import *
from enum import Enum


class BaseAttribute(object):

    def __init__(self, value):
        self.value = value


class EgoSpeed(BaseAttribute):

    def __init__(self, value):
        super().__init__(value)
        self.range = EGO_SPEED_RANGE


class EgoAcceleration(BaseAttribute):

    def __init__(self, value):
        super().__init__(value)
        self.range = EGO_ACCELERATION_RANGE


class EgoDistance(BaseAttribute):

    def __init__(self, value):
        super().__init__(value)
        self.range = EGO_SPEED_RANGE


class OtherAcceleration(BaseAttribute):
    def __init__(self, value, max_value, min_value):
        super().__init__(value)
        self.range = OTHER_ACCELERATION_RANGE


class OtherVelocity(BaseAttribute):
    def __init__(self, value, max_value, min_value):
        super().__init__(value)
        self.range = OTHER_VELOCITY_RANGE


class ConnectionStrength(BaseAttribute):
    def __init__(self, value, max_value, min_value):
        super().__init__(value)
        self.range = CONNECTION_STRENGTH_RANGE

class Weather(Enum):
    SUNSHINE = 1
    RAIN = 2
    FOG = 3
