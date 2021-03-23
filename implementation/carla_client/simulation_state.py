from implementation.knowledge.base_attribute import Weather


class SimulationState(object):

    def __init__(self):
        self.ego_speed_available: bool = True
        self.ego_acceleration_available: bool = True
        self.ego_distance_available: bool = True

        self.other_speed_available: bool = True
        self.other_acceleration_available: bool = True
        self.other_emergency_brake_available: bool = True
        self.other_braking_light_available: bool = True
        self.other_perform_emergency_brake: bool = True

        self.leader_speed: float = 60.0

        self.speed_limit: float = 60.0
        self.connection_strength: float = 100.0
        self.weather: Weather = Weather(1)
