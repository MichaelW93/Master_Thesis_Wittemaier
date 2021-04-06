import carla


class MonitorInputData(object):

    def __init__(self, timestamp: carla.Timestamp):
        self.timestamp = timestamp

        self.connection_strength: float = None
        self.speed_limit: float = None
        self.weather = None

        self.ego_acceleration: carla.IMUMeasurement = None
        self.ego_distance: carla.ObstacleDetectionEvent = None
        self.ego_speed: float = None
        self.other_braking_light: bool = None

        self.other_acceleration: carla.IMUMeasurement = None
        self.other_speed: float = None
        self.other_emergency_brake: bool = None
