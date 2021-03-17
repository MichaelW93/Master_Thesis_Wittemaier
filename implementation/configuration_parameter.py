""" This file contains all configuration parameter for the simulation"""

CARLA_SERVER_PORT: int = 2000
CARLA_SERVER_HOST: str = "localhost"
CARLA_SERVER_FPS: int = 20
TRAFFIC_MANAGER_PORT: int = 8000

SCENE_CAMERA_LOCATION_X: float = 0.0
SCENE_CAMERA_LOCATION_Y: float = 0.0
SCENE_CAMERA_LOCATION_Z: float = 20.0

SCENE_CAMERA_ROTATION_PITCH: float = -45.0
SCENE_CAMERA_ROTATION_YAW: float = 0.0
SCENE_CAMERA_ROTATION_ROLL: float = 0.0

LEADER_SPAWN_LOCATION_X: float = 40.0
LEADER_SPAWN_LOCATION_Y: float = -190.0
LEADER_SPAWN_LOCATION_Z: float = 1
LEADER_SPAWN_ROTATION_YAW: float = 0.0

EGO_SPAWN_LOCATION_X: float = 19.0
EGO_SPAWN_LOCATION_Y: float = -190.0
EGO_SPAWN_LOCATION_Z: float = 1.0
EGO_SPAWN_ROTATION_YAW: float = 0.0

PYGAME_WINDOW_WIDTH: float = 1920
PYGAME_WINDOW_HEIGHT: float = 1080
ANTI_ALIASING: bool = True
SHOW_FPS: bool = True

DEBUG_MODE: bool = False
MOVE_SPECTATOR: bool = False