import carla
import math
from typing import List, Optional, Any, TYPE_CHECKING


def initialize_list(value: Optional[Any], list_length: int) -> List[Optional[Any]]:
    """Initializes an array with a specified length and a specified value"""

    list_to_fill = []
    for i in range(list_length):
        list_to_fill.append(value)

    return list_to_fill


def initialize_list_of_lists(value: Optional[Any], list_length: int, number_of_lists: int) -> List[List[Optional[Any]]]:
    list_to_fill = []
    for i in range(number_of_lists):
        list_to_fill.append(initialize_list(value, list_length))
    return list_to_fill


def velocity_to_speed(velocity: carla.Vector3D) -> float:
    return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
