from typing import List, Optional, Union, Any

@staticmethod
def initialize_array(value: Optional[Any], list_length: int) -> List[Optional[Any]]:
    """Initializes an array with a specified length and a specified value"""

    list_to_fill = []
    for i in range(list_length):
        list_to_fill[i] = value

    return list_to_fill
