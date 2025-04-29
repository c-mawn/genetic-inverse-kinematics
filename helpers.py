import math
from ArmSim import ArmViz
from typing import Callable

Configuration = list[int]
Fitness_func = Callable[[Configuration, list[float], list[float], ArmViz|None], float]

def angles(configuration: Configuration, bits_per_theta: int = 16) -> list[float]:
    """
    Converts from a list of integers between 0 and 2^bits_per_theta-1 to a list of angles between 0 and 2*pi.
    
    Args:
        bits_per_theta (int): The number of bits to use to represent each angle

    Returns:
        angles: list of floats for each joint theta in one arm configuration
    """
    return [integer / 2**bits_per_theta * 2 * math.pi for integer in configuration]
