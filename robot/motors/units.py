#
# robot/motors/units.py
# Bart Trzynadlowski, 2024
#
# Motor unit conversion helper functions.
#
# TODO:
# -----
# - The range is likely to be [0,4095] rather than 4096 (fix in control_table.py, too).
#

from math import pi


def degrees_to_motor_step(degrees: float) -> int:
    """
    Converts absolute motor position in degrees to the raw motor position value.
    """
    degrees = max(-180, min(180, degrees))
    abs_pos = round(abs(degrees) / 180 * 2048)
    return 2048 + (abs_pos if degrees >= 0 else -abs_pos)

def radians_to_motor_step(radians: float) -> int:
    radians = max(-pi, min(pi, radians))
    abs_pos = round(abs(radians) / pi * 2048)
    return 2048 + (abs_pos if radians >= 0 else -abs_pos)

def degrees_to_delta_motor_steps(degrees: float) -> int:
    """
    Converts degrees of change to number of raw motor steps.
    """
    return round((abs(degrees) / 360.0) * 4096)

def motor_step_to_degrees(step: int) -> float:
    step = max(0, min(4096, step))
    return (step - 2048.0) / 2048.0 * 180.0

def motor_step_to_radians(step: int) -> float:
    step = max(0, min(4096, step))
    return (step - 2048.0) / 2048.0 * pi
