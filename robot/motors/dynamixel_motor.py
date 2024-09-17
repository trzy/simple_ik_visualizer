#
# robot/motors/dynamixel_motor.py
# Bart Trzynadlowski, 2024
#
# DynamixelMotor class: defines a single motor on the motor bus.
#

from enum import Enum

from .units import radians_to_motor_step


class DynamixelModel(Enum):
    XM540_W270_R = "XM540-W270-R"
    XM430_W350_R = "XM430-W350-R"

class DynamixelMotor:
    id: int
    model: DynamixelModel
    name: str
    lower_limit_motor_step: int
    upper_limit_motor_step: int

    def __init__(self, id: int, model: DynamixelModel, name: str, lower_limit_radians: float, upper_limit_radians: float):
        assert id >= 0 and id <= 252
        self.id = id
        self.model = model
        self.name = name
        self.lower_limit_motor_step = radians_to_motor_step(radians=lower_limit_radians)
        self.upper_limit_motor_step = radians_to_motor_step(radians=upper_limit_radians)