#
# robot/motors/control_table.py
# Bart Trzynadlowski, 2024
#
# Control table definition. Defines the various data items that can be accessed via the Dynamixel
# bus.
#

from abc import abstractmethod
from enum import IntEnum

from .units import degrees_to_motor_step, radians_to_motor_step, motor_step_to_degrees, motor_step_to_radians


class ControlAttribute:
    address: int
    num_bytes: int
    read: bool
    write: bool

    @property
    @abstractmethod
    def value(self) -> int:
        pass

    @abstractmethod
    def __init__(self, value: int | None = None, **kwargs):
        pass

class ControlMode(IntEnum):
    CURRENT_CONTROL = 0
    VELOCITY_CONTROL = 1
    POSITION_CONTROL = 3
    EXTENDED_POSITION_CONTROL = 4
    CURRENT_BASED_POSITION_CONTROL = 5
    PWM_CONTROL = 16

class OperatingMode(ControlAttribute):
    address = 11
    num_bytes = 1
    read = True
    write = True

    def __init__(self, value: int | None = None, **kwargs):
        if value is None:
            mode = kwargs["mode"]
            if not isinstance(mode, ControlMode):
                raise TypeError(f"mode must be of type ControlMode")
            self._value = mode.value
        else:
            if not value in ControlMode.__members__.values():
                raise ValueError(f"Invalid operating mode ({value}). Must be one of options in ControlMode enum.")
            self._value = value

    @property
    def value(self) -> int:
        return self._value

    @property
    def mode(self) -> ControlMode:
        return ControlMode(self._value)

class TorqueEnable(ControlAttribute):
    address = 64
    num_bytes = 1
    read = True
    write = True

    def __init__(self, value: int | None = None, **kwargs):
        if value is None:
            self._value = 1 if kwargs["enabled"] else 0
        else:
            self._value = value

    @property
    def value(self) -> int:
        return self._value

    @value.setter
    def value(self, value: int):
        self._value = value

    def __str__(self) -> str:
        return f"[TorqueEnable: {'ON' if self.value else 'OFF'}]"

class ProfileVelocity(ControlAttribute):
    address = 112
    num_bytes = 4
    read = True
    write = True

    def __init__(self, value: int | None = None, **kwargs):
        if value is None:
            rev_per_min = kwargs["rev_per_min"]
            if rev_per_min < 0:
                raise ValueError("rev_per_min must be positive")
            self._value = min(32767, round(rev_per_min / 0.229))
        else:
            self._value = value

    @property
    def value(self) -> int:
        return self._value

    @property
    def rev_per_min(self) -> float:
        return self._value * 0.229

    def __str__(self) -> str:
        return f"[ProfileVelocity: {self.value}, {self.rev_per_min} rev/min]"

class GoalPosition(ControlAttribute):
    address = 116
    num_bytes = 4
    read = True
    write = True

    def __init__(self, value: int | None = None, **kwargs):
        if value is None:
            if len(kwargs.keys()) != 1 or list(kwargs.keys())[0] not in [ "degrees", "radians" ]:
                raise ValueError("GoalPosition requires one named argument: degrees or radians")
            if "degrees" in kwargs:
                self._value = degrees_to_motor_step(degrees=kwargs["degrees"])
            else:
                self._value = radians_to_motor_step(radians=kwargs["radians"])
        else:
            self._value = max(0, min(4096, value))

    @property
    def value(self) -> int:
        return self._value

    @property
    def degrees(self) -> float:
        return motor_step_to_degrees(step=self._value)

    @property
    def radians(self) -> float:
        return motor_step_to_radians(step=self._value)

    def __str__(self) -> str:
        return f"[GoalPosition: {self.value}, {self.degrees} deg]"

class PresentPosition(ControlAttribute):
    address = 132
    num_bytes = 4
    read = True
    write = False

    def __init__(self, value: int):
        self._value = value

    @property
    def value(self) -> int:
        return self._value

    @property
    def degrees(self) -> float:
        return motor_step_to_degrees(step=self._value)

    @property
    def radians(self) -> float:
        return motor_step_to_radians(step=self._value)

    def __str__(self) -> str:
        return f"[PresentPosition: {self.value}, {self.degrees} deg]"

class PresentTemperature(ControlAttribute):
    address = 146
    num_bytes = 1
    read = True
    write = False

    def __init__(self, value: int):
        self._value = value

    @property
    def value(self) -> int:
        return self._value

    @property
    def celsius(self) -> float:
        return float(self._value)

    @property
    def fahrenheit(self) -> float:
        return self.celsius * (9.0 / 5.0) + 32.0

    def __str__(self) -> str:
        return f"[PresentTemperature: {self.celsius:.1f} C, {self.fahrenheit:.1f} F]"