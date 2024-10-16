#
# robot/crane_x7.py
# Bart Trzynadlowski, 2024
#
# CRANE-X7 robot.
#

from enum import StrEnum
import timeit
from typing import Dict

from .motors import DynamixelModel, DynamixelMotor, DynamixelBus, degrees_to_delta_motor_steps
from .motors.control_table import *
from .util import URDF


class CraneX7Joint(StrEnum):
    SHOULDER_PAN = "crane_x7_shoulder_fixed_part_pan_joint"
    SHOULDER_TILT = "crane_x7_shoulder_revolute_part_tilt_joint"
    UPPER_TWIST = "crane_x7_upper_arm_revolute_part_twist_joint"
    UPPER_ROTATE = "crane_x7_upper_arm_revolute_part_rotate_joint"
    LOWER_FIXED = "crane_x7_lower_arm_fixed_part_joint"
    LOWER_REVOLUTE = "crane_x7_lower_arm_revolute_part_joint"
    WRIST = "crane_x7_wrist_joint"
    GRIPPER = "crane_x7_gripper_finger_a_joint"

class CraneX7Robot:
    _bus: DynamixelBus

    def __init__(self, serial_port: str, urdf_filepath: str):
        urdf = URDF(filepath=urdf_filepath)
        shoulder_pan_joint = urdf.joints_by_name[CraneX7Joint.SHOULDER_PAN.value]
        shoulder_tilt_joint = urdf.joints_by_name[CraneX7Joint.SHOULDER_TILT.value]
        upper_twist_joint = urdf.joints_by_name[CraneX7Joint.UPPER_TWIST.value]
        upper_rotate_joint = urdf.joints_by_name[CraneX7Joint.UPPER_ROTATE.value]
        lower_fixed_joint = urdf.joints_by_name[CraneX7Joint.LOWER_FIXED.value]
        lower_revolute_joint = urdf.joints_by_name[CraneX7Joint.LOWER_REVOLUTE.value]
        wrist_joint = urdf.joints_by_name[CraneX7Joint.WRIST.value]
        gripper_joint = urdf.joints_by_name[CraneX7Joint.GRIPPER.value]

        self._bus = DynamixelBus(
            port=serial_port,
            baud_rate=57600,
            motors=[
                DynamixelMotor(id=2, model=DynamixelModel.XM430_W350_R, name=CraneX7Joint.SHOULDER_PAN.value, lower_limit_radians=shoulder_pan_joint.lower_limit, upper_limit_radians=shoulder_pan_joint.upper_limit),
                DynamixelMotor(id=3, model=DynamixelModel.XM540_W270_R, name=CraneX7Joint.SHOULDER_TILT.value, lower_limit_radians=shoulder_tilt_joint.lower_limit, upper_limit_radians=shoulder_tilt_joint.upper_limit),
                DynamixelMotor(id=4, model=DynamixelModel.XM430_W350_R, name=CraneX7Joint.UPPER_TWIST.value, lower_limit_radians=upper_twist_joint.lower_limit, upper_limit_radians=upper_twist_joint.upper_limit),
                DynamixelMotor(id=5, model=DynamixelModel.XM430_W350_R, name=CraneX7Joint.UPPER_ROTATE.value, lower_limit_radians=upper_rotate_joint.lower_limit, upper_limit_radians=upper_rotate_joint.upper_limit),
                DynamixelMotor(id=6, model=DynamixelModel.XM430_W350_R, name=CraneX7Joint.LOWER_FIXED.value, lower_limit_radians=lower_fixed_joint.lower_limit, upper_limit_radians=lower_fixed_joint.upper_limit),
                DynamixelMotor(id=7, model=DynamixelModel.XM430_W350_R, name=CraneX7Joint.LOWER_REVOLUTE.value, lower_limit_radians=lower_revolute_joint.lower_limit, upper_limit_radians=lower_revolute_joint.upper_limit),
                DynamixelMotor(id=8, model=DynamixelModel.XM430_W350_R, name=CraneX7Joint.WRIST.value, lower_limit_radians=wrist_joint.lower_limit, upper_limit_radians=wrist_joint.upper_limit),
                DynamixelMotor(id=9, model=DynamixelModel.XM430_W350_R, name=CraneX7Joint.GRIPPER.value, lower_limit_radians=gripper_joint.lower_limit, upper_limit_radians=gripper_joint.upper_limit),
            ]
        )

        # Default settings
        self._bus.write_all(value=OperatingMode(mode=ControlMode.POSITION_CONTROL))
        self.speed(rev_per_min=2.0) # very low speed by default

    def torque(self, enabled: bool):
        self._bus.write_all(value=TorqueEnable(enabled=enabled))
        log(f"Torque: {'ON' if enabled else 'OFF'}")

    def speed(self, rev_per_min: float):
        self._bus.write_all(value=ProfileVelocity(rev_per_min=rev_per_min))

    def goal(self, joint: CraneX7Joint, **kwargs):
        goal_position: GoalPosition | None = None
        if "radians" in kwargs:
            goal_position = GoalPosition(radians=kwargs["radians"])
        elif "degrees" in kwargs:
            goal_position = GoalPosition(degrees=kwargs["degrees"])
        if goal_position is not None:
            self._bus.write(motor=joint.value, value=goal_position)

    def goals(self, joint_degrees: Dict[CraneX7Joint, float]):
        for (joint, degrees) in joint_degrees.items():
            self._bus.write(motor=joint.value, value=GoalPosition(degrees=degrees))

    def home_pose(self, timeout_seconds: float | None = 60):
        home_motor_step_by_joint = {
            CraneX7Joint.SHOULDER_PAN: 1032,
            CraneX7Joint.SHOULDER_TILT: 2357,
            CraneX7Joint.UPPER_TWIST: 2048,
            CraneX7Joint.UPPER_ROTATE: 220,
            CraneX7Joint.LOWER_FIXED: 2048,
            CraneX7Joint.LOWER_REVOLUTE: 2145,
            CraneX7Joint.WRIST: 2008,
            CraneX7Joint.GRIPPER: 2318
        }

        # Command robot to return to home pose
        self.torque(enabled=True)
        log("Returning to home pose...")
        for (joint, motor_step) in home_motor_step_by_joint.items():
            self._bus.write(motor=joint.value, value=GoalPosition(value=motor_step))

        # Check to see whether joints have reached their target
        threshold = degrees_to_delta_motor_steps(degrees=2)
        t0 = timeit.default_timer()
        reached_goal = False
        while not reached_goal:
            # Compare against threshold
            reached_goal = True
            for (joint, goal) in home_motor_step_by_joint.items():
                present_position = self._bus.read(motor=joint.value, attribute=PresentPosition)
                motor_at_goal = abs(present_position.value - goal) <= threshold
                reached_goal = reached_goal and motor_at_goal
            if reached_goal:
                log("Home pose reached")

            # Check timeout, if requested
            if timeout_seconds is not None:
                elapsed = timeit.default_timer() - t0
                if elapsed >= timeout_seconds:
                    log("Aborting homing procedure due to timeout")
                    reached_goal = True


def log(message: str):
    print(f"[CRANE-X7] {message}")