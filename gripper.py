#
# gripper.py
# Bart Trzynadlowski, 2024
#
# Standalone UMI gripper tester.
#
# Usage Instructions
# ------------------
# Run the tester with a serial port:
#
#   python gripper.py --port=COM5
#

import argparse
import time

from numpy import deg2rad

from robot.motors import (
    DynamixelModel, DynamixelMotor, DynamixelBus,
    ControlMode,
    ProfileVelocity, GoalPosition, PresentPosition, TorqueEnable, OperatingMode
)
from robot.util import serial_ports, find_serial_port


####################################################################################################
# Main Loop
####################################################################################################

def set_position(bus: DynamixelBus, degrees: float):
    print(f"Setting position: {degrees} deg")
    present_position: PresentPosition = bus.read(motor="gripper", attribute=PresentPosition)
    print(f"  Before: {present_position.degrees} deg")
    bus.write(motor="gripper", value=GoalPosition(degrees=degrees))
    time.sleep(1)
    present_position: PresentPosition = bus.read(motor="gripper", attribute=PresentPosition)
    print(f"  After : {present_position.degrees} deg")

def main(serial_port: str, motor_id: int):
    bus = DynamixelBus(
        port=serial_port,
        baud_rate=57600,
        motors=[
            DynamixelMotor(id=motor_id, model=DynamixelModel.XM430_W350_R, name="gripper", lower_limit_radians=deg2rad(0), upper_limit_radians=deg2rad(75))
        ]
    )

    try:
        # Limit speed
        bus.write(motor="gripper", value=ProfileVelocity(rev_per_min=0.01))

        # Read initial position
        present_position: PresentPosition = bus.read(motor="gripper", attribute=PresentPosition)
        print(f"Initial position: {present_position.degrees} deg")
        time.sleep(1)

        # Open and close repeatedly
        bus.write(motor="gripper", value=OperatingMode(mode=ControlMode.POSITION_CONTROL))
        bus.write(motor="gripper", value=TorqueEnable(enabled=True))
        angle_sequence = [ 0, 40, 0, 40, 60, 70, 72, 0, 72, 0, 72 ]
        while True:
            for degrees in angle_sequence:
                set_position(bus=bus, degrees=degrees)
                time.sleep(1)

    finally:
        # Torque off
        bus.write(motor="gripper", value=TorqueEnable(enabled=False))
        bus.write(motor="gripper", value=ProfileVelocity(rev_per_min=2.0))


####################################################################################################
# Program Entry Point and Command Line Processing
####################################################################################################

def get_serial_port() -> str:
    ports = serial_ports()
    if options.list_ports:
        if len(ports) == 0:
            print("No serial ports.")
        else:
            print("\n".join(ports))
        exit()
    port = ports[0] if options.port is None else find_serial_port(port_pattern=options.port)
    if port is None:
        print("Error: No serial ports.")
        exit()
    print(f"Serial port: {port}")
    return port

if __name__ == "__main__":
    parser = argparse.ArgumentParser("gripper")
    parser.add_argument("--list-ports", action="store_true", help="List available serial ports and exit")
    parser.add_argument("--port", metavar="name", action="store", type=str, help="Serial port to use to connect to gripper")
    parser.add_argument("--motor-id", metavar="id", action="store", type=int, default=9, help="Dynamixel motor ID")
    options = parser.parse_args()

    serial_port = get_serial_port()
    main(serial_port=serial_port, motor_id=options.motor_id)