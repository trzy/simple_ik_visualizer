#
# robot/motors/dynamixel_bus.py
# Bart Trzynadlowski, 2024
#
# DynamixelBus class: handles bidirectional communication with all Dynamixel motors connected to the
# same serial port.
#

from typing import Dict, List, Type

from dynamixel_sdk import *
from dynamixel_sdk.port_handler import *

from .control_table import ControlAttribute
from .dynamixel_motor import DynamixelMotor


class DynamixelBus:
    _motor_by_id: Dict[int, DynamixelMotor]
    _motor_by_name: Dict[str, DynamixelMotor]
    _port_handler: PortHandler
    _packet_handler: PacketHandler
    _bulk_write: GroupBulkWrite

    def __init__(self, port: str, baud_rate: int, motors: List[DynamixelMotor]):
        self._motor_by_id = { motor.id: motor for motor in motors }
        self._motor_by_name = { motor.name: motor for motor in motors }
        self._port_handler = PortHandler(port)
        if not self._port_handler.openPort():
            raise RuntimeError(f"Unable to connect to Dynamixel bus on {port}")
        self._packet_handler = PacketHandler(protocol_version=2.0)
        if not self._port_handler.setBaudRate(baudrate=baud_rate):
            raise RuntimeError(f"Unable to set baud rate of {baud_rate}")
        self._bulk_write = GroupBulkWrite(self._port_handler, self._packet_handler)

    def __del__(self):
        if self._port_handler.is_open:
            self._port_handler.closePort()

    def read(self, motor: DynamixelMotor | int | str, attribute: Type[ControlAttribute]) -> ControlAttribute | None:
        if not attribute.read:
            raise ValueError(f"{attribute.__name__} is not readable")
        if attribute.num_bytes not in [ 1, 2, 4 ]:
            raise ValueError(f"{attribute.__name__} is defined with an invalid size ({attribute.num_bytes}). Must be 1, 2, or 4 bytes.")

        motor = self._get_motor(motor)

        if attribute.num_bytes == 1:
            raw_value, result, error = self._packet_handler.read1ByteTxRx(self._port_handler, motor.id, attribute.address)
        elif attribute.num_bytes == 2:
            raw_value, result, error = self._packet_handler.read2ByteTxRx(self._port_handler, motor.id, attribute.address)
        else:
            raw_value, result, error = self._packet_handler.read4ByteTxRx(self._port_handler, motor.id, attribute.address)

        if result != COMM_SUCCESS or error != 0:
            print(f"Error: Unable to read {attribute.__name__} from {motor.name} (ID={motor.id}): {self._comm_result_to_string(result)}, error={error}")
            return None

        return attribute(value=raw_value)

    def write(self, motor: DynamixelMotor | int | str, value: ControlAttribute) -> bool:
        if not value.write:
            raise ValueError(f"{value.__class__.__name__} is not writeable")
        if value.num_bytes not in [ 1, 2, 4 ]:
            raise ValueError(f"{value.__class__.__name__} is defined with an invalid size ({value.num_bytes}). Must be 1, 2, or 4 bytes.")

        motor = self._get_motor(motor)

        if value.num_bytes == 1:
            result, error = self._packet_handler.write1ByteTxRx(self._port_handler, motor.id, value.address, value.value)
        elif value.num_bytes == 2:
            result, error = self._packet_handler.write2ByteTxRx(self._port_handler, motor.id, value.address, value.value)
        else:
            result, error = self._packet_handler.write4ByteTxRx(self._port_handler, motor.id, value.address, value.value)

        if result != COMM_SUCCESS or error != 0:
            print(f"Error: Unable to write {value.__class__.__name__} to {motor.name} (ID={motor.id}): {self._comm_result_to_string(result)}, error={error}")
            return False

        return True

    def write_all(self, value: ControlAttribute | None = None, values: List[ControlAttribute] | None = None) -> bool:
        if value is not None and values is not None:
            raise ValueError("Cannot supply both \"value\" and \"values\"")
        if value is not None:
            return self._write_all_one_value(value=value)
        else:
            succeeded = True
            for value in values:
                succeeded = succeeded and self._write_all_one_value(value=value)
            return succeeded

    def _write_all_one_value(self, value: ControlAttribute) -> bool:
        if not value.write:
            raise ValueError(f"{value.__class__.__name__} is not writeable")
        if value.num_bytes not in [ 1, 2, 4 ]:
            raise ValueError(f"{value.__class__.__name__} is defined with an invalid size ({value.num_bytes}). Must be 1, 2, or 4 bytes.")

        succeeded = True
        for id in self._motor_by_id.keys():
            if value.num_bytes == 1:
                byte_buffer = [ value.value ]
            elif value.num_bytes == 2:
                byte_buffer [ DXL_LOBYTE(value.value), DXL_HIBYTE(value.value) ]
            else:
                loword = DXL_LOWORD(value.value)
                hiword = DXL_HIWORD(value.value)
                byte_buffer = [ DXL_LOBYTE(loword), DXL_HIBYTE(loword), DXL_LOBYTE(hiword), DXL_HIBYTE(hiword) ]
            succeeded = succeeded and self._bulk_write.addParam(dxl_id=id, start_address=value.address, data_length=value.num_bytes, data=byte_buffer)

        if not succeeded:
            print(f"Error: Unable to attach all values to the group bulk write of {value.__class__.__name__}")

        result = self._bulk_write.txPacket()
        self._bulk_write.clearParam()

        if result != COMM_SUCCESS:
            print(f"Error: Unable to bulk write {value.__class__.__name__}: {self._comm_result_to_string(result)}")
            succeeded = False

        return succeeded

    def _get_motor(self, motor: DynamixelMotor | int | str) -> DynamixelMotor:
        if isinstance(motor, str):
            if motor not in self._motor_by_name:
                raise KeyError("No motor named \"{motor}\"")
            return self._motor_by_name[motor]
        elif isinstance(motor, int):
            if motor not in self._motor_by_id:
                raise KeyError(f"No motor with ID={motor}")
            return self._motor_by_id[motor]
        else:
            assert isinstance(motor, DynamixelMotor)
            return motor

    @staticmethod
    def _comm_result_to_string(result: int) -> str:
        comm_result_to_str = {
            COMM_SUCCESS: "COMM_SUCCESS",
            COMM_PORT_BUSY: "COMM_PORT_BUSY",
            COMM_TX_FAIL: "COMM_TX_FAIL",
            COMM_RX_FAIL: "COMM_RX_FAIL",
            COMM_TX_ERROR: "COMM_TX_ERROR",
            COMM_RX_WAITING: "COMM_RX_WAITING",
            COMM_RX_TIMEOUT: "COMM_RX_TIMEOUT",
            COMM_RX_CORRUPT: "COMM_RX_CORRUPT",
            COMM_NOT_AVAILABLE: "COMM_NOT_AVAILABLE"
        }
        if result not in comm_result_to_str:
            return f"{result}"
        return comm_result_to_str[result]