import argparse
import time
from typing import Optional, Sequence

from dynamixel_sdk import COMM_SUCCESS, GroupBulkRead, PacketHandler, PortHandler

from uart_reader_base import UARTReaderBase
from udp_receiver import udp_ip, udp_port_servo

ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
BAUDRATE = 3000000
PROTOCOL_VERSION = 2.0
DXL_IDS = [0, 1, 2, 3, 4]
DEFAULT_POS_SCALE = 360.0 / 4096.0


def _parse_cli_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read servo positions and publish UDP data.")
    parser.add_argument(
        "--uart-port",
        default="/dev/tty.usbserial-FTAA0A4I",
        help="UART device path used to read servo data. Use ls /dev to find the correct port. (default: %(default)s)",
    )
    return parser.parse_args(argv)


class ServoReader(UARTReaderBase):
    def __init__(self, uart_device: str):
        super().__init__("Servo reader")
        if not uart_device:
            raise ValueError("uart_device must be a non-empty device path.")

        self.uart_device = uart_device
        self.port_handler = PortHandler(self.uart_device)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)
        self.udp_socket: Optional[object] = self._create_udp_socket(timeout_s=None)

        if not self.port_handler.openPort():
            raise RuntimeError("Failed to open the servo port.")
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.port_handler.closePort()
            raise RuntimeError(f"Failed to change the servo baudrate to {BAUDRATE}.")

    def _on_start(self) -> None:
        self.group_bulk_read.clearParam()
        for dxl_id in DXL_IDS:
            if not self.group_bulk_read.addParam(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                raise RuntimeError(f"[ID:{dxl_id:03d}] groupBulkRead addparam failed")

    def _loop_once(self) -> None:
        dxl_comm_result = self.group_bulk_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))
            return

        servo_joint_angles = [180.0] * len(DXL_IDS)
        for dxl_id in DXL_IDS:
            if not self.group_bulk_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                print(f"[ID:{dxl_id:03d}] groupBulkRead getdata failed")
                continue
            dxl_present_position = self.group_bulk_read.getData(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            servo_joint_angles[dxl_id] = dxl_present_position * DEFAULT_POS_SCALE

        print(f"Servo joint angles: {servo_joint_angles}")
        if self.udp_socket is not None:
            self.udp_socket.sendto(self._pack_floats(servo_joint_angles), (udp_ip, udp_port_servo))

    def _on_stop(self) -> None:
        self.group_bulk_read.clearParam()
        self.port_handler.closePort()
        self._close_socket(self.udp_socket)
        self.udp_socket = None


if __name__ == "__main__":
    args = _parse_cli_args()
    reader = ServoReader(uart_device=args.uart_port)
    try:
        print(f"Opening UART port {reader.uart_device} at {BAUDRATE} baudrate...")
        reader.start()
        print("Servo port opened successfully")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        reader.stop()
