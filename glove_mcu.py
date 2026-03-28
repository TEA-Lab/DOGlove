import argparse
import socket
import struct
import threading
import time
from typing import Optional, Sequence

import serial

from uart_reader_base import UARTReaderBase
from udp_receiver import udp_ip, udp_port_joint, udp_port_lra

# Glove MCU UART settings
baud_rate = 921600  # Main board baud rate
header = bytes([0xAA, 0x55, 0x00, 0x00])
data_length = 16  # Number of data values in the block
block_size = 76  # Total size including header, data, and CRC

def _parse_cli_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read glove MCU UART and publish UDP data.")
    parser.add_argument(
        "--uart-port",
        default="/dev/tty.wchusbserial59090454391",
        help="UART device path used to read glove MCU data. Use ls /dev to find the correct port. (default: %(default)s)",
    )
    return parser.parse_args(argv)


class UARTReader(UARTReaderBase):
    def __init__(self, uart_device: str):
        super().__init__("UART reader")
        if not uart_device:
            raise ValueError("uart_device must be a non-empty device path.")

        self.buffer = bytearray()
        self.write_fps = 30
        self.most_recent_kp: Optional[Sequence[float]] = None
        self.uart_device = uart_device

        self.serial_port: Optional[serial.Serial] = None
        self.udp_socket: Optional[socket.socket] = None
        self.sock: Optional[socket.socket] = None
        self.thread_lra: Optional[threading.Thread] = None
        self._open_io()

    def _open_io(self) -> None:
        self.serial_port = serial.Serial(self.uart_device, baud_rate, timeout=1)
        self.udp_socket = self._create_udp_socket(timeout_s=None)
        self.sock = self._create_udp_socket(bind_addr=(udp_ip, udp_port_lra), timeout_s=0.2)

    def _loop_once(self) -> None:
        if self.serial_port is None:
            return
        try:
            data = self.serial_port.read(self.serial_port.in_waiting or 1)
        except serial.SerialException:
            if self.is_running:
                raise
            return

        if data:
            self.buffer.extend(data)
            self.process_buffer()

    def process_buffer(self) -> None:
        while len(self.buffer) >= block_size:
            start_index = self.buffer.find(header)
            if start_index == -1:
                if len(self.buffer) > block_size:
                    self.buffer = self.buffer[-block_size:]
                break

            if start_index == 0:
                next_header_index = self.buffer.find(header, len(header))
                if next_header_index == -1:
                    break
                if (next_header_index - start_index) != block_size:
                    self.buffer = self.buffer[next_header_index:]
                    continue

                block = self.buffer[start_index:next_header_index]
                self.process_block(block)
                self.buffer = self.buffer[next_header_index:]
            else:
                self.buffer = self.buffer[start_index:]

    def process_block(self, block: bytes) -> None:
        data = struct.unpack("<I16iI", block[4:])
        reference_voltage = data[0]
        data_values = data[1:-1]
        crc = data[-1]

        voltages = [(float(val) * 5.0 / 0x7FFFFF) for val in data_values]
        checksum = (crc >> 16) & 0xFFFF
        timestamp = crc & 0xFFFF
        expected_checksum = sum(data[:-1]) & 0xFFFF

        reference_voltage = reference_voltage / 1000000
        joint_angles = [val / reference_voltage * 360 for val in voltages if reference_voltage != 0]

        print(f"Voltages: {[round(val, 2) for val in voltages]}")
        print(f"Joint Angle: {[round(val, 2) for val in joint_angles]}")
        print(f"Timestamp: {int(timestamp)}")
        print(f"Checksum: {(checksum)} (Expected: {(expected_checksum)})")
        print(f"Checksum Valid: {checksum == expected_checksum}")
        print("-" * 40)

        if self.udp_socket is not None:
            self.udp_socket.sendto(self._pack_floats(joint_angles), (udp_ip, udp_port_joint))

    def lra_control(self, channel: int, wave: int, duration: int) -> None:
        if channel not in range(5):
            print("Invalid channel. Please choose a channel between 0-4.")
            return
        if wave not in range(256):
            print("Invalid wave. Please choose a wave between 0-255.")
            return
        if duration not in range(65536):
            print("Invalid duration. Please choose a duration between 0-65535.")
            return

        if self.serial_port is None:
            return

        duration_h = (duration >> 8) & 0xFF
        duration_l = duration & 0xFF
        checksum = (channel + wave + duration_h + duration_l) & 0xFF
        send_data = [0x55, 0xAA, channel, wave, duration_h, duration_l, checksum]
        try:
            self.serial_port.write(bytes(send_data))
        except serial.SerialException:
            if self.is_running:
                raise

    def _listen_lra(self) -> None:
        print(f"Listening on {udp_ip}:{udp_port_lra}...")
        while self.is_running:
            sock = self.sock
            if sock is None:
                break
            try:
                data, _addr = sock.recvfrom(4 * 4)
            except socket.timeout:
                continue
            except OSError:
                if self.is_running:
                    raise
                break

            if not data:
                continue
            received_kp = struct.unpack("f" * 4, data)
            self.most_recent_kp = received_kp

            threshold = 100
            for i, kp in enumerate(received_kp):
                if 10 < kp < threshold:
                    self.lra_control(i, 56, int(1000 / self.write_fps))
                else:
                    self.lra_control(i, 222, 100)
            time.sleep(1 / self.write_fps)

    def _on_start(self) -> None:
        if self.serial_port is None or self.udp_socket is None or self.sock is None:
            self._open_io()
        if self.thread_lra is None or not self.thread_lra.is_alive():
            self.thread_lra = threading.Thread(target=self._listen_lra, daemon=True)
            self.thread_lra.start()

    def _on_stop(self) -> None:
        print("Closing UART port...")
        self._close_socket(self.sock)
        self.sock = None
        self._close_socket(self.udp_socket)
        self.udp_socket = None

        if self.thread_lra is not None:
            self.thread_lra.join(timeout=2.0)
            self.thread_lra = None

        try:
            if self.serial_port is not None:
                self.serial_port.close()
        except serial.SerialException:
            pass
        self.serial_port = None


if __name__ == "__main__":
    args = _parse_cli_args()

    reader = UARTReader(uart_device=args.uart_port)
    try:
        print(f"Opening UART port {reader.uart_device} at {baud_rate} baudrate...")
        reader.start()
        print("UART port opened successfully")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        reader.stop()
