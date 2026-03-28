import math
import socket
import struct
import threading
from typing import Optional, Sequence

# Configure UDP settings
udp_ip = "127.0.0.1"  # Localhost IP
udp_port_joint = 5009  # Port for joint data
udp_port_servo = 5010  # Port for servo data
udp_port_lra = 5012  # Port for LRA data


# (source_index, zero_offset_deg, sign)
_JOINT_TO_URDF_TRANSFORMS = (
    (0, 90.0, -1),   # thumb_dip
    (1, 270.0, 1),   # thumb_pip
    (2, 180.0, 1),   # thumb_mcp_s
    (3, 270.0, 1),   # thumb_mcp_r
    (4, 90.0, -1),   # index_dip
    (5, 270.0, 1),   # index_pip
    (6, 180.0, -1),  # index_mcp_s
    (7, 90.0, -1),   # middle_dip
    (8, 270.0, 1),   # middle_pip
    (9, 180.0, -1),  # middle_mcp_s
    (10, 90.0, -1),  # ring_dip
    (11, 270.0, 1),  # ring_pip
    (12, 180.0, -1), # ring_mcp_s
    (13, 90.0, -1),  # little_dip
    (14, 270.0, 1),  # little_pip
    (15, 180.0, -1), # little_mcp_s
)

_SERVO_TO_URDF_TRANSFORMS = (
    (0, 182.2, -1),   # thumb_mcp_b
    (1, 165.2, -1),   # index_mcp_b
    (2, 150.29, -1),  # middle_mcp_b
    (3, 151.44, -1),  # ring_mcp_b
    (4, 145.9, -1),   # little_mcp_b
)


def _to_urdf_radians(raw_angle_deg: float, zero_offset_deg: float, sign: int) -> float:
    return math.radians(sign * (raw_angle_deg - zero_offset_deg))


def _create_udp_socket(
    bind_addr: Optional[tuple] = None, timeout_s: Optional[float] = 0.2
) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    if timeout_s is not None:
        sock.settimeout(timeout_s)
    if bind_addr is not None:
        sock.bind(bind_addr)
    return sock


def _close_socket(sock: Optional[socket.socket]) -> None:
    if sock is None:
        return
    try:
        sock.close()
    except OSError:
        pass


class UDPReceiver:
    def __init__(
        self,
        host: str = udp_ip,
        joints_port: int = udp_port_joint,
        servo_port: int = udp_port_servo,
        length: int = 16,
        socket_timeout_s: float = 0.2,
    ):
        self.most_recent_joints: Optional[Sequence[float]] = None
        self.most_recent_servo: Optional[Sequence[float]] = None
        self.urdf_joints: Optional[tuple] = None
        self.length = length
        self._host = host
        self._joints_port = joints_port
        self._servo_port = servo_port
        self._socket_timeout_s = socket_timeout_s

        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        self.sock1: Optional[socket.socket] = None
        self.sock2: Optional[socket.socket] = None

        self.thread1: Optional[threading.Thread] = None
        self.thread2: Optional[threading.Thread] = None
        self._open_sockets()

    def _open_sockets(self):
        self.sock1 = _create_udp_socket(
            bind_addr=(self._host, self._joints_port), timeout_s=self._socket_timeout_s
        )
        self.sock2 = _create_udp_socket(
            bind_addr=(self._host, self._servo_port), timeout_s=self._socket_timeout_s
        )

    def listen_joint_stream(self):
        print(f"Listening on {self._host}:{self._joints_port}...")
        while self._running.is_set():
            sock = self.sock1
            if sock is None:
                break
            try:
                data, _addr = sock.recvfrom(4 * self.length)
            except socket.timeout:
                continue
            except OSError:
                if self._running.is_set():
                    raise
                break

            if data:
                received_joints = struct.unpack("f" * self.length, data)
                with self._lock:
                    self.most_recent_joints = received_joints

    def listen_servo_stream(self):
        print(f"Listening on {self._host}:{self._servo_port}...")
        while self._running.is_set():
            sock = self.sock2
            if sock is None:
                break
            try:
                data, _addr = sock.recvfrom(4 * 5)
            except socket.timeout:
                continue
            except OSError:
                if self._running.is_set():
                    raise
                break

            if data:
                received_servo = struct.unpack("f" * 5, data)
                with self._lock:
                    self.most_recent_servo = received_servo

    def start(self):
        if self.thread1 is not None and self.thread1.is_alive():
            return
        if self.sock1 is None or self.sock2 is None:
            self._open_sockets()
        self._running.set()
        self.thread1 = threading.Thread(target=self.listen_joint_stream, daemon=True)
        self.thread1.start()
        self.thread2 = threading.Thread(target=self.listen_servo_stream, daemon=True)
        self.thread2.start()

    def stop(self):
        self._running.clear()
        _close_socket(self.sock1)
        _close_socket(self.sock2)
        self.sock1 = None
        self.sock2 = None
        if self.thread1 is not None:
            self.thread1.join(timeout=2.0)
        if self.thread2 is not None:
            self.thread2.join(timeout=2.0)

    def convert_to_urdf_joint(self):
        with self._lock:
            joint_angles = self.most_recent_joints
            servo_angles = self.most_recent_servo

        if joint_angles is None or servo_angles is None:
            return
        if len(joint_angles) != self.length or len(servo_angles) != len(_SERVO_TO_URDF_TRANSFORMS):
            return

        urdf_joint_angles = [0.0] * (len(_JOINT_TO_URDF_TRANSFORMS) + len(_SERVO_TO_URDF_TRANSFORMS))

        for urdf_idx, (source_idx, zero_offset_deg, sign) in enumerate(_JOINT_TO_URDF_TRANSFORMS):
            urdf_joint_angles[urdf_idx] = _to_urdf_radians(
                joint_angles[source_idx], zero_offset_deg, sign
            )

        base_idx = len(_JOINT_TO_URDF_TRANSFORMS)
        for servo_idx, (source_idx, zero_offset_deg, sign) in enumerate(_SERVO_TO_URDF_TRANSFORMS):
            urdf_joint_angles[base_idx + servo_idx] = _to_urdf_radians(
                servo_angles[source_idx], zero_offset_deg, sign
            )

        self.urdf_joints = tuple(urdf_joint_angles)

    def get_most_recent_joints(self):
        self.convert_to_urdf_joint()
        return self.urdf_joints
