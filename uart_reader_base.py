import socket
import struct
import threading
from typing import Optional, Sequence


class UARTReaderBase:
    def __init__(self, reader_name: str):
        self._reader_name = reader_name
        self._running = threading.Event()
        self._loop_thread: Optional[threading.Thread] = None

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    @staticmethod
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

    @staticmethod
    def _close_socket(sock: Optional[socket.socket]) -> None:
        if sock is None:
            return
        try:
            sock.close()
        except OSError:
            pass

    @staticmethod
    def _pack_floats(values: Sequence[float]) -> bytes:
        return struct.pack("f" * len(values), *values)

    def _on_start(self) -> None:
        return

    def _on_stop(self) -> None:
        return

    def _loop_once(self) -> None:
        raise NotImplementedError

    def _run_loop(self) -> None:
        while self._running.is_set():
            self._loop_once()

    def start(self) -> None:
        if self._loop_thread is not None and self._loop_thread.is_alive():
            return
        self._running.set()
        try:
            self._on_start()
        except Exception:
            self._running.clear()
            raise
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()
        print(f"{self._reader_name} started")

    def stop(self) -> None:
        if not self._running.is_set() and (
            self._loop_thread is None or not self._loop_thread.is_alive()
        ):
            return
        self._running.clear()
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=2.0)
            self._loop_thread = None
        self._on_stop()
