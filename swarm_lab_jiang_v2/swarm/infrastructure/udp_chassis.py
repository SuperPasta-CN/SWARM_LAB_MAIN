"""UDP transport for four-wheel ground-vehicle commands.

Unlike the legacy driver, the datagram socket is persistent (created once
per chassis and reused) to avoid per-cycle socket setup cost; a lock
serializes sends.  The wire format is unchanged: ``<FL,FR,RL,RR>`` with
``int(round())`` values, for firmware compatibility.
"""

from __future__ import annotations

import socket
from dataclasses import dataclass
from threading import Lock
from typing import Mapping, Optional


@dataclass(frozen=True)
class SocketConfig:
    port: int = 12345
    timeout_s: float = 0.2
    encoding: str = "utf-8"


class ChassisSocketDriver:
    """Send commands using the original chassis datagram format."""

    def __init__(self, address: str, config: SocketConfig) -> None:
        self.address = address
        self.config = config
        self._lock = Lock()
        self._socket: Optional[socket.socket] = None

    def _ensure_socket(self) -> socket.socket:
        if self._socket is None:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.settimeout(self.config.timeout_s)
        return self._socket

    def encode(self, command: Mapping[str, float]) -> bytes:
        """Encode wheel values with the original rounding and field order."""

        return (
            "<%d,%d,%d,%d>"
            % (
                int(round(command["front_left"])),
                int(round(command["front_right"])),
                int(round(command["rear_left"])),
                int(round(command["rear_right"])),
            )
        ).encode(self.config.encoding)

    def send(self, command: Mapping[str, float]) -> bool:
        payload = self.encode(command)
        with self._lock:
            try:
                sock = self._ensure_socket()
                sock.sendto(payload, (self.address, self.config.port))
            except OSError:
                self._drop_socket()
                return False
        return True

    def _drop_socket(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

    def close(self) -> None:
        with self._lock:
            self._drop_socket()
