"""
TCP client for sending commands to the relay RPi on the car.

The RPi acts as a WiFi access point. Connect your machine to that network,
then use CarConnection to send commands.

Wire protocol (matches RaspberryPi/Robot/server.py):
  Request  — JSON line:  {"id": <int>, "cmd": "<KIND><3-digit-value>"}
  Response — JSON:       {"id": <int>, "status": 200, "msg": "<KIND><value> OK"}

Command examples:
  FW050  — move forward 50 cm
  BW020  — move backward 20 cm
  RL090  — rotate left 90 degrees
  RR038  — rotate right 38 degrees
WAIT commands are simulator-only and are silently skipped.
"""
import json
import socket
from typing import Callable
from simulator.types import Command

from app_config import RPI_HOST, RPI_PORT, RPI_TIMEOUT_S as _TIMEOUT_S


def serialize(cmd: Command) -> str | None:
    """Return the bare command string (e.g. 'FW050'), or None for WAIT."""
    if cmd.kind == 'WAIT':
        return None
    return f"{cmd.kind}{round(cmd.value):03d}"


class CarConnection:
    """Context-manager TCP connection to the RPi.

    Usage:
        with CarConnection() as car:
            car.send_commands(cmds)
    """

    def __init__(self, host: str = RPI_HOST, port: int = RPI_PORT):
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._seq = 0

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(_TIMEOUT_S)
        self._sock.connect((self._host, self._port))
        print(f"[comms] connected to {self._host}:{self._port}")

    def disconnect(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
            print("[comms] disconnected")

    def send_command(self, cmd: Command) -> None:
        """Send one command and block until the RPi responds with status 200."""
        wire = serialize(cmd)
        if wire is None:
            return
        self._seq += 1
        payload = json.dumps({"id": self._seq, "cmd": wire}) + "\n"
        self._sock.sendall(payload.encode())
        print(f"[comms] -> {wire}", end='  ', flush=True)

        # RPi sends a JSON response without a trailing newline; use recv() directly.
        raw = self._sock.recv(4096).decode().strip()
        try:
            resp = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Bad response from RPi: {raw!r}") from exc

        status = resp.get("status")
        print(f"<- {status} {resp.get('msg', '')}")
        if status != 200:
            raise RuntimeError(f"RPi error {status}: {resp.get('msg')}")

    def send_commands(
        self,
        cmds: list[Command],
        on_progress: "Callable[[int, int, str], None] | None" = None,
        on_obstacle_reached: "Callable[[str], None] | None" = None,
    ) -> None:
        """Send a sequence of commands, waiting for status 200 after each one.

        If on_progress is given, it's called as on_progress(sent, total, wire)
        after each successful command acknowledgment.

        WAIT commands are simulator-only and are never sent to the car, but a
        WAIT carrying an obstacle_id marks "the car just finished driving to
        this obstacle" — if on_obstacle_reached is given, it's called with
        that obstacle_id at exactly that point in the sequence.
        """
        total = sum(1 for c in cmds if c.kind != 'WAIT')
        sent = 0
        for cmd in cmds:
            if cmd.kind == 'WAIT':
                if on_obstacle_reached and cmd.obstacle_id:
                    on_obstacle_reached(cmd.obstacle_id)
                continue
            sent += 1
            wire = serialize(cmd)
            print(f"[comms] ({sent}/{total})", end=' ')
            self.send_command(cmd)
            if on_progress:
                on_progress(sent, total, wire)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()
