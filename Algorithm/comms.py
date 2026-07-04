"""
TCP client for sending commands to the relay RPi on the car.

The RPi acts as a WiFi access point. Connect your machine to that network,
then use CarConnection to send commands. Each command is sent as an ASCII
string (e.g. "FW050\n") and the RPi replies with "ACK\n" when done.

Command wire format:  KIND + 3-digit zero-padded integer value
  FW050  — move forward 50 cm
  BW020  — move backward 20 cm
  RL090  — rotate left 90 degrees
  RR038  — rotate right 38 degrees
WAIT commands are simulator-only and are silently skipped.
"""
import socket
from simulator.types import Command

RPI_HOST = '192.168.1.15'
RPI_PORT = 5000
_TIMEOUT_S = 30.0  # max seconds to wait for ACK per command


def serialize(cmd: Command) -> str | None:
    """Return wire-format string for cmd, or None if cmd should not be sent."""
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
        self._reader = None

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(_TIMEOUT_S)
        self._sock.connect((self._host, self._port))
        self._reader = self._sock.makefile('r')
        print(f"[comms] connected to {self._host}:{self._port}")

    def disconnect(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
            self._reader = None
            print("[comms] disconnected")

    def send_command(self, cmd: Command) -> None:
        """Send one command and block until ACK is received."""
        wire = serialize(cmd)
        if wire is None:
            return
        self._sock.sendall((wire + '\n').encode())
        print(f"[comms] → {wire}", end='  ', flush=True)
        ack = self._reader.readline().strip()
        print(f"← {ack}")
        if ack != 'ACK':
            raise RuntimeError(f"Expected ACK from RPi, got {ack!r}")

    def send_commands(self, cmds: list[Command]) -> None:
        """Send a sequence of commands, waiting for ACK after each one."""
        total = sum(1 for c in cmds if c.kind != 'WAIT')
        sent = 0
        for cmd in cmds:
            if cmd.kind == 'WAIT':
                continue
            sent += 1
            print(f"[comms] ({sent}/{total})", end=' ')
            self.send_command(cmd)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()
