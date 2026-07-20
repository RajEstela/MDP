import json

from comms import CarConnection, serialize
from simulator.types import Command


class _FakeSocket:
    def __init__(self, responses: list[bytes]):
        self._responses = list(responses)
        self.sent: list[bytes] = []

    def sendall(self, data: bytes) -> None:
        self.sent.append(data)

    def recv(self, bufsize: int) -> bytes:
        return self._responses.pop(0)


def _ok_response() -> bytes:
    return json.dumps({"id": 1, "status": 200, "msg": "OK"}).encode()


def _make_connection(n_responses: int) -> CarConnection:
    conn = CarConnection.__new__(CarConnection)
    conn._host = "test-host"
    conn._port = 0
    conn._seq = 0
    conn._sock = _FakeSocket([_ok_response() for _ in range(n_responses)])
    return conn


def test_serialize_formats_command():
    assert serialize(Command("FW", 50.0)) == "FW050"


def test_serialize_wait_returns_none():
    assert serialize(Command("WAIT", 300.0)) is None


def test_send_commands_calls_on_progress_for_movement_commands_only():
    conn = _make_connection(n_responses=2)
    calls = []
    conn.send_commands(
        [Command("FW", 50.0), Command("WAIT", 300.0), Command("RL", 90.0)],
        on_progress=lambda sent, total, wire: calls.append((sent, total, wire)),
    )
    assert calls == [(1, 2, "FW050"), (2, 2, "RL090")]


def test_send_commands_without_on_progress_still_sends_all():
    conn = _make_connection(n_responses=1)
    conn.send_commands([Command("BW", 20.0)])
    assert len(conn._sock.sent) == 1
