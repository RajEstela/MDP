from unittest.mock import Mock

import live_arena
from simulator.types import Command, RobotState


def test_process_snapshot_passes_robot_start_to_get_commands(monkeypatch):
    captured = {}

    def fake_get_commands(obstacles, start=None):
        captured["obstacles"] = obstacles
        captured["start"] = start
        return [Command("FW", 10.0)]

    monkeypatch.setattr(live_arena, "get_commands", fake_get_commands)

    snapshot = {
        "version": 1, "type": "arena", "revision": 3,
        "grid": {"columns": 20, "rows": 20, "cellCm": 10, "origin": "bottom-left"},
        "robot": {"x": 1, "y": 1, "direction": "N"},
        "obstacles": [],
    }
    sock = Mock()
    live_arena.process_snapshot(sock, snapshot, host="1.2.3.4", execute=False)

    assert captured["start"] == RobotState(x=25.0, y=25.0, theta=90.0)


def test_process_snapshot_sends_route_ready_status_without_execute(monkeypatch):
    monkeypatch.setattr(live_arena, "get_commands", lambda obstacles, start=None: [Command("FW", 10.0)])
    snapshot = {
        "version": 1, "type": "arena", "revision": 3,
        "grid": {"columns": 20, "rows": 20, "cellCm": 10, "origin": "bottom-left"},
        "robot": {"x": 1, "y": 1, "direction": "N"},
        "obstacles": [],
    }
    sock = Mock()
    live_arena.process_snapshot(sock, snapshot, host="1.2.3.4", execute=False)

    sent_payloads = [call.args[0] for call in sock.sendall.call_args_list]
    states = [c.decode() for c in sent_payloads]
    assert any('"state":"route_ready"' in s for s in states)
    assert not any('"state":"running"' in s for s in states)


def test_process_snapshot_sends_obstacle_reached_status_when_executing(monkeypatch):
    commands = [Command("FW", 10.0), Command("WAIT", 300.0, obstacle_id="B1")]
    monkeypatch.setattr(live_arena, "get_commands", lambda obstacles, start=None: commands)

    sent_scan_commands = []

    class _FakeCarConnection:
        def __init__(self, host=None):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def send_command(self, cmd):
            sent_scan_commands.append(cmd)

        def send_commands(self, cmds, on_progress=None, on_obstacle_reached=None):
            for cmd in cmds:
                if cmd.kind == "WAIT" and cmd.obstacle_id and on_obstacle_reached:
                    on_obstacle_reached(cmd.obstacle_id)

    monkeypatch.setattr(live_arena, "CarConnection", _FakeCarConnection)

    snapshot = {
        "version": 1, "type": "arena", "revision": 3,
        "grid": {"columns": 20, "rows": 20, "cellCm": 10, "origin": "bottom-left"},
        "robot": {"x": 1, "y": 1, "direction": "N"},
        "obstacles": [],
    }
    sock = Mock()
    live_arena.process_snapshot(sock, snapshot, host="1.2.3.4", execute=True)

    sent_payloads = [call.args[0] for call in sock.sendall.call_args_list]
    states = [c.decode() for c in sent_payloads]
    assert any('"state":"obstacle_reached"' in s and '"obstacleId":"B1"' in s for s in states)

    assert len(sent_scan_commands) == 1
    assert sent_scan_commands[0].kind == "SCAN"
    assert sent_scan_commands[0].obstacle_id == "B1"
