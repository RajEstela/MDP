import json
import sys
import types
import unittest


sys.modules["bluetooth"] = types.ModuleType("bluetooth")
nanocar_stub = types.ModuleType("nanocar")
nanocar_stub.NanoCarLink = type("NanoCarLink", (), {})
sys.modules["nanocar"] = nanocar_stub

import server


class ArenaProtocolTest(unittest.TestCase):
    def setUp(self):
        server.latest_arena = None
        server.arena_clients.clear()

    def test_complete_snapshot_is_validated_and_cached(self):
        payload = {
            "version": 1,
            "type": "arena",
            "revision": 7,
            "grid": {
                "columns": 20,
                "rows": 20,
                "cellCm": 10,
                "origin": "bottom-left",
            },
            "robot": {"x": 1, "y": 1, "direction": "N"},
            "obstacles": [
                {
                    "id": "B1",
                    "x": 10,
                    "y": 6,
                    "direction": "E",
                    "targetId": 11,
                }
            ],
        }

        response = json.loads(server.handle_message(json.dumps(payload), source="test"))

        self.assertEqual(200, response["status"])
        self.assertEqual(7, server.latest_arena["revision"])
        self.assertEqual("B1", server.latest_arena["obstacles"][0]["id"])

    def test_incremental_edits_are_cached_without_starting_route(self):
        add_response = json.loads(server.handle_message("ADD,B2,(4,8)", source="test"))
        face_response = json.loads(server.handle_message("FACE,B2,S", source="test"))

        self.assertEqual(200, add_response["status"])
        self.assertEqual(200, face_response["status"])
        self.assertEqual("S", server.latest_arena["obstacles"][0]["direction"])

    def test_out_of_bounds_snapshot_is_rejected(self):
        payload = {
            "type": "arena",
            "grid": {
                "columns": 20,
                "rows": 20,
                "cellCm": 10,
                "origin": "bottom-left",
            },
            "robot": {"x": 1, "y": 1, "direction": "N"},
            "obstacles": [
                {"id": "B1", "x": 20, "y": 6, "direction": "N", "targetId": None}
            ],
        }

        response = json.loads(server.handle_message(json.dumps(payload), source="test"))

        self.assertEqual(400, response["status"])


if __name__ == "__main__":
    unittest.main()
