from face_maneuver import build_face_change_commands


def test_build_face_change_commands_default_step():
    assert build_face_change_commands(25) == ["RR090", "FW025", "RL090", "FW025", "RL090"]


def test_build_face_change_commands_custom_step():
    assert build_face_change_commands(30) == ["RR090", "FW030", "RL090", "FW030", "RL090"]


def test_build_face_change_commands_single_digit_step():
    assert build_face_change_commands(5) == ["RR090", "FW005", "RL090", "FW005", "RL090"]
