"""Loads Algorithm/config.yaml at runtime, falling back to built-in defaults
for any missing file or key so the app never crashes on a stale/partial config.

Resolves config.yaml relative to the running executable when frozen
(PyInstaller --onefile), or relative to this file when run from source.
"""
import sys
from pathlib import Path

import yaml

from simulator.types import Obstacle

DEFAULTS: dict = {
    'rpi': {
        'host': '192.168.3.3',
        'port': 5000,
        'timeout_s': 60.0,
    },
    'simulator': {
        'cell_cm': 10,
        'grid_size': 20,
        'cell_px': 40,
        'arena_px': 800,
        'fps': 60,
        'turn_radius_cm': 25.0,
        'robot_w_cm': 30,
        'robot_h_cm': 30,
        'step_cm_per_frame': 2.0,
        'deg_per_frame': 3.0,
        'approach_cm': 20,
        'start_x_cm': 20.0,
        'start_y_cm': 30.0,
        'start_theta': 90.0,
    },
    'default_obstacles': [
        {'x': 50,  'y': 100, 'face': 'N'},
        {'x': 110, 'y': 100, 'face': 'E'},
        {'x': 50,  'y': 160, 'face': 'S'},
        {'x': 110, 'y': 160, 'face': 'W'},
        {'x': 170, 'y': 60,  'face': 'N'},
    ],
}


def config_dir() -> Path:
    """Directory to look for config.yaml in: the exe's own folder when frozen, else this file's folder."""
    if getattr(sys, 'frozen', False):
        return Path(sys.executable).parent
    return Path(__file__).parent


def _deep_merge(defaults: dict, overrides: dict) -> dict:
    """Merge `overrides` onto `defaults`, recursing into nested dicts.
    Non-dict values (including lists) are replaced wholesale when present in overrides.
    If a section is a dict in `defaults` but overridden with a non-dict scalar, the
    override is rejected (with a warning) and the default section is kept, so callers
    downstream can always rely on the shape of nested sections matching DEFAULTS."""
    merged = {}
    for key, default_val in defaults.items():
        override_val = overrides.get(key)
        if isinstance(default_val, dict) and isinstance(override_val, dict):
            merged[key] = _deep_merge(default_val, override_val)
        elif isinstance(default_val, dict) and key in overrides:
            print(f"[app_config] '{key}' override is not a mapping — keeping built-in defaults for that section")
            merged[key] = default_val
        elif key in overrides:
            merged[key] = override_val
        else:
            merged[key] = default_val
    return merged


def _valid_default_obstacles(value) -> bool:
    """True if `value` is a list of dicts each containing 'x', 'y', and 'face'."""
    if not isinstance(value, list):
        return False
    return all(
        isinstance(entry, dict) and {'x', 'y', 'face'} <= entry.keys()
        for entry in value
    )


def load_config(path: Path) -> dict:
    """Load and merge config.yaml at `path` onto DEFAULTS.

    Missing file, unreadable file, or malformed YAML all fall back to
    DEFAULTS (with a console warning) rather than raising.
    """
    if not path.exists():
        print(f"[app_config] {path} not found — using built-in defaults")
        return DEFAULTS
    try:
        with open(path, 'r') as f:
            raw = yaml.safe_load(f) or {}
    except (yaml.YAMLError, OSError) as exc:
        print(f"[app_config] failed to read/parse {path}: {exc} — using built-in defaults")
        return DEFAULTS

    if not isinstance(raw, dict):
        print(f"[app_config] {path} does not contain a mapping at the top level — using built-in defaults")
        return DEFAULTS

    merged = _deep_merge(DEFAULTS, raw)

    if not _valid_default_obstacles(merged.get('default_obstacles')):
        print(f"[app_config] 'default_obstacles' in {path} is malformed — using built-in defaults for that section")
        merged['default_obstacles'] = DEFAULTS['default_obstacles']

    return merged


_cfg = load_config(config_dir() / 'config.yaml')

RPI_HOST = _cfg['rpi']['host']
RPI_PORT = _cfg['rpi']['port']
RPI_TIMEOUT_S = _cfg['rpi']['timeout_s']

CELL_CM = _cfg['simulator']['cell_cm']
GRID_SIZE = _cfg['simulator']['grid_size']
CELL_PX = _cfg['simulator']['cell_px']
ARENA_PX = _cfg['simulator']['arena_px']
FPS = _cfg['simulator']['fps']
TURN_RADIUS_CM = _cfg['simulator']['turn_radius_cm']
ROBOT_W_CM = _cfg['simulator']['robot_w_cm']
ROBOT_H_CM = _cfg['simulator']['robot_h_cm']
STEP_CM_PER_FRAME = _cfg['simulator']['step_cm_per_frame']
DEG_PER_FRAME = _cfg['simulator']['deg_per_frame']
APPROACH_CM = _cfg['simulator']['approach_cm']
START_X_CM = _cfg['simulator']['start_x_cm']
START_Y_CM = _cfg['simulator']['start_y_cm']
START_THETA = _cfg['simulator']['start_theta']

DEFAULT_OBSTACLES = [Obstacle(x=o['x'], y=o['y'], face=o['face']) for o in _cfg['default_obstacles']]
