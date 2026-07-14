from pathlib import Path

from app_config import DEFAULTS, _deep_merge, load_config


def test_deep_merge_overlays_nested_dict():
    defaults = {'rpi': {'host': 'a', 'port': 1}}
    overrides = {'rpi': {'host': 'b'}}
    merged = _deep_merge(defaults, overrides)
    assert merged == {'rpi': {'host': 'b', 'port': 1}}


def test_deep_merge_replaces_list_wholesale():
    defaults = {'default_obstacles': [{'x': 1}]}
    overrides = {'default_obstacles': [{'x': 2}, {'x': 3}]}
    merged = _deep_merge(defaults, overrides)
    assert merged == {'default_obstacles': [{'x': 2}, {'x': 3}]}


def test_load_config_missing_file_returns_defaults(tmp_path):
    result = load_config(tmp_path / 'does_not_exist.yaml')
    assert result == DEFAULTS


def test_load_config_partial_file_falls_back_for_missing_keys(tmp_path):
    config_file = tmp_path / 'config.yaml'
    config_file.write_text('rpi:\n  host: 10.0.0.5\n')
    result = load_config(config_file)
    assert result['rpi']['host'] == '10.0.0.5'
    assert result['rpi']['port'] == DEFAULTS['rpi']['port']
    assert result['simulator'] == DEFAULTS['simulator']


def test_load_config_invalid_yaml_falls_back_to_defaults(tmp_path):
    config_file = tmp_path / 'config.yaml'
    config_file.write_text('rpi: [unclosed\n')
    result = load_config(config_file)
    assert result == DEFAULTS


def test_load_config_overrides_default_obstacles(tmp_path):
    config_file = tmp_path / 'config.yaml'
    config_file.write_text('default_obstacles:\n  - {x: 1, y: 2, face: N}\n')
    result = load_config(config_file)
    assert result['default_obstacles'] == [{'x': 1, 'y': 2, 'face': 'N'}]


def test_load_config_non_dict_yaml_falls_back_to_defaults(tmp_path):
    config_file = tmp_path / 'config.yaml'
    config_file.write_text('- a\n- b\n')
    result = load_config(config_file)
    assert result == DEFAULTS


def test_deep_merge_ignores_type_mismatched_section_override():
    defaults = {'rpi': {'host': 'a'}}
    overrides = {'rpi': 'not-a-dict'}
    merged = _deep_merge(defaults, overrides)
    assert merged == {'rpi': {'host': 'a'}}


def test_load_config_survives_unreadable_file(tmp_path, monkeypatch):
    config_file = tmp_path / 'config.yaml'
    config_file.write_text('rpi:\n  host: 10.0.0.5\n')

    real_open = open

    def fake_open(file, *args, **kwargs):
        if Path(file) == config_file:
            raise PermissionError(13, 'Permission denied', str(config_file))
        return real_open(file, *args, **kwargs)

    monkeypatch.setattr('builtins.open', fake_open)
    result = load_config(config_file)
    assert result == DEFAULTS


def test_load_config_malformed_default_obstacles_falls_back(tmp_path):
    config_file = tmp_path / 'config.yaml'
    config_file.write_text('default_obstacles:\n  - {x: 1, y: 2}\n')
    result = load_config(config_file)
    assert result['default_obstacles'] == DEFAULTS['default_obstacles']
