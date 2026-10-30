from pathlib import Path

import pytest
import yaml

from pm_robot_modules.submodules.configuration_requirements import (
    DEFAULT_CONFIGURATION_FILE_NAME,
    RobotConfigurationState,
    read_configuration,
    compare_configuration_subset,
    merge_configuration_subset,
    resolve_configuration_file,
)


def test_resolve_default_file():
    assert resolve_configuration_file('/tmp/process', '') == Path(
        '/tmp/process', DEFAULT_CONFIGURATION_FILE_NAME
    )


def test_resolve_relative_override():
    assert resolve_configuration_file(
        '/tmp/process', 'special.yaml'
    ) == Path('/tmp/process/special.yaml')


def test_compare_only_checks_required_properties():
    current = {
        'tools': {
            'vacuum': {'active': True, 'tip': 'A'},
            'gripper': {'active': False},
        },
        'ignored': 123,
    }
    required = {
        'tools': {
            'vacuum': {'active': True},
            'gripper': {'active': False},
        }
    }

    assert compare_configuration_subset(current, required) == []


def test_compare_reports_mismatches_and_missing_properties():
    current = {'tools': {'vacuum': {'active': False}}}
    required = {
        'tools': {
            'vacuum': {'active': True},
            'gripper': {'active': False},
        }
    }

    assert compare_configuration_subset(current, required) == [
        'tools.vacuum.active: required True, current False',
        'tools.gripper: property is missing from the current bringup '
        'configuration',
    ]


def test_merge_only_replaces_properties_in_partial_configuration():
    current = {
        'tools': {
            'vacuum': {'active': True, 'tip': 'A'},
            'gripper': {'active': False},
        },
        'dispenser': {'tip': 'unchanged'},
    }

    merged = merge_configuration_subset(
        current,
        {'tools': {'vacuum': {'active': False}}},
    )

    assert merged == {
        'tools': {
            'vacuum': {'active': False, 'tip': 'A'},
            'gripper': {'active': False},
        },
        'dispenser': {'tip': 'unchanged'},
    }
    assert current['tools']['vacuum']['active'] is True


def test_merge_rejects_unknown_properties_without_modifying_current():
    current = {'tools': {'vacuum': {'active': True}}}

    try:
        merge_configuration_subset(
            current,
            {'tools': {'vacuum': {'unknown': False}}},
        )
    except ValueError as error:
        assert 'tools.vacuum.unknown' in str(error)
    else:
        raise AssertionError('Expected an unknown property to be rejected')

    assert current == {'tools': {'vacuum': {'active': True}}}


@pytest.fixture
def configuration_state(tmp_path):
    path = tmp_path / 'bringup.yaml'
    loaded = {'tool': {'active': True, 'tip': 'A'}, 'other': False}
    path.write_text(yaml.safe_dump(loaded))
    return RobotConfigurationState(str(path), loaded)


def test_check_matches_loaded_and_disk(configuration_state):
    assert configuration_state.check({'tool': {'active': True}}) == []


def test_check_detects_file_edit_even_outside_required_fields(configuration_state):
    configuration_state.path.write_text(yaml.safe_dump({
        'tool': {'active': True, 'tip': 'A'}, 'other': True,
    }))
    differences = configuration_state.check({'tool': {'active': True}})
    assert len(differences) == 1
    assert 'differs from the loaded robot' in differences[0]


def test_update_does_not_change_loaded_robot(configuration_state):
    assert configuration_state.update({'tool': {'tip': 'B'}}) is True
    assert configuration_state.loaded['tool']['tip'] == 'A'
    assert read_configuration(configuration_state.path) == {
        'tool': {'active': True, 'tip': 'B'}, 'other': False,
    }
    differences = configuration_state.check({'tool': {'tip': 'B'}})
    assert any('Loaded robot: tool.tip' in difference for difference in differences)
    assert not any('On-disk: tool.tip' in difference for difference in differences)
    # Periodic reads and repeated checks cannot erase the mismatch.
    assert configuration_state.check({'tool': {'tip': 'B'}}) == differences


def test_check_reports_on_disk_requirement_mismatch(configuration_state):
    configuration_state.path.write_text(yaml.safe_dump({
        'tool': {'active': True, 'tip': 'B'}, 'other': False,
    }))
    differences = configuration_state.check({'tool': {'tip': 'A'}})
    assert any('On-disk: tool.tip' in difference for difference in differences)


def test_check_requires_bringup_snapshot(configuration_state):
    state = RobotConfigurationState(str(configuration_state.path), None)
    with pytest.raises(RuntimeError, match='No launch-time'):
        state.check({'other': False})


@pytest.mark.parametrize('updates', [
    {'unknown': True}, {'tool': {'active': 'true'}}, {'tool': 'invalid'},
])
def test_invalid_update_leaves_file_unchanged(configuration_state, updates):
    original = configuration_state.path.read_bytes()
    with pytest.raises(ValueError):
        configuration_state.update(updates)
    assert configuration_state.path.read_bytes() == original


def test_update_through_symlink_preserves_link(configuration_state, tmp_path):
    link = tmp_path / 'installed.yaml'
    link.symlink_to(configuration_state.path)
    state = RobotConfigurationState(str(link), configuration_state.loaded)
    assert state.update({'other': True}) is True
    assert link.is_symlink()
    assert read_configuration(configuration_state.path)['other'] is True


def test_restart_and_noop_update(configuration_state):
    assert configuration_state.update({'other': False}) is False
    configuration_state.update({'other': True})
    restarted = RobotConfigurationState(
        str(configuration_state.path), read_configuration(configuration_state.path)
    )
    assert restarted.check({'other': True}) == []
    assert restarted.update({'other': True}) is False


@pytest.mark.parametrize('contents', ['', '[]', 'tool: ['])
def test_invalid_yaml_is_rejected(configuration_state, contents):
    configuration_state.path.write_text(contents)
    with pytest.raises((ValueError, yaml.YAMLError)):
        configuration_state.check({'other': False})
    with pytest.raises((ValueError, yaml.YAMLError)):
        configuration_state.update({'other': True})
