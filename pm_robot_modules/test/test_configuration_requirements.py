import copy
from pathlib import Path

import pytest
import yaml

from pm_robot_modules.submodules.configuration_requirements import (
    DEFAULT_CONFIGURATION_FILE_NAME,
    RobotConfigurationState,
    parse_configuration,
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


def test_loaded_only_ignores_unloaded_file_edits(configuration_state):
    configuration_state.path.write_text(yaml.safe_dump({
        'tool': {'active': True, 'tip': 'B'}, 'other': True,
    }))
    assert configuration_state.check(
        {'tool': {'tip': 'A'}}, loaded_only=True) == []


def test_loaded_only_reports_loaded_mismatch(configuration_state):
    assert configuration_state.check(
        {'tool': {'tip': 'B'}}, loaded_only=True
    ) == ["Loaded robot: tool.tip: required 'B', current 'A'"]


def test_loaded_only_does_not_need_the_file(configuration_state):
    configuration_state.path.unlink()
    assert configuration_state.check({'other': False}, loaded_only=True) == []


def test_parse_configuration_text():
    assert parse_configuration('tool:\n  active: true\n') == {
        'tool': {'active': True}
    }


@pytest.mark.parametrize('contents', ['', '[]', '42'])
def test_parse_configuration_rejects_empty_or_non_mapping(contents):
    with pytest.raises(ValueError, match='configuration_yaml'):
        parse_configuration(contents, 'configuration_yaml')


ROBOT = {
    'measuring_systems': {'use_confocal_head': False, 'use_confocal_bottom': True},
    'confocal_sensor_head': {'use_config': 'z_5'},
    'pm_robot_gonio_left': {'use_chuck': 'A', 'with_Gonio_Left': True},
    'pm_smparpod_station': {
        'use_chuck': 'empty', 'use_chuck_center': 'empty', 'with_smarpod_station': False},
    'pm_robot_tools': {
        'pm_robot_tool_parallel_gripper_2_jaws': {
            'use_jaw_type': 'Jaw_ASG', 'use_paralell_gripper': False, 'use_tool': 'MPG_20'},
        'pm_robot_vacuum_tools': {
            'use_tip': 'Tip_A', 'use_tool': 'Array', 'use_vacuum_tool': True},
    },
}


def _required(**changes):
    required = copy.deepcopy(ROBOT)
    for path, value in changes.items():
        *parents, key = path.split('__')
        target = required
        for parent in parents:
            target = target[parent]
        target[key] = value
    return required


def test_selection_of_disabled_gripper_is_ignored():
    required = _required(
        pm_robot_tools__pm_robot_tool_parallel_gripper_2_jaws__use_tool='MPG_10',
        pm_robot_tools__pm_robot_tool_parallel_gripper_2_jaws__use_jaw_type='Jaw_Lens')
    assert compare_configuration_subset(ROBOT, required, ignore_disabled=True) == []
    # Without the option every property is compared, as before.
    assert len(compare_configuration_subset(ROBOT, required)) == 2


def test_selection_of_enabled_tool_is_still_checked():
    required = _required(pm_robot_tools__pm_robot_vacuum_tools__use_tip='Tip_B')
    assert compare_configuration_subset(ROBOT, required, ignore_disabled=True) == [
        "pm_robot_tools.pm_robot_vacuum_tools.use_tip: required 'Tip_B', current 'Tip_A'"]


def test_enable_flag_itself_is_always_checked():
    required = _required(
        pm_robot_tools__pm_robot_tool_parallel_gripper_2_jaws__use_paralell_gripper=True)
    assert compare_configuration_subset(ROBOT, required, ignore_disabled=True) == [
        'pm_robot_tools.pm_robot_tool_parallel_gripper_2_jaws.use_paralell_gripper: '
        'required True, current False']


def test_required_disabled_while_robot_enabled_reports_only_the_flag():
    required = _required(
        pm_robot_gonio_left__with_Gonio_Left=False, pm_robot_gonio_left__use_chuck='B')
    assert compare_configuration_subset(ROBOT, required, ignore_disabled=True) == [
        'pm_robot_gonio_left.with_Gonio_Left: required False, current True']


def test_flag_in_another_section_disables_the_section():
    # confocal_sensor_head is enabled by measuring_systems.use_confocal_head.
    required = _required(confocal_sensor_head__use_config='z_20')
    assert compare_configuration_subset(ROBOT, required, ignore_disabled=True) == []
    required['measuring_systems']['use_confocal_head'] = True
    assert len(compare_configuration_subset(ROBOT, required, ignore_disabled=True)) == 2


def test_disabled_smarpod_chucks_are_ignored():
    required = _required(pm_smparpod_station__use_chuck='chuck_fiber')
    assert compare_configuration_subset(ROBOT, required, ignore_disabled=True) == []


def test_section_without_flag_in_requirements_is_compared():
    # Requirements that omit the enable flag keep the strict comparison.
    required = {'pm_robot_tools': {'pm_robot_tool_parallel_gripper_2_jaws': {
        'use_tool': 'MPG_10'}}}
    assert len(compare_configuration_subset(ROBOT, required, ignore_disabled=True)) == 1


def test_check_ignores_disabled_selections(tmp_path):
    path = tmp_path / 'bringup.yaml'
    path.write_text(yaml.safe_dump(ROBOT))
    state = RobotConfigurationState(str(path), ROBOT)
    required = _required(
        pm_robot_tools__pm_robot_tool_parallel_gripper_2_jaws__use_tool='MPG_10')
    assert state.check(required) == []
    assert state.check(required, loaded_only=True) == []


def test_unloaded_edit_of_disabled_selection_still_needs_restart(tmp_path):
    # The file-vs-loaded comparison stays strict: an edited file is not loaded.
    path = tmp_path / 'bringup.yaml'
    edited = _required(
        pm_robot_tools__pm_robot_tool_parallel_gripper_2_jaws__use_tool='MPG_10')
    path.write_text(yaml.safe_dump(edited))
    state = RobotConfigurationState(str(path), ROBOT)
    differences = state.check({'measuring_systems': {'use_confocal_bottom': True}})
    assert differences == [
        'The on-disk configuration differs from the loaded robot configuration. '
        'Restart the robot to load the file.']
