"""Check config-only tool changes without starting ROS or modifying robot files."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from pm_msgs.srv import ChangeTool
from pm_robot_modules.submodules.pm_robot_config import PmRobotConfig, PmRobotToolsConfig
from pm_robot_primitive_skills import pm_robot_primitive_skills as skills


CONFIG = (
    Path(get_package_share_directory('pm_robot_bringup'))
    / 'config/pm_robot_bringup_config.yaml'
).read_text()
GRIPPERS = ['_gripper_vacuum', '_gripper_1_jaw', '_gripper_2_jaw']
PAIRS = [
    (attr, name, attachment)
    for attr in GRIPPERS
    for name, attachments in getattr(
        PmRobotToolsConfig(yaml.safe_load(CONFIG)), attr
    ).get_available_tools()
    for attachment in attachments
]


@pytest.fixture
def service(tmp_path, monkeypatch):
    paths = {True: tmp_path / 'real.yaml', False: tmp_path / 'sim.yaml'}
    for path in paths.values():
        path.write_text(CONFIG)

    def load_config(use_real_config):
        config = PmRobotConfig.__new__(PmRobotConfig)
        config._active_path = str(paths[use_real_config])
        config.reload_config()
        return config

    monkeypatch.setattr(skills, 'PmRobotConfig', load_config)
    node = SimpleNamespace(
        use_real_config=False, logger=Mock(),
        configuration_state=SimpleNamespace(loaded={"launch_snapshot": True}),
    )
    return node, paths


def call(node, tool, attachment):
    return skills.PrimitiveSkillsNode.change_tool_callback(
        node,
        ChangeTool.Request(tool_name=tool, attachment_name=attachment),
        ChangeTool.Response(),
    )


@pytest.mark.parametrize('use_real_config', [True, False])
@pytest.mark.parametrize('attr,tool,attachment', PAIRS)
def test_valid_pair_updates_only_active_file(service, use_real_config, attr, tool, attachment):
    node, paths = service
    node.use_real_config = use_real_config
    response = call(node, f' {tool} ', f' {attachment} ')
    assert response.success, response.message
    saved = PmRobotToolsConfig(yaml.safe_load(paths[use_real_config].read_text()))
    selected = getattr(saved, attr)
    assert selected.get_current_tool() == tool
    assert selected.get_current_tool_attachment() == attachment
    assert selected.get_activate_status()
    assert sum(getattr(saved, key).get_activate_status() for key in GRIPPERS) == 1
    assert paths[not use_real_config].read_text() == CONFIG
    assert str(paths[use_real_config]) in response.message
    assert node.configuration_state.loaded == {"launch_snapshot": True}


@pytest.mark.parametrize('tool,attachment', [
    ('unknown_tool', 'unknown_attachment'),
    ('', ''),
    ('Siemens_Vacuum_Array_angled', ''),
    ('Siemens_Vacuum_Array_angled', 'Sensing_Jaw'),
])
def test_invalid_request_leaves_files_unchanged(service, tool, attachment):
    node, paths = service
    response = call(node, tool, attachment)
    assert not response.success
    assert response.message
    assert all(path.read_text() == CONFIG for path in paths.values())


def test_read_error_is_reported(service):
    node, paths = service
    paths[False].unlink()
    response = call(node, 'Siemens_Vacuum_Array_angled', 'Tip_Siemens_Vacuum_Array')
    assert not response.success
    assert response.message
    assert paths[True].read_text() == CONFIG


def test_interface_contains_only_tool_and_attachment():
    assert set(ChangeTool.Request.get_fields_and_field_types()) == {
        'tool_name', 'attachment_name',
    }


def test_preserves_other_on_disk_changes(service):
    node, paths = service
    data = yaml.safe_load(paths[False].read_text())
    data['unrelated_setting'] = {'value': 'changed since launch'}
    paths[False].write_text(yaml.safe_dump(data))
    response = call(node, 'Siemens_Vacuum_Array_angled', 'Tip_Siemens_Vacuum_Array')
    assert response.success, response.message
    saved = yaml.safe_load(paths[False].read_text())
    assert saved['unrelated_setting'] == data['unrelated_setting']


def test_write_error_is_reported(service, monkeypatch):
    node, paths = service
    monkeypatch.setattr(PmRobotConfig, 'save_config', Mock(side_effect=OSError('write failed')))
    response = call(node, 'Siemens_Vacuum_Array_angled', 'Tip_Siemens_Vacuum_Array')
    assert not response.success
    assert 'write failed' in response.message
    assert all(path.read_text() == CONFIG for path in paths.values())
