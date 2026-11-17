import copy
from pathlib import Path

import pytest
import yaml

from pm_robot_sim_state.checks import joint_deviations, simulated_hardware_problems
from pm_robot_sim_state.state_file import (
    FORMAT_VERSION, StateFileError, build_document, read_document,
    resolve_state_path, select_configuration, write_document)


BRINGUP = {
    'adhesive_test_point_station': [{'current_point': 4, 'num_x_points': 8}],
    'confocal_sensor_head': {
        'availabe_configs': ['a', 'b'],
        'use_config': 'a',
    },
    'pm_robot_gonio_left': {
        'availabe_chucks': ['c1', 'c2'],
        'unity_parent': 'Gonio_Left_Chuck_Origin',
        'use_chuck': 'c1',
        'with_Gonio_Left': True,
    },
    'pm_robot_tools': {
        'pm_robot_tool_parallel_gripper_1_jaw': {
            'available_tools': [{'tool_name': 'SmarAct', 'available_jaws': ['J']}],
            'use_jaw_type': 'J',
            'use_paralell_gripper': False,
            'use_tool': 'SmarAct',
        },
    },
    'pm_smparpod_station': {'with_smarpod_station': False, 'use_chuck': 'empty'},
}

STATE = {
    'unity_joints': {'RobotAxisX': 0.01, 'RobotAxisT': 1.5},
    'joint_states': {'X_Axis_Joint': 0.01, 'T_Axis_Joint': -1.5},
    'smarpod': {'pose': [0.0, 0.001, 0.0, 0.0, 0.0, 0.1], 'speed': 0.001,
                'virtual_pivot_active': False},
    'pneumatics': {'1K_Dispenser_Joint': -1, 'UV_LED_Front_Joint': 1},
    'nozzles': {'Head_Nozzle': -1, 'Gonio_Nozzle': 0},
    'lights': {'coax_light': True, 'ring_light': [True, False, False, True],
               'ring_light_rgb': [255, 0, 10], 'camera2_light': 50},
    'uv': {'on_off': [False] * 4, 'power': [10, 20, 30, 40], 'time': [1.0, 2.0, 3.0, 4.0]},
}


def document(state=None):
    return build_document(
        configuration=select_configuration(BRINGUP),
        robot_state=copy.deepcopy(STATE if state is None else state),
        description='test',
        created='2026-11-17T12:00:00',
    )


def test_select_configuration_keeps_only_equipment_selection():
    assert select_configuration(BRINGUP) == {
        'confocal_sensor_head': {'use_config': 'a'},
        'pm_robot_gonio_left': {'use_chuck': 'c1', 'with_Gonio_Left': True},
        'pm_robot_tools': {
            'pm_robot_tool_parallel_gripper_1_jaw': {
                'use_jaw_type': 'J',
                'use_paralell_gripper': False,
                'use_tool': 'SmarAct',
            },
        },
        'pm_smparpod_station': {'with_smarpod_station': False, 'use_chuck': 'empty'},
    }


def test_resolve_relative_name_into_state_directory(tmp_path):
    assert resolve_state_path('my_state', str(tmp_path)) == tmp_path / 'my_state.yaml'
    assert resolve_state_path('sub/a.yml', str(tmp_path)) == tmp_path / 'sub' / 'a.yml'


def test_resolve_absolute_path_ignores_state_directory(tmp_path):
    target = tmp_path / 'elsewhere' / 'state.yaml'
    assert resolve_state_path(str(target), '/unused') == target


def test_resolve_rejects_empty_name():
    with pytest.raises(StateFileError):
        resolve_state_path('  ', '/tmp')


def test_write_and_read_round_trip(tmp_path):
    path = tmp_path / 'states' / 'a.yaml'
    write_document(path, document(), overwrite=False)
    loaded = read_document(path)
    assert loaded == document()
    assert loaded['format_version'] == FORMAT_VERSION
    # The configuration is plain YAML inside the same file.
    assert yaml.safe_load(path.read_text())['pm_robot_configuration'][
        'pm_robot_gonio_left']['use_chuck'] == 'c1'


def test_write_refuses_to_overwrite_unless_asked(tmp_path):
    path = tmp_path / 'a.yaml'
    write_document(path, document(), overwrite=False)
    with pytest.raises(StateFileError, match='already exists'):
        write_document(path, document(), overwrite=False)
    write_document(path, document(), overwrite=True)
    assert not [p for p in tmp_path.iterdir() if p.name.startswith('.')]


def test_optional_sections_may_be_missing():
    state = {key: STATE[key] for key in ('unity_joints', 'joint_states')}
    assert document(state)['robot_state'] == state


@pytest.mark.parametrize('change, message', [
    (lambda s: s.update(unity_joints={}), 'must not be empty'),
    (lambda s: s['unity_joints'].update(RobotAxisX='a'), 'finite number'),
    (lambda s: s['unity_joints'].update(RobotAxisX=float('nan')), 'finite number'),
    (lambda s: s.pop('joint_states'), 'joint_states'),
    (lambda s: s['smarpod'].update(pose=[0.0] * 5), 'smarpod.pose'),
    (lambda s: s['pneumatics'].update(Glue=0), 'pneumatics.Glue'),
    (lambda s: s['nozzles'].update(Head_Nozzle=True), 'nozzles.Head_Nozzle'),
    (lambda s: s['lights'].update(ring_light_rgb=[256, 0, 0]), 'ring_light_rgb'),
    (lambda s: s['uv'].update(power=[1.5, 2, 3, 4]), 'uv.power'),
])
def test_invalid_states_are_rejected(change, message):
    state = copy.deepcopy(STATE)
    change(state)
    with pytest.raises(StateFileError, match=message):
        document(state)


def test_read_rejects_unknown_version_and_missing_file(tmp_path):
    with pytest.raises(StateFileError, match='does not exist'):
        read_document(tmp_path / 'missing.yaml')
    path = tmp_path / 'old.yaml'
    data = document()
    data['format_version'] = 99
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(StateFileError, match='format_version'):
        read_document(path)


def _description(endpoint='opc.tcp://localhost:4840',
                 smarpod_plugin='smaract_hexapod_unity_hw_interface/SmarPodUnitySystem'):
    return f"""<?xml version="1.0"?>
<robot name="pm_robot">
  <ros2_control name="PMSystem" type="system">
    <hardware>
      <plugin>pm_hardware_interface/PMSystem</plugin>
      <param name="opcua_endpoint">{endpoint}</param>
    </hardware>
  </ros2_control>
  <ros2_control name="PMSystemPneumatic_2" type="system">
    <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
  </ros2_control>
  <ros2_control name="SmarPod System" type="system">
    <hardware><plugin>{smarpod_plugin}</plugin></hardware>
  </ros2_control>
</robot>"""


def test_unity_description_is_simulated():
    assert simulated_hardware_problems(_description()) == []
    assert simulated_hardware_problems(
        _description(endpoint='opc.tcp://127.0.0.1:4840')) == []


def test_real_opcua_endpoint_is_rejected():
    problems = simulated_hardware_problems(_description(endpoint='opc.tcp://PC1M0484-1:4840'))
    assert len(problems) == 1
    assert 'PC1M0484-1' in problems[0]


def test_real_smarpod_is_rejected():
    problems = simulated_hardware_problems(
        _description(smarpod_plugin='smaract_hexapod_hw_interface/SmarPodSystem'))
    assert len(problems) == 1
    assert 'SmarPodSystem' in problems[0]


def test_missing_pm_system_and_bad_xml_are_rejected():
    assert simulated_hardware_problems('<robot name="x"/>')
    assert simulated_hardware_problems('<robot')


def test_joint_deviations():
    expected = {'a': 1.0, 'b': 2.0, 'c': 3.0}
    actual = {'a': 1.00001, 'b': 2.5}
    assert joint_deviations(expected, actual, 1e-4) == [
        'b: expected 2.000000, is 2.500000',
        'c: missing from /joint_states',
    ]
    assert joint_deviations(expected, actual, 1e-4, names=['a', 'unknown']) == []


def test_module_has_no_ros_imports():
    # state_file and checks must stay importable without a running ROS graph.
    source = Path(__file__).parents[1] / 'pm_robot_sim_state'
    for name in ('state_file.py', 'checks.py'):
        assert 'rclpy' not in (source / name).read_text()
