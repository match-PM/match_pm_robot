"""Offline tests for the SmarPod controller switching sequence."""

from concurrent.futures import Future
import subprocess
import sys

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from smarpod_interfaces.srv import SetVirtualPivot
from std_srvs.srv import Trigger

from pm_robot_primitive_skills.py_modules.smarpod_virtual_controller import (
    SmarpodVirtualController,
)


class FakeClient:
    def __init__(self, name, handler):
        self.srv_name = name
        self.handler = handler

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        future = Future()
        future.set_result(self.handler(request))
        return future


class FakeNode:
    def __init__(self):
        self.events = []
        self.states = {
            SmarpodVirtualController.SMARPOD: 'active',
            SmarpodVirtualController.DEFAULT: 'active',
            SmarpodVirtualController.VIRTUAL: 'inactive',
        }
        self.set_result = SetVirtualPivot.Response(
            success=True, message='Pivot selected')
        self.reset_result = Trigger.Response(
            success=True, message='Pivot reset')
        self.fail_activation_of = None

    def create_client(self, service_type, name, callback_group):
        return FakeClient(name, {
            ListControllers: self.list_controllers,
            SwitchController: self.switch_controller,
            SetVirtualPivot: self.set_pivot,
            Trigger: self.reset_pivot,
        }[service_type])

    def list_controllers(self, request):
        return ListControllers.Response(controller=[
            ControllerState(name=name, state=state)
            for name, state in self.states.items()
        ])

    def switch_controller(self, request):
        if self.fail_activation_of in request.activate_controllers:
            return SwitchController.Response(ok=False)
        for name in request.deactivate_controllers:
            self.events.append(f'deactivate:{name}')
            self.states[name] = 'inactive'
        for name in request.activate_controllers:
            self.events.append(f'activate:{name}')
            self.states[name] = 'active'
        return SwitchController.Response(ok=True)

    def set_pivot(self, request):
        self.events.append(f'set:{request.frame_name}')
        assert self.states[SmarpodVirtualController.DEFAULT] == 'inactive'
        return self.set_result

    def reset_pivot(self, request):
        self.events.append('reset')
        assert self.states[SmarpodVirtualController.VIRTUAL] == 'inactive'
        return self.reset_result


def test_activate_and_reset_use_safe_order():
    node = FakeNode()
    switcher = SmarpodVirtualController(node)

    success, message = switcher.activate('smarpod_chuck')
    assert success, message
    success, message = switcher.reset()
    assert success, message
    assert node.events == [
        f'deactivate:{switcher.DEFAULT}',
        'set:smarpod_chuck',
        f'activate:{switcher.VIRTUAL}',
        f'deactivate:{switcher.VIRTUAL}',
        'reset',
        f'activate:{switcher.DEFAULT}',
    ]


def test_failed_frame_selection_restores_default_controller():
    node = FakeNode()
    node.set_result = SetVirtualPivot.Response(
        success=False, message='TF lookup failed')
    switcher = SmarpodVirtualController(node)

    success, message = switcher.activate('missing_frame')
    assert not success
    assert 'TF lookup failed' in message
    assert 'default controller restored' in message
    assert node.states[switcher.DEFAULT] == 'active'
    assert node.events[-2:] == ['reset', f'activate:{switcher.DEFAULT}']


def test_failed_reset_does_not_activate_default_controller():
    node = FakeNode()
    switcher = SmarpodVirtualController(node)
    assert switcher.activate('smarpod_chuck')[0]
    node.reset_result = Trigger.Response(
        success=False, message='Stage still moving')

    success, message = switcher.reset()
    assert not success
    assert 'Stage still moving' in message
    assert node.states[switcher.DEFAULT] == 'inactive'
    assert node.states[switcher.VIRTUAL] == 'inactive'
    assert node.events[-2:] == [f'deactivate:{switcher.VIRTUAL}', 'reset']


def test_failed_virtual_activation_restores_original_mode():
    node = FakeNode()
    node.fail_activation_of = SmarpodVirtualController.VIRTUAL
    switcher = SmarpodVirtualController(node)

    success, message = switcher.activate('smarpod_chuck')
    assert not success
    assert 'activate smaract_hexapod_virtual_controller' in message
    assert 'default controller restored' in message
    assert node.states[switcher.DEFAULT] == 'active'
    assert node.states[switcher.VIRTUAL] == 'inactive'
    assert node.events[-2:] == ['reset', f'activate:{switcher.DEFAULT}']


def test_empty_frame_rejected_without_switching():
    node = FakeNode()
    success, message = SmarpodVirtualController(node).activate('  ')
    assert not success
    assert 'frame_name' in message
    assert not node.events


def test_node_starts_without_smarpod_interfaces():
    script = '''
import sys
sys.modules['smarpod_interfaces'] = None
import rclpy
from pm_robot_primitive_skills.pm_robot_primitive_skills import (
    PrimitiveSkillsNode
)
rclpy.init()
node = PrimitiveSkillsNode()
names = {name for name, _ in node.get_service_names_and_types()}
assert '/pm_robot_primitive_skills/reset_test_station' in names
assert not any('smarpod_virtual_controller' in name for name in names)
node.destroy_node()
rclpy.shutdown()
'''
    result = subprocess.run(
        [sys.executable, '-c', script], capture_output=True, text=True,
        timeout=30)
    assert result.returncode == 0, result.stderr
