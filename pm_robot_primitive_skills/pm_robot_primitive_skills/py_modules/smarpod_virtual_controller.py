"""Switch the SmarPod between its original and virtual pivot controllers."""

from threading import Event

from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.callback_groups import ReentrantCallbackGroup
from smarpod_interfaces.srv import SetVirtualPivot
from std_srvs.srv import Trigger


class SmarpodVirtualController:
    """Coordinate controller switches with the SmarPod pivot services."""

    DEFAULT = 'smaract_hexapod_controller'
    VIRTUAL = 'smaract_hexapod_virtual_controller'
    SMARPOD = 'smarpod_controller'
    MOTION_JOINTS = {f'SP_{axis}_Joint' for axis in 'XYZABC'} | {
        f'SP_PV_{axis}_Joint' for axis in 'XYZABC'
    }

    def __init__(self, node):
        group = ReentrantCallbackGroup()
        self.list_client = node.create_client(
            ListControllers, '/controller_manager/list_controllers',
            callback_group=group)
        self.switch_client = node.create_client(
            SwitchController, '/controller_manager/switch_controller',
            callback_group=group)
        self.set_client = node.create_client(
            SetVirtualPivot, '/smarpod_controller/SetVirtualPivot',
            callback_group=group)
        self.reset_client = node.create_client(
            Trigger, '/smarpod_controller/ResetVirtualPivot',
            callback_group=group)

    @staticmethod
    def _call(client, request, timeout=7.0):
        name = client.srv_name
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"Service '{name}' is unavailable")
        try:
            future = client.call_async(request)
            complete = Event()
            future.add_done_callback(lambda _: complete.set())
            if not complete.wait(timeout):
                client.remove_pending_request(future)
                raise RuntimeError(
                    f"Service '{name}' timed out; state is uncertain")
            result = future.result()
        except RuntimeError:
            raise
        except Exception as error:
            raise RuntimeError(f"Service '{name}' failed: {error}") from error
        if result is None:
            raise RuntimeError(f"Service '{name}' returned no response")
        return result

    def _states(self):
        result = self._call(self.list_client, ListControllers.Request())
        controllers = result.controller
        states = {controller.name: controller for controller in controllers}
        for name in (self.SMARPOD, self.DEFAULT, self.VIRTUAL):
            if name not in states:
                raise RuntimeError(f"Controller '{name}' is not loaded")
        if states[self.SMARPOD].state != 'active':
            raise RuntimeError(f"Controller '{self.SMARPOD}' is not active")
        for name in (self.DEFAULT, self.VIRTUAL):
            if states[name].state not in ('active', 'inactive'):
                raise RuntimeError(
                    f"Controller '{name}' is {states[name].state}")
        both_active = (states[self.DEFAULT].state == 'active' and
                       states[self.VIRTUAL].state == 'active')
        if both_active:
            raise RuntimeError('Both SmarPod motion controllers are active')
        for controller in controllers:
            claims_motion = any(
                interface.split('/')[0] in self.MOTION_JOINTS
                for interface in controller.claimed_interfaces)
            is_other = controller.name not in (self.DEFAULT, self.VIRTUAL)
            if controller.state == 'active' and claims_motion and is_other:
                raise RuntimeError(
                    f"Controller '{controller.name}' claims motion joints")
        return states

    def _switch(self, activate=(), deactivate=()):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 5
        result = self._call(self.switch_client, request)
        if not result.ok:
            actions = [f"activate {name}" for name in activate]
            actions += [f"deactivate {name}" for name in deactivate]
            raise RuntimeError(
                f"Controller manager could not {', '.join(actions)}")

    def _restore_default(self):
        """Recover after an activation failure stops the controller."""
        states = self._states()
        if states[self.VIRTUAL].state == 'active':
            self._switch(deactivate=(self.VIRTUAL,))
        result = self._call(self.reset_client, Trigger.Request())
        if not result.success:
            raise RuntimeError(f"ResetVirtualPivot failed: {result.message}")
        states = self._states()
        if states[self.DEFAULT].state != 'active':
            self._switch(activate=(self.DEFAULT,))

    def activate(self, frame_name):
        """Select a TF frame and activate the virtual trajectory controller."""
        frame_name = frame_name.strip()
        if not frame_name:
            return False, 'frame_name must not be empty'
        try:
            states = self._states()
            if states[self.VIRTUAL].state == 'active':
                return False, (
                    'Virtual controller is already active; reset it before '
                    'selecting another frame')
            if states[self.DEFAULT].state == 'active':
                self._switch(deactivate=(self.DEFAULT,))
        except RuntimeError as error:
            return False, str(error)

        try:
            result = self._call(
                self.set_client,
                SetVirtualPivot.Request(frame_name=frame_name))
            if not result.success:
                raise RuntimeError(f"SetVirtualPivot failed: {result.message}")
            self._switch(activate=(self.VIRTUAL,))
        except RuntimeError as error:
            try:
                self._restore_default()
                return False, f"{error}; default controller restored"
            except RuntimeError as recovery_error:
                return False, (
                    f"{error}; recovery failed: {recovery_error}. "
                    'Check controller state before moving')
        return True, (
            f"Virtual SmarPod controller active with frame '{frame_name}'")

    def reset(self):
        """Restore the original pivot and trajectory controller."""
        try:
            states = self._states()
            if states[self.DEFAULT].state == 'active':
                return True, 'Default SmarPod controller is already active'
            if states[self.VIRTUAL].state == 'active':
                self._switch(deactivate=(self.VIRTUAL,))
            result = self._call(self.reset_client, Trigger.Request())
            if not result.success:
                raise RuntimeError(
                    f"ResetVirtualPivot failed: {result.message}")
            self._switch(activate=(self.DEFAULT,))
        except RuntimeError as error:
            return False, f"{error}. Check controller state before moving"
        return True, 'Default SmarPod controller active; virtual pivot reset'
