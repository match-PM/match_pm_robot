"""
Save and restore complete states of the Unity-simulated PM robot.

Services (only started by pm_robot_unity_HW.launch.py):

    ~/save_robot_state  pm_msgs/srv/SaveRobotState
    ~/load_robot_state  pm_msgs/srv/LoadRobotState

A load first checks the configuration saved in the file against the
configuration the robot was launched with. On a mismatch the saved
configuration is written to the bringup file, Unity is restarted and the
caller is asked to restart the robot launch. Otherwise the robot jumps to
the saved state:

1. deactivate the motion controllers (trajectory and forward controllers),
   so nothing commands the old positions,
2. place every joint and the SmarPod directly in Unity,
3. wait until /joint_states reports the new positions,
4. reactivate the controllers - they start by holding the new positions,
5. restore pneumatics, nozzles (vacuum), lights and UV through their
   controllers, and verify the result.
"""

from datetime import datetime
from threading import Event, Lock
import time

from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SwitchController)
from pm_msgs.msg import PmRobotConfiguration
import pm_msgs.srv as pm_srv
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
import yaml

from pm_robot_sim_state.checks import (
    REAL_HARDWARE_PLUGINS, joint_deviations, simulated_hardware_problems)
from pm_robot_sim_state.state_file import (
    DEFAULT_STATE_DIRECTORY, StateFileError, build_document, read_document,
    resolve_state_path, select_configuration, write_document)

try:
    from smarpod_interfaces.srv import SetSpeed as SmarpodSetSpeed
except ImportError:  # SmarPod packages are optional
    SmarpodSetSpeed = None


TRAJECTORY_CONTROLLER = 'joint_trajectory_controller/JointTrajectoryController'
FORWARD_CONTROLLERS = frozenset({
    'position_controllers/JointGroupPositionController',
    'forward_command_controller/ForwardCommandController',
})
MOTION_CONTROLLERS = FORWARD_CONTROLLERS | {TRAJECTORY_CONTROLLER}

SMARPOD_DEFAULT_CONTROLLER = 'smaract_hexapod_controller'
SMARPOD_VIRTUAL_CONTROLLER = 'smaract_hexapod_virtual_controller'

CONFIGURATION_MISMATCH_PREFIX = 'PM robot configuration does not match'


class StateServiceError(RuntimeError):
    """A step of a save or load failed; the message is returned to the caller."""


def _topic_segment(name: str) -> str:
    """Topic segment the pneumatic controller uses for a cylinder name."""
    # pm_pneumatic_controller prepends 'N' because names may not start with digits.
    return f'N{name}' if name[:1].isdigit() else name


class SimStateNode(Node):

    def __init__(self):
        super().__init__('pm_robot_sim_state')

        self.state_directory = self.declare_parameter(
            'state_directory', DEFAULT_STATE_DIRECTORY).value
        self.joint_tolerance = float(self.declare_parameter(
            'joint_tolerance', 1e-4).value)
        self.service_timeout = float(self.declare_parameter(
            'service_timeout', 10.0).value)
        self.settle_time = float(self.declare_parameter(
            'settle_time', 0.5).value)
        self.state_timeout = float(self.declare_parameter(
            'state_timeout', 10.0).value)
        self.controller_manager = self.declare_parameter(
            'controller_manager', '/controller_manager').value
        self.configuration_node = self.declare_parameter(
            'configuration_node', '/pm_robot_primitive_skills').value
        self.unity_get_service = self.declare_parameter(
            'unity_get_service', '/unity/get_sim_robot_state').value
        self.unity_set_service = self.declare_parameter(
            'unity_set_service', '/unity/set_sim_robot_state').value
        self.unity_restart_service = self.declare_parameter(
            'unity_restart_service', '/unity/restart_simulation').value
        self.unity_configure_service = self.declare_parameter(
            'unity_configure_service', '/configure_robot_node/configure_robot').value

        self._client_group = ReentrantCallbackGroup()
        self._service_clients = {}
        self._command_publishers = {}

        self._data_lock = Lock()
        self._joint_states = {}
        self._joint_states_received = 0.0
        self._robot_description = None
        self._loaded_configuration = None

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10,
            callback_group=self._client_group)
        self.create_subscription(
            String, '/robot_description', self._on_robot_description, latched,
            callback_group=self._client_group)
        self.create_subscription(
            PmRobotConfiguration,
            f'{self.configuration_node}/current_pm_robot_configuration',
            self._on_configuration, latched,
            callback_group=self._client_group)

        # One save or load at a time.
        service_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            pm_srv.SaveRobotState, '~/save_robot_state', self._save_callback,
            callback_group=service_group)
        self.create_service(
            pm_srv.LoadRobotState, '~/load_robot_state', self._load_callback,
            callback_group=service_group)

        self.get_logger().info(
            f"Robot state services ready; states are stored in "
            f"'{self.state_directory}'.")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_joint_states(self, message: JointState):
        with self._data_lock:
            for name, position in zip(message.name, message.position):
                self._joint_states[name] = float(position)
            self._joint_states_received = time.monotonic()

    def _on_robot_description(self, message: String):
        with self._data_lock:
            self._robot_description = message.data

    def _on_configuration(self, message: PmRobotConfiguration):
        with self._data_lock:
            self._loaded_configuration = message

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _save_callback(self, request, response):
        try:
            path, message = self._save(request)
            response.success = True
            response.file_path = str(path)
            response.message = message
            self.get_logger().info(message)
        except (StateServiceError, StateFileError, OSError) as error:
            response.success = False
            response.message = f'Robot state not saved: {error}'
            self.get_logger().error(response.message)
        return response

    def _load_callback(self, request, response):
        try:
            restart_required, message = self._load(request)
            response.success = not restart_required
            response.restart_required = restart_required
            response.message = message
            log = self.get_logger().warn if restart_required else self.get_logger().info
            log(message)
        except (StateServiceError, StateFileError, OSError) as error:
            response.success = False
            response.restart_required = False
            response.message = f'Robot state not loaded: {error}'
            self.get_logger().error(response.message)
        return response

    # ------------------------------------------------------------------
    # Save
    # ------------------------------------------------------------------

    def _save(self, request):
        path = resolve_state_path(request.file_path, self.state_directory)
        if path.exists() and not request.overwrite:
            raise StateFileError(
                f"'{path}' already exists. Set overwrite to replace it.")
        self._ensure_simulation()

        configuration_message = self._wait_for_configuration()
        loaded = yaml.safe_load(configuration_message.config_yaml)
        if not isinstance(loaded, dict):
            raise StateServiceError('The loaded robot configuration is not a mapping.')
        configuration = select_configuration(loaded)
        if not configuration:
            raise StateServiceError(
                'The loaded robot configuration has no with_*/use_* properties.')

        unity = self._call(
            pm_srv.GetSimRobotState, self.unity_get_service,
            pm_srv.GetSimRobotState.Request())
        if not unity.success:
            raise StateServiceError(f'Unity could not read the state: {unity.message}')
        if len(unity.joint_names) != len(unity.joint_positions):
            raise StateServiceError(
                'Unity returned different numbers of joint names and positions.')

        robot_state = {
            'unity_joints': {
                name: float(position)
                for name, position in zip(unity.joint_names, unity.joint_positions)
            },
            'joint_states': self._fresh_joint_states(),
        }

        notes = []
        if unity.has_smarpod:
            controllers = self._list_controllers()
            virtual = controllers.get(SMARPOD_VIRTUAL_CONTROLLER)
            robot_state['smarpod'] = {
                'pose': [float(value) for value in unity.smarpod_pose],
                'speed': float(unity.smarpod_speed),
                'virtual_pivot_active': virtual is not None and virtual.state == 'active',
            }

        for section, reader in (
                ('pneumatics', self._read_pneumatics),
                ('nozzles', self._read_nozzles),
                ('lights', self._read_lights),
                ('uv', self._read_uv)):
            try:
                robot_state[section] = reader()
            except StateServiceError as error:
                notes.append(f'{section} not saved ({error})')

        document = build_document(
            configuration=configuration,
            robot_state=robot_state,
            description=request.description,
            created=datetime.now().isoformat(timespec='seconds'),
        )
        write_document(path, document, request.overwrite)

        message = (
            f"Saved the robot state to '{path}' "
            f"({len(robot_state['unity_joints'])} joints"
            f"{', SmarPod' if 'smarpod' in robot_state else ''}).")
        if notes:
            message += ' Not saved: ' + '; '.join(notes) + '.'
        return path, message

    def _read_pneumatics(self):
        values = {}
        for name in self._controller_list_parameter('/pm_pneumatic_controller', 'cylinders'):
            result = self._call(
                pm_srv.PneumaticGetPosition,
                f'/pm_pneumatic_controller/{_topic_segment(name)}/GetPosition',
                pm_srv.PneumaticGetPosition.Request())
            # Neutral (0) is not a rest position and cannot be commanded back.
            if result.position in (-1, 1):
                values[name] = int(result.position)
        return values

    def _read_nozzles(self):
        values = {}
        for name in self._controller_list_parameter('/pm_nozzle_controller', 'nozzles'):
            result = self._call(
                pm_srv.NozzleGetPosition,
                f'/pm_nozzle_controller/{name}/GetPosition',
                pm_srv.NozzleGetPosition.Request())
            if result.position in (-1, 0, 1):
                values[name] = int(result.position)
        return values

    def _read_lights(self):
        coax = self._call(
            pm_srv.CoaxLightGetState, '/pm_lights_controller/CoaxLight/GetState',
            pm_srv.CoaxLightGetState.Request())
        ring = self._call(
            pm_srv.RingLightGetState, '/pm_lights_controller/RingLight/GetState',
            pm_srv.RingLightGetState.Request())
        camera2 = self._call(
            pm_srv.Cam2LightGetState, '/pm_lights_controller/Cam2Light/GetState',
            pm_srv.Cam2LightGetState.Request())
        return {
            'coax_light': bool(coax.is_on),
            'ring_light': [bool(value) for value in ring.is_on],
            'ring_light_rgb': [int(value) for value in ring.rgb],
            'camera2_light': int(camera2.intensity),
        }

    def _read_uv(self):
        on_off = self._call(
            pm_srv.UVGetOnOff, '/pm_uv_controller/Hoenle_UV/GetOnOff',
            pm_srv.UVGetOnOff.Request())
        power = self._call(
            pm_srv.UVGetPower, '/pm_uv_controller/Hoenle_UV/GetPower',
            pm_srv.UVGetPower.Request())
        uv_time = self._call(
            pm_srv.UVGetTime, '/pm_uv_controller/Hoenle_UV/GetTime',
            pm_srv.UVGetTime.Request())
        return {
            'on_off': [bool(value) for value in on_off.is_on],
            'power': [int(value) for value in power.power],
            'time': [float(value) for value in uv_time.time],
        }

    # ------------------------------------------------------------------
    # Load
    # ------------------------------------------------------------------

    def _load(self, request):
        path = resolve_state_path(request.file_path, self.state_directory)
        document = read_document(path)
        self._ensure_simulation()

        configuration_yaml = yaml.safe_dump(
            document['pm_robot_configuration'], sort_keys=False)
        check = self._call(
            pm_srv.CheckPmRobotConfiguration,
            f'{self.configuration_node}/check_pm_robot_configuration',
            pm_srv.CheckPmRobotConfiguration.Request(
                configuration_yaml=configuration_yaml, check_loaded_only=True))
        if not check.success:
            if not check.message.startswith(CONFIGURATION_MISMATCH_PREFIX):
                raise StateServiceError(f'Configuration check failed: {check.message}')
            return True, self._prepare_restart(path, configuration_yaml, check.message)

        return False, self._restore(path, document['robot_state'])

    def _prepare_restart(self, path, configuration_yaml, differences):
        """Write the saved configuration and restart Unity with it."""
        update = self._call(
            pm_srv.SetPmRobotConfiguration,
            f'{self.configuration_node}/set_pm_robot_configuration',
            pm_srv.SetPmRobotConfiguration.Request(configuration_yaml=configuration_yaml))
        if not update.success:
            raise StateServiceError(
                f'The robot configuration differs from the state file, and the '
                f'saved configuration could not be written: {update.message}')

        unity_note = self._restart_unity()
        return (
            f"The robot configuration differs from '{path}':\n{differences}\n"
            f'The saved configuration was written to the bringup file. '
            f'{unity_note} Restart the robot launch, then call load_robot_state '
            'again to restore the state.')

    def _restart_unity(self):
        """Restart Unity Play Mode, or reconfigure the scene in a player build."""
        try:
            result = self._call(
                pm_srv.RestartSimulation, self.unity_restart_service,
                pm_srv.RestartSimulation.Request())
            if result.success and result.play_mode_restart:
                return ('Unity is restarting Play Mode; wait until it is running '
                        'again before you restart the robot launch.')
            reason = result.message
        except StateServiceError as error:
            reason = str(error)

        try:
            result = self._call(
                pm_srv.EmptyWithSuccess, self.unity_configure_service,
                pm_srv.EmptyWithSuccess.Request())
            if result.success:
                return (f'Unity could not restart Play Mode ({reason}); the running '
                        'scene was reconfigured instead.')
        except StateServiceError as error:
            reason = f'{reason}; {error}'
        return f'Unity could not be restarted or reconfigured ({reason}); restart it by hand.'

    def _restore(self, path, state):
        unity_joints = state['unity_joints']
        saved_joint_states = state['joint_states']
        smarpod = state.get('smarpod')

        # Everything that can fail is checked before a controller is touched.
        current = self._call(
            pm_srv.GetSimRobotState, self.unity_get_service,
            pm_srv.GetSimRobotState.Request())
        if not current.success:
            raise StateServiceError(f'Unity could not read the state: {current.message}')
        missing = sorted(set(unity_joints) - set(current.joint_names))
        if missing:
            raise StateServiceError(
                'Unity does not have these saved joints: ' + ', '.join(missing))
        if smarpod is not None and not current.has_smarpod:
            raise StateServiceError('The state has a SmarPod, but Unity has none.')

        notes = []
        if smarpod is None and current.has_smarpod:
            notes.append('the state has no SmarPod pose; the SmarPod was not moved')

        controllers = self._list_controllers()
        motion = [
            controller for controller in controllers.values()
            if controller.state == 'active' and controller.type in MOTION_CONTROLLERS
        ]
        deactivate = [controller.name for controller in motion]
        reactivate = list(deactivate)
        virtual_active = SMARPOD_VIRTUAL_CONTROLLER in deactivate
        if smarpod is not None and virtual_active:
            # The virtual pivot zero is the pose at selection time and cannot be
            # restored; continue with the default controller.
            reactivate.remove(SMARPOD_VIRTUAL_CONTROLLER)
            if SMARPOD_DEFAULT_CONTROLLER in controllers:
                reactivate.append(SMARPOD_DEFAULT_CONTROLLER)
        trajectory_joints = [
            interface.split('/')[0]
            for controller in motion if controller.type == TRAJECTORY_CONTROLLER
            for interface in controller.claimed_interfaces
        ]
        forward_commands = {
            controller.name: [
                interface.split('/')[0] for interface in controller.claimed_interfaces]
            for controller in motion if controller.type in FORWARD_CONTROLLERS
        }

        self._switch_controllers(deactivate=deactivate)
        failure = None
        try:
            # Let commands sent before the switch arrive in Unity first.
            time.sleep(self.settle_time)
            if smarpod is not None and virtual_active:
                reset = self._call(
                    Trigger, '/smarpod_controller/ResetVirtualPivot', Trigger.Request())
                if not reset.success:
                    raise StateServiceError(f'ResetVirtualPivot failed: {reset.message}')

            request = pm_srv.SetSimRobotState.Request()
            request.joint_names = list(unity_joints)
            request.joint_positions = [float(value) for value in unity_joints.values()]
            request.set_smarpod = smarpod is not None
            if smarpod is not None:
                request.smarpod_pose = [float(value) for value in smarpod['pose']]
            result = self._call(pm_srv.SetSimRobotState, self.unity_set_service, request)
            if not result.success:
                raise StateServiceError(f'Unity could not set the state: {result.message}')

            # The trajectory controllers hold whatever the hardware reports when
            # they are activated, so the new positions must have arrived first.
            deviations = self._wait_for_joint_states(saved_joint_states, trajectory_joints)
            if deviations:
                raise StateServiceError(
                    'The joints did not reach the saved positions: ' + '; '.join(deviations))
        except (StateServiceError, StateFileError) as error:
            failure = error
        finally:
            try:
                self._switch_controllers(activate=reactivate)
            except StateServiceError as error:
                previous = f'{failure}; ' if failure is not None else ''
                raise StateServiceError(
                    f'{previous}the motion controllers '
                    f'{", ".join(reactivate)} could not be reactivated ({error}). '
                    'Check them before moving the robot.') from error
        if failure is not None:
            raise StateServiceError(f'{failure}. The motion controllers were reactivated.')

        # Forward controllers (grippers, UV cart) command mock or Unity joints
        # directly; send them the saved positions so their state matches.
        for name, joints in forward_commands.items():
            if name in reactivate and all(joint in saved_joint_states for joint in joints):
                self._publish_command(name, [saved_joint_states[joint] for joint in joints])

        if smarpod is not None:
            if smarpod.get('virtual_pivot_active') or virtual_active:
                notes.append(
                    'the SmarPod virtual pivot was not restored; it now uses the default '
                    'controller - select the virtual pivot frame again if needed')
            if 'speed' in smarpod:
                self._optional(notes, 'SmarPod speed', self._restore_smarpod_speed,
                               smarpod['speed'])

        for section, writer in (
                ('pneumatics', self._restore_pneumatics),
                ('nozzles', self._restore_nozzles),
                ('lights', self._restore_lights),
                ('uv', self._restore_uv)):
            if section in state:
                self._optional(notes, section, writer, state[section])
            else:
                notes.append(f'{section} are not in the state file')

        deviations = self._wait_for_joint_states(saved_joint_states)
        if deviations:
            raise StateServiceError(
                'The state was applied, but these joints differ from the saved state: '
                + '; '.join(deviations))

        message = f"Restored the robot state from '{path}'."
        if notes:
            message += ' Note: ' + '; '.join(notes) + '.'
        return message

    @staticmethod
    def _optional(notes, label, function, *args):
        try:
            function(*args)
        except StateServiceError as error:
            notes.append(f'{label} not restored ({error})')

    def _restore_smarpod_speed(self, speed):
        if SmarpodSetSpeed is None:
            raise StateServiceError('smarpod_interfaces is not installed')
        result = self._call(
            SmarpodSetSpeed, '/smarpod_controller/SetSpeed',
            SmarpodSetSpeed.Request(speed=float(speed)))
        if not result.success:
            raise StateServiceError('SetSpeed failed')

    def _restore_pneumatics(self, values):
        for name, position in values.items():
            # Returns once the cylinder reports the position; Unity already
            # placed it there, so nothing moves.
            self._call(
                pm_srv.PneumaticSetPosition,
                f'/pm_pneumatic_controller/{_topic_segment(name)}/SetPosition',
                pm_srv.PneumaticSetPosition.Request(command=int(position)))

    def _restore_nozzles(self, values):
        for name, position in values.items():
            self._call(
                pm_srv.NozzleSetPosition, f'/pm_nozzle_controller/{name}/SetPosition',
                pm_srv.NozzleSetPosition.Request(command=int(position)))

    def _restore_lights(self, lights):
        self._call(
            pm_srv.CoaxLightSetState, '/pm_lights_controller/CoaxLight/SetState',
            pm_srv.CoaxLightSetState.Request(turn_on=lights['coax_light']))
        self._call(
            pm_srv.RingLightSetState, '/pm_lights_controller/RingLight/SetState',
            pm_srv.RingLightSetState.Request(
                turn_on=lights['ring_light'], rgb=lights['ring_light_rgb']))
        self._call(
            pm_srv.Cam2LightSetState, '/pm_lights_controller/Cam2Light/SetState',
            pm_srv.Cam2LightSetState.Request(intensity=lights['camera2_light']))

    def _restore_uv(self, uv):
        self._call(
            pm_srv.UVSetPower, '/pm_uv_controller/Hoenle_UV/SetPower',
            pm_srv.UVSetPower.Request(power=uv['power']))
        self._call(
            pm_srv.UVSetTime, '/pm_uv_controller/Hoenle_UV/SetTime',
            pm_srv.UVSetTime.Request(time=[float(value) for value in uv['time']]))
        self._call(
            pm_srv.UVSetOnOff, '/pm_uv_controller/Hoenle_UV/SetOnOff',
            pm_srv.UVSetOnOff.Request(turn_on=uv['on_off']))

    # ------------------------------------------------------------------
    # Simulation guard
    # ------------------------------------------------------------------

    def _ensure_simulation(self):
        """Fail unless the robot runs against the Unity simulation."""
        if not self._client(pm_srv.GetSimRobotState, self.unity_get_service).wait_for_service(
                timeout_sec=2.0):
            raise StateServiceError(
                f"the Unity simulation is not running (service '{self.unity_get_service}' "
                'is unavailable). Robot states can only be saved and loaded in the '
                'Unity simulation.')

        robot_description = self._wait_for(lambda: self._robot_description, 2.0)
        if robot_description is None:
            raise StateServiceError('no robot_description received on /robot_description.')
        problems = simulated_hardware_problems(robot_description)

        components = self._call(
            ListHardwareComponents, f'{self.controller_manager}/list_hardware_components',
            ListHardwareComponents.Request())
        problems.extend(
            f"hardware component '{component.name}' is the real device "
            f"'{component.class_type}'."
            for component in components.component
            if component.class_type in REAL_HARDWARE_PLUGINS)
        if problems:
            raise StateServiceError(
                'the robot is not running against the Unity simulation: '
                + ' '.join(problems))

    def _wait_for_configuration(self):
        message = self._wait_for(lambda: self._loaded_configuration, 2.0)
        if message is None:
            raise StateServiceError(
                f"no configuration received on '{self.configuration_node}/"
                "current_pm_robot_configuration'. Is pm_robot_primitive_skills running?")
        if message.use_real_config:
            raise StateServiceError('the loaded configuration is the real-hardware configuration.')
        return message

    # ------------------------------------------------------------------
    # Controllers
    # ------------------------------------------------------------------

    def _list_controllers(self):
        result = self._call(
            ListControllers, f'{self.controller_manager}/list_controllers',
            ListControllers.Request())
        return {controller.name: controller for controller in result.controller}

    def _switch_controllers(self, activate=(), deactivate=()):
        if not activate and not deactivate:
            return
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 5
        result = self._call(
            SwitchController, f'{self.controller_manager}/switch_controller', request)
        if not result.ok:
            actions = [f'activate {name}' for name in activate]
            actions += [f'deactivate {name}' for name in deactivate]
            raise StateServiceError(
                f"the controller manager could not {', '.join(actions)}")

    def _controller_list_parameter(self, controller, parameter):
        result = self._call(
            GetParameters, f'{controller}/get_parameters',
            GetParameters.Request(names=[parameter]))
        if not result.values:
            raise StateServiceError(f"'{controller}' has no parameter '{parameter}'")
        return list(result.values[0].string_array_value)

    def _publish_command(self, controller, values):
        topic = f'/{controller}/commands'
        publisher = self._command_publishers.get(topic)
        if publisher is None:
            publisher = self.create_publisher(Float64MultiArray, topic, 10)
            self._command_publishers[topic] = publisher
        self._wait_for(lambda: publisher.get_subscription_count() > 0 or None, 2.0)
        publisher.publish(Float64MultiArray(data=[float(value) for value in values]))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _fresh_joint_states(self):
        """Joint states received after this call started."""
        requested = time.monotonic()

        def fresh():
            with self._data_lock:
                if self._joint_states_received > requested:
                    return dict(self._joint_states)
            return None

        states = self._wait_for(fresh, 2.0)
        if states is None:
            raise StateServiceError('no /joint_states received.')
        return states

    def _wait_for_joint_states(self, expected, names=None):
        """Wait until /joint_states matches; return the remaining deviations."""
        deadline = time.monotonic() + self.state_timeout
        while True:
            deviations = joint_deviations(
                expected, self._fresh_joint_states(), self.joint_tolerance, names)
            if not deviations or time.monotonic() > deadline:
                return deviations
            time.sleep(0.05)

    def _wait_for(self, read, timeout):
        """Poll ``read`` until it returns something other than None."""
        deadline = time.monotonic() + timeout
        while True:
            value = read()
            if value is not None or time.monotonic() > deadline:
                return value
            time.sleep(0.02)

    def _client(self, service_type, name):
        key = (service_type, name)
        if key not in self._service_clients:
            self._service_clients[key] = self.create_client(
                service_type, name, callback_group=self._client_group)
        return self._service_clients[key]

    def _call(self, service_type, name, request, timeout=None):
        """Call a service and wait for the response, or raise StateServiceError."""
        client = self._client(service_type, name)
        if not client.wait_for_service(timeout_sec=2.0):
            raise StateServiceError(f"service '{name}' is not available")
        timeout = self.service_timeout if timeout is None else timeout
        future = client.call_async(request)
        done = Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            raise StateServiceError(f"service '{name}' did not answer within {timeout:.0f} s")
        if future.exception() is not None:
            raise StateServiceError(f"service '{name}' failed: {future.exception()}")
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = SimStateNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
