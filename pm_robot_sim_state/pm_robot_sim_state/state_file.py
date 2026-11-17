"""
File format of saved PM robot simulation states.

One YAML file holds everything needed to restore a state::

    format_version: 1
    created: '2026-11-17T14:03:12'
    description: ''
    pm_robot_configuration:   # with_*/use_* keys of the loaded bringup config
      ...
    robot_state:
      unity_joints: {<Unity link name>: <m or rad>}   # restored in Unity
      joint_states: {<ROS joint name>: <value>}       # used for checks
      smarpod: {pose: [x, y, z, a, b, c], speed: <m/s>,
                virtual_pivot_active: <bool>}         # only with the SmarPod
      pneumatics: {<cylinder>: 1 | -1}                # forward / backward
      nozzles: {<nozzle>: -1 | 0 | 1}                 # vacuum / off / air
      lights: {coax_light: <bool>, ring_light: [<bool> x4],
               ring_light_rgb: [<0..255> x3], camera2_light: <0..255>}
      uv: {on_off: [<bool> x4], power: [<int> x4], time: [<s> x4]}

Everything below ``robot_state`` except ``unity_joints`` and
``joint_states`` is optional; a missing section is not restored.
"""

import math
import os
from pathlib import Path
import tempfile
from typing import Any

import yaml


FORMAT_VERSION = 1
DEFAULT_STATE_DIRECTORY = '~/pm_robot_states'

# Bringup properties that select equipment. Everything else (catalogs of
# available tools, counters such as adhesive_test_point_station.current_point,
# unity_parent names) does not change which joints and parts exist.
CONFIGURATION_KEY_PREFIXES = ('with_', 'use_')


class StateFileError(ValueError):
    """A state file is missing, malformed or of an unsupported version."""


def resolve_state_path(file_path: str, state_directory: str) -> Path:
    """Return the absolute state file path for a service request."""
    name = file_path.strip()
    if not name:
        raise StateFileError('file_path must not be empty.')
    path = Path(name).expanduser()
    if not path.is_absolute():
        path = Path(state_directory).expanduser() / path
    if path.suffix not in ('.yaml', '.yml'):
        path = path.with_name(path.name + '.yaml')
    return path.resolve()


def select_configuration(configuration: dict[str, Any]) -> dict[str, Any]:
    """Keep only the scalar with_*/use_* properties and their parents."""
    selected = {}
    for key, value in configuration.items():
        if isinstance(value, dict):
            child = select_configuration(value)
            if child:
                selected[key] = child
        elif (isinstance(key, str) and key.startswith(CONFIGURATION_KEY_PREFIXES)
              and not isinstance(value, (list, dict))):
            selected[key] = value
    return selected


def build_document(configuration: dict[str, Any], robot_state: dict[str, Any],
                   description: str, created: str) -> dict[str, Any]:
    """Assemble a state document; validates it before returning."""
    document = {
        'format_version': FORMAT_VERSION,
        'created': created,
        'description': description,
        'pm_robot_configuration': configuration,
        'robot_state': robot_state,
    }
    validate_document(document)
    return document


def write_document(path: Path, document: dict[str, Any], overwrite: bool) -> None:
    """Write a state file atomically, so a failed save keeps the old file."""
    if path.exists() and not overwrite:
        raise StateFileError(
            f"'{path}' already exists. Set overwrite to replace it.")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8', dir=path.parent,
                prefix=f'.{path.name}.', delete=False) as temporary:
            temporary_path = Path(temporary.name)
            yaml.safe_dump(document, temporary, sort_keys=False,
                           default_flow_style=None)
            temporary.flush()
            os.fsync(temporary.fileno())
        temporary_path.chmod(0o644)
        os.replace(temporary_path, path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def read_document(path: Path) -> dict[str, Any]:
    """Read and validate a state file."""
    if not path.is_file():
        raise StateFileError(f"State file '{path}' does not exist.")
    try:
        with path.open(encoding='utf-8') as state_file:
            document = yaml.safe_load(state_file)
    except yaml.YAMLError as error:
        raise StateFileError(f"'{path}' is not valid YAML: {error}") from error
    try:
        validate_document(document)
    except StateFileError as error:
        raise StateFileError(f"'{path}': {error}") from error
    return document


def validate_document(document: Any) -> None:
    """Raise StateFileError unless ``document`` is a supported state."""
    if not isinstance(document, dict):
        raise StateFileError('a state file must be a YAML mapping.')
    version = document.get('format_version')
    if version != FORMAT_VERSION:
        raise StateFileError(
            f'unsupported format_version {version!r} '
            f'(supported: {FORMAT_VERSION}).')

    configuration = document.get('pm_robot_configuration')
    if not isinstance(configuration, dict) or not configuration:
        raise StateFileError('pm_robot_configuration must be a nonempty mapping.')

    state = document.get('robot_state')
    if not isinstance(state, dict):
        raise StateFileError('robot_state must be a mapping.')
    for key in ('unity_joints', 'joint_states'):
        _check_number_map(state.get(key), f'robot_state.{key}')
    if not state['unity_joints']:
        raise StateFileError('robot_state.unity_joints must not be empty.')

    smarpod = state.get('smarpod')
    if smarpod is not None:
        if not isinstance(smarpod, dict):
            raise StateFileError('robot_state.smarpod must be a mapping.')
        _check_number_list(smarpod.get('pose'), 6, 'robot_state.smarpod.pose')
        if 'speed' in smarpod and not _is_number(smarpod['speed']):
            raise StateFileError('robot_state.smarpod.speed must be a number.')

    for key, allowed in (('pneumatics', (-1, 1)), ('nozzles', (-1, 0, 1))):
        values = state.get(key)
        if values is None:
            continue
        if not isinstance(values, dict):
            raise StateFileError(f'robot_state.{key} must be a mapping.')
        for name, value in values.items():
            if isinstance(value, bool) or value not in allowed:
                raise StateFileError(
                    f'robot_state.{key}.{name} must be one of {allowed}.')

    lights = state.get('lights')
    if lights is not None:
        if not isinstance(lights, dict):
            raise StateFileError('robot_state.lights must be a mapping.')
        _check_bool_list(lights.get('ring_light'), 4, 'robot_state.lights.ring_light')
        _check_byte_list(lights.get('ring_light_rgb'), 3, 'robot_state.lights.ring_light_rgb')
        if not isinstance(lights.get('coax_light'), bool):
            raise StateFileError('robot_state.lights.coax_light must be true or false.')
        _check_byte(lights.get('camera2_light'), 'robot_state.lights.camera2_light')

    uv = state.get('uv')
    if uv is not None:
        if not isinstance(uv, dict):
            raise StateFileError('robot_state.uv must be a mapping.')
        _check_bool_list(uv.get('on_off'), 4, 'robot_state.uv.on_off')
        _check_number_list(uv.get('power'), 4, 'robot_state.uv.power', integer=True)
        _check_number_list(uv.get('time'), 4, 'robot_state.uv.time')


def _is_number(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _check_number_map(values: Any, label: str) -> None:
    if not isinstance(values, dict):
        raise StateFileError(f'{label} must be a mapping of names to numbers.')
    for name, value in values.items():
        if not isinstance(name, str) or not _is_number(value):
            raise StateFileError(f'{label}.{name} must be a finite number.')


def _check_number_list(values: Any, length: int, label: str,
                       integer: bool = False) -> None:
    if (not isinstance(values, list) or len(values) != length
            or not all(_is_number(value) for value in values)
            or (integer and not all(isinstance(value, int) for value in values))):
        kind = 'integers' if integer else 'numbers'
        raise StateFileError(f'{label} must be a list of {length} {kind}.')


def _check_bool_list(values: Any, length: int, label: str) -> None:
    if (not isinstance(values, list) or len(values) != length
            or not all(isinstance(value, bool) for value in values)):
        raise StateFileError(f'{label} must be a list of {length} booleans.')


def _check_byte(value: Any, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255:
        raise StateFileError(f'{label} must be an integer from 0 to 255.')


def _check_byte_list(values: Any, length: int, label: str) -> None:
    if not isinstance(values, list) or len(values) != length:
        raise StateFileError(f'{label} must be a list of {length} integers.')
    for value in values:
        _check_byte(value, label)
