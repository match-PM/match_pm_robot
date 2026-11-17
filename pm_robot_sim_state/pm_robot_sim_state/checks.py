"""Checks used by the state services that do not need a running robot."""

from typing import Iterable, Mapping, Optional
from urllib.parse import urlparse
import xml.etree.ElementTree as ElementTree


PM_SYSTEM_PLUGIN = 'pm_hardware_interface/PMSystem'
# ros2_control plugins that drive physical devices. The state services must
# never act while one of them is loaded.
REAL_HARDWARE_PLUGINS = frozenset({
    'smaract_hexapod_hw_interface/SmarPodSystem',
})
LOCAL_HOSTS = frozenset({'localhost', '127.0.0.1', '::1'})


def simulated_hardware_problems(robot_description: str) -> list[str]:
    """
    Return why ``robot_description`` is not the Unity simulation.

    The robot runs against Unity when PMSystem talks to the OPC UA server on
    this computer (started by Unity) and no real-device plugin is loaded.
    """
    try:
        root = ElementTree.fromstring(robot_description)
    except ElementTree.ParseError as error:
        return [f'robot_description is not valid XML: {error}']

    problems = []
    found_pm_system = False
    for system in root.iter('ros2_control'):
        name = system.get('name', '<unnamed>')
        hardware = system.find('hardware')
        if hardware is None:
            continue
        plugin = (hardware.findtext('plugin') or '').strip()
        if plugin in REAL_HARDWARE_PLUGINS:
            problems.append(
                f"ros2_control system '{name}' uses the real hardware plugin "
                f"'{plugin}'.")
        if plugin != PM_SYSTEM_PLUGIN:
            continue
        found_pm_system = True
        endpoint = ''
        for parameter in hardware.findall('param'):
            if parameter.get('name') == 'opcua_endpoint':
                endpoint = (parameter.text or '').strip()
        host = urlparse(endpoint).hostname if endpoint else None
        if host not in LOCAL_HOSTS:
            problems.append(
                f"ros2_control system '{name}' connects to the OPC UA server "
                f"'{endpoint or '<unset>'}', not to the Unity simulation on "
                'this computer.')
    if not found_pm_system:
        problems.append(
            f"robot_description has no '{PM_SYSTEM_PLUGIN}' hardware.")
    return problems


def joint_deviations(expected: Mapping[str, float], actual: Mapping[str, float],
                     tolerance: float,
                     names: Optional[Iterable[str]] = None) -> list[str]:
    """
    List joints whose ``actual`` value is missing or off by > tolerance.

    Only joints in ``expected`` are compared; ``names`` narrows them further.
    """
    selected = expected if names is None else [
        name for name in names if name in expected]
    deviations = []
    for name in selected:
        if name not in actual:
            deviations.append(f'{name}: missing from /joint_states')
            continue
        difference = abs(actual[name] - expected[name])
        if difference > tolerance:
            deviations.append(
                f'{name}: expected {expected[name]:.6f}, '
                f'is {actual[name]:.6f}')
    return deviations
