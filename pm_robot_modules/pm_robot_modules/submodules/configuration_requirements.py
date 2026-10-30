"""Helpers for partial PM robot configuration requirement files."""

import copy
import os
from pathlib import Path
import tempfile
from typing import Any

import yaml


DEFAULT_CONFIGURATION_FILE_NAME = 'pm_robot_configuration.yaml'


def read_configuration(path: Path) -> dict[str, Any]:
    """Read a nonempty YAML configuration mapping."""
    with path.open(encoding='utf-8') as config_file:
        configuration = yaml.safe_load(config_file)
    if not isinstance(configuration, dict) or not configuration:
        raise ValueError(f"'{path}' must contain at least one YAML property.")
    return configuration


class RobotConfigurationState:
    """Keep the robot's launch snapshot separate from its editable file."""

    def __init__(self, path: str, loaded_configuration: dict | None):
        self.path = Path(path)
        self.loaded = copy.deepcopy(loaded_configuration)

    def check(self, required: dict) -> list[str]:
        if not isinstance(self.loaded, dict) or not self.loaded:
            raise RuntimeError(
                'No launch-time configuration snapshot is available. Start the '
                'node through the robot bringup launch file or supply the '
                'loaded_pm_robot_configuration_yaml parameter from bringup.'
            )
        disk = read_configuration(self.path)
        differences = []
        for label, current in [('Loaded robot', self.loaded), ('On-disk', disk)]:
            differences.extend(
                f'{label}: {difference}'
                for difference in compare_configuration_subset(current, required)
            )
        # Check both directions to detect added/deleted properties as well.
        if (compare_configuration_subset(disk, self.loaded)
                or compare_configuration_subset(self.loaded, disk)):
            differences.append(
                'The on-disk configuration differs from the loaded robot '
                'configuration. Restart the robot to load the file.'
            )
        return differences

    def update(self, required: dict) -> bool:
        """Save only supplied fields; return whether a restart is required."""
        current = read_configuration(self.path)
        merged = merge_configuration_subset(current, required)
        if merged != current:
            # Resolve symlinks so a symlink install still updates the source file.
            destination = self.path.resolve(strict=True)
            temporary_path = None
            try:
                with tempfile.NamedTemporaryFile(
                        mode='w', encoding='utf-8', dir=destination.parent,
                        delete=False) as temporary:
                    temporary_path = Path(temporary.name)
                    yaml.safe_dump(merged, temporary, sort_keys=False)
                    temporary.flush()
                    os.fsync(temporary.fileno())
                temporary_path.chmod(destination.stat().st_mode & 0o777)
                os.replace(temporary_path, destination)
            finally:
                if temporary_path is not None:
                    temporary_path.unlink(missing_ok=True)
        return self.loaded is None or bool(
            compare_configuration_subset(merged, self.loaded)
            or compare_configuration_subset(self.loaded, merged)
        )


def merge_configuration_subset(
        current: dict[str, Any], updates: dict[str, Any]) -> dict[str, Any]:
    """Return ``current`` with only properties supplied by ``updates`` set.

    Unknown properties and values with an incompatible type are rejected. The
    input mappings are not modified, which lets callers keep the active
    configuration unchanged when validation fails.
    """
    merged = copy.deepcopy(current)
    errors: list[str] = []

    def merge(target: Any, source: Any, path: str):
        if isinstance(source, dict):
            if not isinstance(target, dict):
                errors.append(
                    f'{path}: expected a value, but the loaded file contains '
                    'a mapping'
                )
                return

            for key, source_child in source.items():
                child_path = f'{path}.{key}' if path else str(key)
                if key not in target:
                    errors.append(
                        f'{child_path}: property does not exist in the active '
                        'bringup configuration'
                    )
                    continue
                target[key] = merge(target[key], source_child, child_path)
            return target

        if type(target) is not type(source):
            errors.append(
                f'{path}: expected {type(target).__name__}, got '
                f'{type(source).__name__}'
            )
            return target
        return copy.deepcopy(source)

    merge(merged, updates, '')
    if errors:
        raise ValueError(
            'The configuration file is not compatible:\n- '
            + '\n- '.join(errors)
        )
    return merged


def resolve_configuration_file(rsap_path: str, file_path_override: str) -> Path:
    """Resolve the requirement file requested by the configuration service."""
    base_path = Path(rsap_path).expanduser()
    override = file_path_override.strip()

    if override:
        override_path = Path(override).expanduser()
        if override_path.is_absolute():
            return override_path
        if not rsap_path.strip():
            raise ValueError(
                'rsap_path is required when file_path_override is relative.'
            )
        return base_path / override_path

    if not rsap_path.strip():
        raise ValueError('rsap_path must not be empty.')
    return base_path / DEFAULT_CONFIGURATION_FILE_NAME


def compare_configuration_subset(
        current: dict[str, Any], required: dict[str, Any]) -> list[str]:
    """Return differences where ``required`` does not match ``current``."""
    differences: list[str] = []

    def compare(current_value: Any, required_value: Any, path: str):
        if isinstance(required_value, dict):
            if not isinstance(current_value, dict):
                differences.append(
                    f'{path}: expected a mapping, current value is '
                    f'{current_value!r}'
                )
                return

            for key, expected_child in required_value.items():
                child_path = f'{path}.{key}' if path else str(key)
                if key not in current_value:
                    differences.append(
                        f'{child_path}: property is missing from the current '
                        'bringup configuration'
                    )
                    continue
                compare(current_value[key], expected_child, child_path)
            return

        values_match = (
            type(current_value) is type(required_value)
            and current_value == required_value
        )
        if not values_match:
            differences.append(
                f'{path}: required {required_value!r}, current '
                f'{current_value!r}'
            )

    compare(current, required, '')
    return differences
