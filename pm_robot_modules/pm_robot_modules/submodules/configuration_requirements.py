"""Helpers for partial PM robot configuration requirement files."""

import copy
from pathlib import Path
from typing import Any


DEFAULT_CONFIGURATION_FILE_NAME = 'pm_robot_configuration.yaml'


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
