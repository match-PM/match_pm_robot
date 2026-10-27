from pathlib import Path

from pm_robot_modules.submodules.configuration_requirements import (
    DEFAULT_CONFIGURATION_FILE_NAME,
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
