"""Prepare the active venv's cmeel runtime before importing Pinocchio."""

from __future__ import annotations

import os
import sys
import sysconfig
from pathlib import Path


_RUNTIME_MARKER = 'ROBOTDYNID_CMEEL_PREFIX'


def _prepend_paths(existing: str, *paths: Path) -> str:
    """Prepend unique filesystem paths to a path-list environment value."""
    requested = [str(path) for path in paths]
    retained = [
        item
        for item in existing.split(os.pathsep)
        if item and item not in requested
    ]
    return os.pathsep.join([*requested, *retained])


def _starts_with_paths(existing: str, *paths: Path) -> bool:
    """Return whether a path-list starts with all requested paths."""
    requested = [str(path) for path in paths]
    actual = [item for item in existing.split(os.pathsep) if item]
    return actual[:len(requested)] == requested


def reexec_with_venv_cmeel() -> None:
    """
    Re-exec once with the pip Pinocchio runtime ahead of ROS libraries.

    ROS Jazzy contributes its Python and shared-library paths through
    ``PYTHONPATH`` and ``LD_LIBRARY_PATH``.  In a NumPy 2 venv those paths
    would otherwise select the ROS Pinocchio build compiled against NumPy 1.
    The ``pin`` wheel keeps its Python modules and libraries under
    ``cmeel.prefix``; both locations must precede the ROS overlay before the
    dynamic loader starts importing Pinocchio.
    """
    python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    purelib = Path(sysconfig.get_path('purelib'))
    cmeel_prefix = purelib / 'cmeel.prefix'
    cmeel_python = cmeel_prefix / 'lib' / python_version / 'site-packages'
    cmeel_lib = cmeel_prefix / 'lib'
    pinocchio_package = cmeel_python / 'pinocchio'

    if not pinocchio_package.is_dir() or not cmeel_lib.is_dir():
        return

    marker = str(cmeel_prefix.resolve())
    runtime_is_prepared = all(
        (
            os.environ.get(_RUNTIME_MARKER) == marker,
            _starts_with_paths(
                os.environ.get('PYTHONPATH', ''),
                purelib,
                cmeel_python,
            ),
            _starts_with_paths(
                os.environ.get('LD_LIBRARY_PATH', ''),
                cmeel_lib,
            ),
        )
    )
    if runtime_is_prepared:
        return

    environment = os.environ.copy()
    environment['PYTHONPATH'] = _prepend_paths(
        environment.get('PYTHONPATH', ''),
        purelib,
        cmeel_python,
    )
    environment['LD_LIBRARY_PATH'] = _prepend_paths(
        environment.get('LD_LIBRARY_PATH', ''),
        cmeel_lib,
    )
    environment[_RUNTIME_MARKER] = marker
    argv = [sys.executable, str(Path(sys.argv[0]).resolve()), *sys.argv[1:]]
    os.execve(sys.executable, argv, environment)
