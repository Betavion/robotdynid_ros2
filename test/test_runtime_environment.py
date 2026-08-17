"""Tests for the robotdynid cmeel runtime bootstrap."""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

from robotdynid_ros2 import runtime_environment


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _fake_cmeel_prefix(tmp_path: Path) -> tuple[Path, Path, Path]:
    python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    purelib = tmp_path / 'lib' / python_version / 'site-packages'
    prefix = purelib / 'cmeel.prefix'
    cmeel_python = prefix / 'lib' / python_version / 'site-packages'
    (cmeel_python / 'pinocchio').mkdir(parents=True)
    return purelib, prefix, cmeel_python


def test_reexec_prepends_venv_cmeel_before_ros_paths(tmp_path, monkeypatch):
    """The child process must resolve pip Pinocchio before the ROS overlay."""
    purelib, prefix, cmeel_python = _fake_cmeel_prefix(tmp_path)
    monkeypatch.setattr(
        runtime_environment.sysconfig,
        'get_path',
        lambda name: str(purelib),
    )
    monkeypatch.setattr(sys, 'argv', ['/tmp/robotdynid-gui', '--help'])
    monkeypatch.setenv('PYTHONPATH', '/opt/ros/jazzy/python')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/opt/ros/jazzy/lib')
    monkeypatch.delenv(runtime_environment._RUNTIME_MARKER, raising=False)
    captured = {}

    def fake_execve(executable, argv, environment):
        captured.update(
            executable=executable,
            argv=argv,
            environment=environment,
        )
        raise RuntimeError('re-exec intercepted')

    monkeypatch.setattr(os, 'execve', fake_execve)

    with pytest.raises(RuntimeError, match='re-exec intercepted'):
        runtime_environment.reexec_with_venv_cmeel()

    environment = captured['environment']
    assert environment['PYTHONPATH'].split(os.pathsep)[:2] == [
        str(purelib),
        str(cmeel_python),
    ]
    assert environment['LD_LIBRARY_PATH'].split(os.pathsep)[0] == str(
        prefix / 'lib'
    )
    assert environment[runtime_environment._RUNTIME_MARKER] == str(
        prefix.resolve()
    )
    assert captured['argv'][1:] == [
        '/tmp/robotdynid-gui',
        '--help',
    ]


def test_reexec_is_skipped_after_runtime_is_prepared(tmp_path, monkeypatch):
    """The marker prevents a re-exec loop in the prepared child process."""
    purelib, prefix, cmeel_python = _fake_cmeel_prefix(tmp_path)
    cmeel_lib = prefix / 'lib'
    monkeypatch.setattr(
        runtime_environment.sysconfig,
        'get_path',
        lambda name: str(purelib),
    )
    monkeypatch.setenv(
        runtime_environment._RUNTIME_MARKER,
        str(prefix.resolve()),
    )
    monkeypatch.setenv(
        'PYTHONPATH',
        os.pathsep.join((str(purelib), str(cmeel_python))),
    )
    monkeypatch.setenv('LD_LIBRARY_PATH', str(cmeel_lib))
    monkeypatch.setattr(
        os,
        'execve',
        lambda *_args: pytest.fail('unexpected re-exec'),
    )

    runtime_environment.reexec_with_venv_cmeel()


def test_stale_marker_reexecutes_when_runtime_paths_were_replaced(
    tmp_path,
    monkeypatch,
):
    """A marker alone must not hide a rewritten ROS-only environment."""
    purelib, prefix, _ = _fake_cmeel_prefix(tmp_path)
    monkeypatch.setattr(
        runtime_environment.sysconfig,
        'get_path',
        lambda name: str(purelib),
    )
    monkeypatch.setenv(
        runtime_environment._RUNTIME_MARKER,
        str(prefix.resolve()),
    )
    monkeypatch.setenv('PYTHONPATH', '/opt/ros/jazzy/python')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/opt/ros/jazzy/lib')
    monkeypatch.setattr(sys, 'argv', ['/tmp/robotdynid-gui'])

    def fake_execve(*_args):
        raise RuntimeError('re-exec')

    monkeypatch.setattr(os, 'execve', fake_execve)

    with pytest.raises(RuntimeError, match='re-exec'):
        runtime_environment.reexec_with_venv_cmeel()


def test_missing_cmeel_runtime_is_a_noop(tmp_path, monkeypatch):
    """A system-only ROS installation keeps using its packaged Pinocchio."""
    purelib = tmp_path / 'site-packages'
    purelib.mkdir()
    monkeypatch.setattr(
        runtime_environment.sysconfig,
        'get_path',
        lambda name: str(purelib),
    )
    monkeypatch.setattr(
        os,
        'execve',
        lambda *_args: pytest.fail('unexpected re-exec'),
    )

    runtime_environment.reexec_with_venv_cmeel()


def test_prepend_paths_deduplicates_and_preserves_other_entries():
    """Runtime paths appear once while unrelated paths retain their order."""
    first = Path('/venv/site-packages')
    second = Path('/venv/cmeel/site-packages')
    existing = os.pathsep.join(('/opt/ros/python', str(first), '/workspace'))

    result = runtime_environment._prepend_paths(existing, first, second)

    assert result.split(os.pathsep) == [
        str(first),
        str(second),
        '/opt/ros/python',
        '/workspace',
    ]


def test_all_installed_scripts_bootstrap_before_business_imports():
    """Every CMake-installed entry point must prepare cmeel first."""
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    install_block = re.search(
        r'install\(\s+PROGRAMS\s+(.*?)\s+DESTINATION lib/\$\{PROJECT_NAME\}',
        cmake,
        flags=re.DOTALL,
    )
    assert install_block is not None
    script_paths = [
        PACKAGE_ROOT / item.strip()
        for item in install_block.group(1).splitlines()
        if item.strip()
    ]
    assert len(script_paths) == 12

    for path in script_paths:
        source = path.read_text(encoding='utf-8')
        bootstrap = source.index('reexec_with_venv_cmeel()')
        business_imports = [
            match.start()
            for match in re.finditer(
                r'^from robotdynid_ros2\.(?!runtime_environment)',
                source,
                flags=re.MULTILINE,
            )
        ]
        assert business_imports, path
        assert bootstrap < min(business_imports), path


def test_active_cmeel_runtime_imports_pinocchio_with_rclpy():
    """When available, pip Pinocchio and ROS Python coexist in one child."""
    purelib = Path(runtime_environment.sysconfig.get_path('purelib'))
    python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    prefix = purelib / 'cmeel.prefix'
    cmeel_python = prefix / 'lib' / python_version / 'site-packages'
    cmeel_lib = prefix / 'lib'
    if not (cmeel_python / 'pinocchio').is_dir():
        pytest.skip('pip Pinocchio runtime is not installed')

    environment = os.environ.copy()
    environment['PYTHONPATH'] = runtime_environment._prepend_paths(
        environment.get('PYTHONPATH', ''),
        purelib,
        cmeel_python,
    )
    environment['LD_LIBRARY_PATH'] = runtime_environment._prepend_paths(
        environment.get('LD_LIBRARY_PATH', ''),
        cmeel_lib,
    )
    result = subprocess.run(
        [
            sys.executable,
            '-c',
            'import pinocchio; import rclpy; '
            'assert pinocchio.__version__ == "3.9.0"',
        ],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
