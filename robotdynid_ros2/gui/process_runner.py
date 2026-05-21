"""Qt process wrapper used by the GUI."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QObject, QProcess, Signal

from robotdynid_ros2.gui.command_builder import CommandSpec


@dataclass(frozen=True)
class ProcessRecord:
    label: str
    command: str
    exit_code: int | None = None
    failed_to_start: bool = False


class ProcessRunner(QObject):
    """Run one command at a time and stream output through Qt signals."""

    started = Signal(object)
    output = Signal(str)
    finished = Signal(object)

    def __init__(self, workspace: str | Path | None = None, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._workspace = Path(workspace).expanduser() if workspace else Path.cwd()
        self._process: QProcess | None = None
        self._record: ProcessRecord | None = None

    def is_running(self) -> bool:
        return self._process is not None and self._process.state() != QProcess.ProcessState.NotRunning

    def run(self, spec: CommandSpec) -> bool:
        if self.is_running():
            self.output.emit("A process is already running. Stop it before starting another command.\n")
            return False
        self._process = QProcess(self)
        self._process.setWorkingDirectory(str(self._workspace))
        self._process.setProgram(spec.program)
        self._process.setArguments(list(spec.args))
        self._process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self._process.readyReadStandardOutput.connect(self._on_ready_read)
        self._process.finished.connect(self._on_finished)
        self._process.errorOccurred.connect(self._on_error)
        self._record = ProcessRecord(label=spec.label, command=spec.display())
        self.started.emit(self._record)
        self.output.emit(f"$ {spec.display()}\n")
        self._process.start()
        return True

    def stop(self) -> None:
        if not self.is_running() or self._process is None:
            return
        self.output.emit("Stopping process...\n")
        self._process.terminate()
        if not self._process.waitForFinished(2500):
            self.output.emit("Process did not terminate, killing it.\n")
            self._process.kill()

    def _on_ready_read(self) -> None:
        if self._process is None:
            return
        raw = bytes(self._process.readAllStandardOutput()).decode(errors="replace")
        if raw:
            self.output.emit(raw)

    def _on_error(self) -> None:
        if self._process is None or self._record is None:
            return
        error = self._process.errorString()
        self.output.emit(f"Process error: {error}\n")
        record = ProcessRecord(self._record.label, self._record.command, failed_to_start=True)
        self.finished.emit(record)

    def _on_finished(self, exit_code: int) -> None:
        if self._record is None:
            return
        record = ProcessRecord(self._record.label, self._record.command, exit_code=int(exit_code))
        self.output.emit(f"Process finished with exit code {exit_code}.\n")
        self.finished.emit(record)
        if self._process is not None:
            self._process.deleteLater()
        self._process = None
        self._record = None

