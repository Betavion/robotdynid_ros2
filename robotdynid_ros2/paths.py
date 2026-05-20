"""Path helpers for timestamped identification runs."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path


def timestamped_run_dir(root: str | Path = "runs", run_name: str | None = None) -> Path:
    """Return a unique run directory under ``root``.

    If ``run_name`` is omitted, the current local timestamp is used. Existing
    directories get a numeric suffix so repeated launches do not overwrite data.
    """

    stem = run_name or datetime.now().strftime("%Y%m%d_%H%M%S")
    base = Path(root).expanduser() / stem
    candidate = base
    suffix = 1
    while candidate.exists():
        candidate = Path(f"{base}_{suffix:02d}")
        suffix += 1
    return candidate


def resolve_optional_path(value: str | Path | None) -> Path | None:
    if value is None or str(value) == "":
        return None
    return Path(value).expanduser()
