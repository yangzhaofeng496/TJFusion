"""Fusion Docker package."""

from __future__ import annotations

from pathlib import Path


def _read_version_from_config() -> str:
    config_path = Path(__file__).with_name("version.conf")
    try:
        content = config_path.read_text(encoding="utf-8")
    except OSError:
        return "0.0.2"

    for raw_line in content.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, sep, value = line.partition("=")
        if sep and key.strip() == "version":
            version = value.strip()
            if version:
                return version
    return "0.0.2"


__version__ = _read_version_from_config()
