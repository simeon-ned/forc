"""Simulation helpers (MuJoCo wrappers grow here)."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def load_model(xml_path: str | Path) -> tuple[Any, Any]:
    """Load an MJCF/URDF path into ``(mjModel, mjData)``.

    Requires the optional ``forc[sim]`` extra (``mujoco``).
    """
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            "mujoco is required for forc.sim — install with: pip install 'forc[sim]'"
        ) from exc

    path = Path(xml_path)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    return model, data
