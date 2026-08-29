"""Plotting helpers for practices and take-homes."""

from __future__ import annotations

from typing import Sequence

import numpy as np


def states(
    t: Sequence[float] | np.ndarray,
    q: np.ndarray,
    *,
    labels: Sequence[str] | None = None,
    title: str | None = None,
):
    """Plot joint trajectories ``q`` over time ``t``.

    Parameters
    ----------
    t:
        Time vector, shape ``(T,)``.
    q:
        States, shape ``(T, n)`` or ``(n, T)``.
    """
    import matplotlib.pyplot as plt

    q = np.asarray(q)
    t = np.asarray(t)
    if q.ndim == 1:
        q = q[:, None]
    if q.shape[0] != t.shape[0] and q.shape[1] == t.shape[0]:
        q = q.T

    fig, ax = plt.subplots(figsize=(7, 3.5))
    for i in range(q.shape[1]):
        name = labels[i] if labels and i < len(labels) else f"q{i}"
        ax.plot(t, q[:, i], label=name)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("state")
    if title:
        ax.set_title(title)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig, ax
