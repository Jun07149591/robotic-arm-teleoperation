from __future__ import annotations

from pathlib import Path


def next_episode_dir(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    episodes = sorted([p for p in root.iterdir() if p.is_dir() and p.name.startswith("episode_")])
    if not episodes:
        return root / "episode_000001"
    last = episodes[-1].name.split("_")[-1]
    try:
        idx = int(last)
    except ValueError:
        idx = len(episodes)
    return root / f"episode_{idx + 1:06d}"

