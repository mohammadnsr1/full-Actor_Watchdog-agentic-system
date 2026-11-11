"""The one reader for the platform config.

Rule 10 of the build prompt: topic, frame, joint, action and service names
come from config/platforms/<platform>.yaml, never from code. This module is
how code asks for them. Lookups are strict: a missing or empty value raises
MissingPlatformValue naming the key, so a platform file that forgot a name
fails at the point of use instead of silently falling back to a literal.

Resolution order for the platform name:
    1. the `platform` argument
    2. CELL_PLATFORM environment variable
    3. `platform:` in config/cell.yaml
Resolution for the repo root (the directory holding config/ and ros2_ws/):
    1. the `repo_root` argument
    2. CELL_REPO_ROOT environment variable
    3. CELL_CONFIG_PATH's parent's parent (cell.yaml lives in <root>/config/)
    4. walk up from this file (works from a colcon install inside the repo)
    5. walk up from the current working directory

Python 3.10 compatible; no ROS imports here, so launch files and tests can
use it without a sourced workspace.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Iterable

CELL_PLATFORM_ENV = "CELL_PLATFORM"
CELL_REPO_ROOT_ENV = "CELL_REPO_ROOT"
CELL_CONFIG_PATH_ENV = "CELL_CONFIG_PATH"

PLATFORMS_DIR = Path("config") / "platforms"
CAMERAS_DIR = Path("config") / "cameras"
CELL_YAML = Path("config") / "cell.yaml"

KNOWN_PLATFORMS = ("sim", "hardware")


class MissingPlatformValue(KeyError):
    """A required platform value is absent or empty."""

    def __init__(self, platform: str, keys: Iterable[str], path: Path | None) -> None:
        self.platform = platform
        self.keys = tuple(keys)
        self.path = path
        where = f" ({path})" if path else ""
        super().__init__(
            f"platform {platform!r} has no value for {'.'.join(self.keys)}{where}; "
            "fill it in the platform config (TODO(mohammad) if unknown), never in code"
        )

    def __str__(self) -> str:  # KeyError wraps the message in quotes otherwise
        return self.args[0]


def _is_repo_root(candidate: Path) -> bool:
    return (candidate / "config").is_dir() and (candidate / "ros2_ws").is_dir()


def find_repo_root(explicit: str | os.PathLike[str] | None = None) -> Path:
    if explicit:
        root = Path(explicit).expanduser().resolve()
        if _is_repo_root(root):
            return root
        raise FileNotFoundError(f"{root} is not the repo root (needs config/ and ros2_ws/)")
    env_root = os.environ.get(CELL_REPO_ROOT_ENV, "").strip()
    if env_root:
        return find_repo_root(env_root)
    env_cfg = os.environ.get(CELL_CONFIG_PATH_ENV, "").strip()
    if env_cfg:
        candidate = Path(env_cfg).expanduser().resolve().parent.parent
        if _is_repo_root(candidate):
            return candidate
    for origin in (Path(__file__).resolve(), Path.cwd().resolve()):
        for parent in [origin, *origin.parents]:
            if _is_repo_root(parent):
                return parent
    raise FileNotFoundError(
        "repo root not found: set CELL_REPO_ROOT or CELL_CONFIG_PATH, or run from inside the repo"
    )


def _read_yaml(path: Path) -> dict[str, Any]:
    import yaml

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping at the top level")
    return data


def load_cell_yaml(repo_root: str | os.PathLike[str] | None = None) -> tuple[dict[str, Any], Path]:
    env_cfg = os.environ.get(CELL_CONFIG_PATH_ENV, "").strip()
    path = Path(env_cfg).expanduser() if env_cfg else find_repo_root(repo_root) / CELL_YAML
    if not path.is_file():
        raise FileNotFoundError(f"cell config not found at {path}")
    return _read_yaml(path), path


def resolve_platform_name(
    platform: str | None = None, repo_root: str | os.PathLike[str] | None = None
) -> str:
    name = (platform or os.environ.get(CELL_PLATFORM_ENV, "")).strip()
    if not name:
        cell, path = load_cell_yaml(repo_root)
        name = str(cell.get("platform", "")).strip()
        if not name:
            raise MissingPlatformValue("(cell.yaml)", ("platform",), path)
    if name not in KNOWN_PLATFORMS:
        raise ValueError(f"unknown platform {name!r}; known: {', '.join(KNOWN_PLATFORMS)}")
    return name


class PlatformConfig:
    """Strict accessor over one platform file (and its cameras file)."""

    def __init__(self, name: str, raw: dict[str, Any], path: Path, repo_root: Path) -> None:
        self.name = name
        self.raw = raw
        self.path = path
        self.repo_root = repo_root
        self._cameras: dict[str, Any] | None = None
        declared = str(raw.get("platform", "")).strip()
        if declared and declared != name:
            raise ValueError(f"{path} declares platform {declared!r} but was loaded as {name!r}")

    # -- generic -----------------------------------------------------------

    def get(self, *keys: str) -> Any:
        """Strict nested lookup. Empty strings, None and empty lists are 'missing'."""
        node: Any = self.raw
        for key in keys:
            if not isinstance(node, dict) or key not in node:
                raise MissingPlatformValue(self.name, keys, self.path)
            node = node[key]
        if node is None or node == "" or node == []:
            raise MissingPlatformValue(self.name, keys, self.path)
        return node

    def get_optional(self, *keys: str, default: Any = None) -> Any:
        try:
            return self.get(*keys)
        except MissingPlatformValue:
            return default

    def has(self, *keys: str) -> bool:
        try:
            self.get(*keys)
        except MissingPlatformValue:
            return False
        return True

    def section(self, *keys: str) -> dict[str, Any]:
        value = self.get_optional(*keys, default={})
        return dict(value) if isinstance(value, dict) else {}

    # -- named groups --------------------------------------------------------

    def topic(self, key: str) -> str:
        return str(self.get("topics", key))

    def frame(self, key: str) -> str:
        return str(self.get("frames", key))

    def service(self, key: str) -> str:
        return str(self.get("services", key))

    def joints(self, key: str = "arm") -> list[str]:
        value = self.get("joints", key)
        return [str(j) for j in value] if isinstance(value, list) else [str(value)]

    @property
    def use_sim_time(self) -> bool:
        return bool(self.get("use_sim_time"))

    @property
    def signals(self) -> dict[str, dict[str, str]]:
        raw = self.section("signals")
        out: dict[str, dict[str, str]] = {}
        for key, value in raw.items():
            if isinstance(value, dict):
                out[str(key)] = {"status": str(value.get("status", "missing")), "source": str(value.get("source", ""))}
            else:
                out[str(key)] = {"status": str(value), "source": ""}
        return out

    def resolve_path(self, spec: str) -> Path:
        """`<package>:<relative>` via ament, else a path relative to the repo root."""
        if ":" in spec and not spec.startswith("/"):
            package, rel = spec.split(":", 1)
            from ament_index_python.packages import get_package_share_directory

            return Path(get_package_share_directory(package)) / rel
        path = Path(spec).expanduser()
        return path if path.is_absolute() else self.repo_root / path

    # -- cameras ---------------------------------------------------------------

    @property
    def cameras_path(self) -> Path:
        return self.repo_root / CAMERAS_DIR / f"{self.name}.yaml"

    @property
    def cameras(self) -> dict[str, dict[str, Any]]:
        if self._cameras is None:
            path = self.cameras_path
            data = _read_yaml(path) if path.is_file() else {}
            cams = data.get("cameras", {})
            self._cameras = {str(k): dict(v) for k, v in cams.items() if isinstance(v, dict)}
        return self._cameras

    def camera(self, name: str) -> dict[str, Any]:
        cams = self.cameras
        if name not in cams:
            raise MissingPlatformValue(self.name, ("cameras", name), self.cameras_path)
        return cams[name]

    def camera_topic(self, camera: str, key: str) -> str:
        topics = self.camera(camera).get("topics", {})
        value = topics.get(key, "") if isinstance(topics, dict) else ""
        if not value:
            raise MissingPlatformValue(self.name, ("cameras", camera, "topics", key), self.cameras_path)
        return str(value)

    def camera_enabled(self, camera: str) -> bool:
        return bool(self.camera(camera).get("enabled", False))

    def __repr__(self) -> str:
        return f"PlatformConfig({self.name!r}, {str(self.path)!r})"


def load_platform_config(
    platform: str | None = None, repo_root: str | os.PathLike[str] | None = None
) -> PlatformConfig:
    root = find_repo_root(repo_root)
    name = resolve_platform_name(platform, root)
    path = root / PLATFORMS_DIR / f"{name}.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"platform config not found: {path}")
    return PlatformConfig(name, _read_yaml(path), path, root)
