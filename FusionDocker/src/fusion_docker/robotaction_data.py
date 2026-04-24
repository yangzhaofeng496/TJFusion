from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import yaml

ROBOTACTION_TEST_BOX_FILE = "test_box.yaml"
ROBOTACTION_GRAPH_FILE = "graph_info.json"
ROBOTACTION_TEMPLATES_SUBDIR = "templates"
ROBOTACTION_GRAPHS_SUBDIR = "graphs"
ROBOTACTION_ACTION_TEMPLATES_SUBDIR = "action_templates"
ROBOTACTION_GRAPH_INFO_SUBDIR = "graph_info"


@dataclass(slots=True)
class RobotactionDataPaths:
    data_root_dir: Path
    templates_dir: Path
    graphs_dir: Path
    test_box_path: Path
    graph_info_path: Path


def resolve_robotaction_data_paths(
    *,
    project_root: Path | None = None,
    data_dir: str | Path | None = None,
    templates_dir: str | Path | None = None,
    graphs_dir: str | Path | None = None,
    test_box_file: str = ROBOTACTION_TEST_BOX_FILE,
    graph_info_file: str = ROBOTACTION_GRAPH_FILE,
) -> RobotactionDataPaths:
    root = (project_root or Path.cwd()).resolve()
    resolved_data_dir = _resolve_path(
        data_dir,
        project_root=root,
        default_relative=Path("configs") / "robotaction_data",
    )
    default_templates_dir = _choose_existing_subdir(
        resolved_data_dir,
        preferred=ROBOTACTION_ACTION_TEMPLATES_SUBDIR,
        fallback=ROBOTACTION_TEMPLATES_SUBDIR,
    )
    default_graphs_dir = _choose_existing_subdir(
        resolved_data_dir,
        preferred=ROBOTACTION_GRAPH_INFO_SUBDIR,
        fallback=ROBOTACTION_GRAPHS_SUBDIR,
    )
    resolved_templates_dir = _resolve_path(
        templates_dir,
        project_root=root,
        default_relative=default_templates_dir,
    )
    resolved_graphs_dir = _resolve_path(
        graphs_dir,
        project_root=root,
        default_relative=default_graphs_dir,
    )
    test_box_path = (resolved_templates_dir / str(test_box_file)).resolve()
    graph_info_path = (resolved_graphs_dir / str(graph_info_file)).resolve()

    legacy_test_box = (resolved_data_dir / str(test_box_file)).resolve()
    legacy_graph = (resolved_data_dir / str(graph_info_file)).resolve()
    if not test_box_path.exists() and legacy_test_box.exists():
        test_box_path = legacy_test_box
    if not graph_info_path.exists() and legacy_graph.exists():
        graph_info_path = legacy_graph

    return RobotactionDataPaths(
        data_root_dir=resolved_data_dir,
        templates_dir=resolved_templates_dir,
        graphs_dir=resolved_graphs_dir,
        test_box_path=test_box_path,
        graph_info_path=graph_info_path,
    )


def ensure_robotaction_split_layout(paths: RobotactionDataPaths) -> None:
    paths.templates_dir.mkdir(parents=True, exist_ok=True)
    paths.graphs_dir.mkdir(parents=True, exist_ok=True)


def load_template_names(test_box_path: str | Path) -> list[str]:
    path = Path(test_box_path)
    with path.open("r", encoding="utf-8") as handle:
        parsed = yaml.safe_load(handle) or {}
    if not isinstance(parsed, dict):
        raise ValueError(f"robotaction test_box root must be a mapping: {path}")
    templates = parsed.get("templates")
    if not isinstance(templates, dict):
        raise ValueError(f"robotaction test_box must contain mapping field 'templates': {path}")
    names = [str(key).strip() for key in templates.keys() if str(key).strip()]
    return list(dict.fromkeys(names))


def load_graph_targets(graph_info_path: str | Path) -> list[str]:
    path = Path(graph_info_path)
    with path.open("r", encoding="utf-8") as handle:
        parsed = json.load(handle) or {}
    if not isinstance(parsed, dict):
        raise ValueError(f"robotaction graph_info root must be an object: {path}")
    nodes = parsed.get("nodes")
    if not isinstance(nodes, list):
        raise ValueError(f"robotaction graph_info must contain list field 'nodes': {path}")

    targets: list[str] = []
    for node in nodes:
        if not isinstance(node, dict):
            continue
        next_action = node.get("next_action")
        for action in _iter_next_actions(next_action):
            target = str(action.get("target", "")).strip()
            if target:
                targets.append(target)
    return list(dict.fromkeys(targets))


def _iter_next_actions(next_action: Any) -> list[dict[str, Any]]:
    if isinstance(next_action, list):
        return [item for item in next_action if isinstance(item, dict)]
    return []


def _resolve_path(
    raw_path: str | Path | None,
    *,
    project_root: Path,
    default_relative: Path,
) -> Path:
    if raw_path is None:
        base = default_relative
    else:
        base = Path(raw_path)
    if base.is_absolute():
        return base.resolve()
    return (project_root / base).resolve()


def _choose_existing_subdir(base_dir: Path, *, preferred: str, fallback: str) -> Path:
    preferred_path = (base_dir / preferred).resolve()
    fallback_path = (base_dir / fallback).resolve()
    if preferred_path.exists():
        return preferred_path
    return fallback_path
