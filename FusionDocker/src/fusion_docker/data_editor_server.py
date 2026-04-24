from __future__ import annotations

from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import yaml

from fusion_docker.console import print_status, print_warning
from fusion_docker.robotaction_data import (
    ROBOTACTION_GRAPH_FILE,
    ROBOTACTION_TEST_BOX_FILE,
    ensure_robotaction_split_layout,
    resolve_robotaction_data_paths,
)


def serve_robotaction_data_editor(
    *,
    host: str = "127.0.0.1",
    port: int = 8770,
    project_root: Path | None = None,
    data_dir: str | Path | None = None,
    templates_dir: str | Path | None = None,
    graphs_dir: str | Path | None = None,
    seed_dir: str | Path | None = None,
) -> None:
    resolved_project_root = (
        project_root.resolve()
        if project_root is not None
        else Path(__file__).resolve().parents[2]
    )
    paths = resolve_robotaction_data_paths(
        project_root=resolved_project_root,
        data_dir=data_dir,
        templates_dir=templates_dir,
        graphs_dir=graphs_dir,
    )
    seed_candidates = _build_seed_candidates(
        project_root=resolved_project_root,
        explicit_seed_dir=seed_dir,
    )
    ensure_robotaction_data_files(
        data_dir=paths.data_root_dir,
        templates_dir=paths.templates_dir,
        graphs_dir=paths.graphs_dir,
        seed_dirs=seed_candidates,
    )

    server = ThreadingHTTPServer(
        (host, port),
        _build_data_editor_handler(
            data_dir=paths.data_root_dir,
            templates_dir=paths.templates_dir,
            graphs_dir=paths.graphs_dir,
        ),
    )
    display_host = "127.0.0.1" if host in {"0.0.0.0", "::"} else host
    print_status(
        "DATA-UI",
        f"Robotaction data editor is available at http://{display_host}:{server.server_port}",
        color="cyan",
    )
    print_status(
        "DATA-UI",
        (
            f"Editing files under templates={paths.templates_dir} "
            f"graphs={paths.graphs_dir}"
        ),
        color="cyan",
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print_warning("Stopping robotaction data editor.")
    finally:
        server.server_close()


def ensure_robotaction_data_files(
    *,
    data_dir: Path,
    templates_dir: Path | None = None,
    graphs_dir: Path | None = None,
    seed_dirs: list[Path] | None = None,
) -> dict[str, Path]:
    paths = resolve_robotaction_data_paths(
        project_root=Path.cwd(),
        data_dir=data_dir,
        templates_dir=templates_dir,
        graphs_dir=graphs_dir,
    )
    paths.data_root_dir.mkdir(parents=True, exist_ok=True)
    ensure_robotaction_split_layout(paths)
    targets = {
        ROBOTACTION_TEST_BOX_FILE: paths.test_box_path,
        ROBOTACTION_GRAPH_FILE: paths.graph_info_path,
    }
    seeds = list(seed_dirs or [])

    for file_name, target_path in targets.items():
        if target_path.exists():
            continue
        copied = False
        for seed_dir in seeds:
            for candidate in _seed_candidates_for_file(seed_dir, file_name):
                if candidate.is_file():
                    target_path.parent.mkdir(parents=True, exist_ok=True)
                    target_path.write_text(
                        candidate.read_text(encoding="utf-8"),
                        encoding="utf-8",
                    )
                    copied = True
                    break
            if copied:
                break
        if copied:
            continue
        target_path.parent.mkdir(parents=True, exist_ok=True)
        target_path.write_text(_default_content_for(file_name), encoding="utf-8")

    return targets


def _default_content_for(file_name: str) -> str:
    if file_name == ROBOTACTION_TEST_BOX_FILE:
        return (
            "templates:\n"
            "  demo object:\n"
            "    hold:\n"
            "      - action_name: hold\n"
            "        arm: right\n"
            "        rotation_constraint: [100, 100, 100]\n"
            "        pose_relative:\n"
            "          - [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]\n"
            "        gripper_state: [0.0]\n"
            "        time: [0.0]\n"
        )
    return json.dumps({"nodes": []}, ensure_ascii=False, indent=2) + "\n"


def _build_seed_candidates(
    *,
    project_root: Path,
    explicit_seed_dir: str | Path | None,
) -> list[Path]:
    candidates: list[Path] = []
    if explicit_seed_dir is not None:
        seed_path = Path(explicit_seed_dir)
        candidates.append(seed_path if seed_path.is_absolute() else (project_root / seed_path))
    candidates.append(project_root / "MarvinDocker" / "robotaction" / "data")
    unique: list[Path] = []
    seen: set[str] = set()
    for path in candidates:
        resolved = path.expanduser().resolve()
        key = str(resolved)
        if key in seen:
            continue
        seen.add(key)
        unique.append(resolved)
    return unique


def _build_data_editor_handler(
    *,
    data_dir: Path,
    templates_dir: Path,
    graphs_dir: Path,
) -> type[BaseHTTPRequestHandler]:
    files = ensure_robotaction_data_files(
        data_dir=data_dir,
        templates_dir=templates_dir,
        graphs_dir=graphs_dir,
        seed_dirs=[],
    )

    class DataEditorHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/":
                self._send_html(_build_data_editor_html())
                return
            if parsed.path == "/api/files":
                query = parse_qs(parsed.query)
                template_file = str(query.get("template_file", [""])[0] or "").strip()
                graph_file = str(query.get("graph_file", [""])[0] or "").strip()
                self._send_json(
                    _read_files_payload(
                        files,
                        templates_dir=templates_dir,
                        graphs_dir=graphs_dir,
                        selected_template_file=template_file,
                        selected_graph_file=graph_file,
                    )
                )
                return
            if parsed.path == "/favicon.ico":
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return
            self._send_json(
                {"ok": False, "error": f"Unknown path: {parsed.path}"},
                status=HTTPStatus.NOT_FOUND,
            )

        def do_POST(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path != "/api/files/save":
                self._send_json(
                    {"ok": False, "error": f"Unknown path: {parsed.path}"},
                    status=HTTPStatus.NOT_FOUND,
                )
                return
            try:
                payload = self._read_json_body()
                template_file = str(payload.get("template_file", "")).strip()
                graph_file = str(payload.get("graph_file", "")).strip()
                yaml_text = str(payload.get("template_content", payload.get("test_box_yaml", "")))
                json_text = str(payload.get("graph_content", payload.get("graph_info_json", "")))
                template_path = _safe_join_file(
                    templates_dir=templates_dir,
                    graphs_dir=graphs_dir,
                    kind="template",
                    file_name=template_file or ROBOTACTION_TEST_BOX_FILE,
                )
                graph_path = _safe_join_file(
                    templates_dir=templates_dir,
                    graphs_dir=graphs_dir,
                    kind="graph",
                    file_name=graph_file or ROBOTACTION_GRAPH_FILE,
                )
                _validate_test_box_yaml(yaml_text)
                _validate_graph_info_json(json_text)
                template_path.write_text(yaml_text, encoding="utf-8")
                graph_path.write_text(json_text, encoding="utf-8")
                self._send_json(
                    {
                        "ok": True,
                        "message": "Saved robotaction data files.",
                        "template_file": template_path.name,
                        "graph_file": graph_path.name,
                        "updated_at": datetime.now(timezone.utc).isoformat(),
                    }
                )
            except ValueError as exc:
                self._send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            except Exception as exc:
                self._send_json(
                    {"ok": False, "error": f"Failed to save files: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )

        def log_message(self, format: str, *args: object) -> None:
            return

        def _read_json_body(self) -> dict[str, Any]:
            length_header = self.headers.get("Content-Length", "")
            try:
                length = int(length_header)
            except ValueError as exc:
                raise ValueError("Content-Length header is required.") from exc
            raw_body = self.rfile.read(length)
            try:
                parsed = json.loads(raw_body.decode("utf-8"))
            except json.JSONDecodeError as exc:
                raise ValueError("Request body must be valid JSON.") from exc
            if not isinstance(parsed, dict):
                raise ValueError("Request body JSON must be an object.")
            return parsed

        def _send_json(self, payload: dict[str, Any], *, status: HTTPStatus = HTTPStatus.OK) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status.value)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_html(self, html_content: str, *, status: HTTPStatus = HTTPStatus.OK) -> None:
            body = html_content.encode("utf-8")
            self.send_response(status.value)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return DataEditorHandler


def _seed_candidates_for_file(seed_dir: Path, file_name: str) -> list[Path]:
    if file_name == ROBOTACTION_TEST_BOX_FILE:
        return [
            seed_dir / "templates" / file_name,
            seed_dir / file_name,
        ]
    if file_name == ROBOTACTION_GRAPH_FILE:
        return [
            seed_dir / "graphs" / file_name,
            seed_dir / file_name,
        ]
    return [seed_dir / file_name]


def _validate_test_box_yaml(content: str) -> None:
    if not content.strip():
        raise ValueError("test_box.yaml content cannot be empty.")
    try:
        parsed = yaml.safe_load(content)
    except Exception as exc:
        raise ValueError(f"test_box.yaml is not valid YAML: {exc}") from exc
    if not isinstance(parsed, dict):
        raise ValueError("test_box.yaml root must be a mapping.")
    templates = parsed.get("templates")
    if not isinstance(templates, dict):
        raise ValueError("test_box.yaml must contain mapping field 'templates'.")


def _validate_graph_info_json(content: str) -> None:
    if not content.strip():
        raise ValueError("graph_info.json content cannot be empty.")
    try:
        parsed = json.loads(content)
    except json.JSONDecodeError as exc:
        raise ValueError(f"graph_info.json is not valid JSON: {exc}") from exc
    if not isinstance(parsed, dict):
        raise ValueError("graph_info.json root must be an object.")
    nodes = parsed.get("nodes")
    if not isinstance(nodes, list):
        raise ValueError("graph_info.json must contain list field 'nodes'.")


def _read_files_payload(
    files: dict[str, Path],
    *,
    templates_dir: Path | None = None,
    graphs_dir: Path | None = None,
    selected_template_file: str = "",
    selected_graph_file: str = "",
) -> dict[str, Any]:
    effective_templates_dir = templates_dir or files[ROBOTACTION_TEST_BOX_FILE].parent
    effective_graphs_dir = graphs_dir or files[ROBOTACTION_GRAPH_FILE].parent

    template_files = _list_template_files(effective_templates_dir)
    graph_files = _list_graph_files(effective_graphs_dir)

    template_name = selected_template_file if selected_template_file in template_files else ""
    graph_name = selected_graph_file if selected_graph_file in graph_files else ""
    if not template_name:
        template_name = ROBOTACTION_TEST_BOX_FILE if ROBOTACTION_TEST_BOX_FILE in template_files else (
            template_files[0] if template_files else ROBOTACTION_TEST_BOX_FILE
        )
    if not graph_name:
        graph_name = ROBOTACTION_GRAPH_FILE if ROBOTACTION_GRAPH_FILE in graph_files else (
            graph_files[0] if graph_files else ROBOTACTION_GRAPH_FILE
        )

    test_box_path = _safe_join_file(
        templates_dir=effective_templates_dir,
        graphs_dir=effective_graphs_dir,
        kind="template",
        file_name=template_name,
    )
    graph_path = _safe_join_file(
        templates_dir=effective_templates_dir,
        graphs_dir=effective_graphs_dir,
        kind="graph",
        file_name=graph_name,
    )
    if not test_box_path.exists():
        test_box_path.write_text(_default_content_for(ROBOTACTION_TEST_BOX_FILE), encoding="utf-8")
    if not graph_path.exists():
        graph_path.write_text(_default_content_for(ROBOTACTION_GRAPH_FILE), encoding="utf-8")

    return {
        "ok": True,
        "paths": {
            "test_box_yaml": str(test_box_path),
            "graph_info_json": str(graph_path),
            "templates_dir": str(effective_templates_dir),
            "graphs_dir": str(effective_graphs_dir),
        },
        "templates_files": template_files,
        "graphs_files": graph_files,
        "selected_template_file": test_box_path.name,
        "selected_graph_file": graph_path.name,
        "test_box_yaml": test_box_path.read_text(encoding="utf-8"),
        "graph_info_json": graph_path.read_text(encoding="utf-8"),
        "template_content": test_box_path.read_text(encoding="utf-8"),
        "graph_content": graph_path.read_text(encoding="utf-8"),
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }


def _safe_join_file(
    *,
    templates_dir: Path,
    graphs_dir: Path,
    kind: str,
    file_name: str,
) -> Path:
    if kind == "template":
        base_dir = templates_dir
        valid_exts = {".yaml", ".yml"}
    elif kind == "graph":
        base_dir = graphs_dir
        valid_exts = {".json"}
    else:
        raise ValueError(f"Unsupported file kind: {kind}")
    name = Path(file_name).name
    if not name:
        raise ValueError(f"{kind} file name cannot be empty.")
    suffix = Path(name).suffix.lower()
    if suffix not in valid_exts:
        raise ValueError(f"{kind} file must use one of {sorted(valid_exts)}: {name}")
    resolved = (base_dir / name).resolve()
    if base_dir.resolve() not in resolved.parents and resolved != base_dir.resolve():
        raise ValueError(f"Invalid {kind} file path: {name}")
    return resolved


def _list_template_files(templates_dir: Path) -> list[str]:
    files = [
        path.name
        for path in sorted(templates_dir.glob("*"))
        if path.is_file() and path.suffix.lower() in {".yaml", ".yml"}
    ]
    return files


def _list_graph_files(graphs_dir: Path) -> list[str]:
    files = [
        path.name
        for path in sorted(graphs_dir.glob("*"))
        if path.is_file() and path.suffix.lower() == ".json"
    ]
    return files


def _build_data_editor_html() -> str:
    return """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Robotaction Data Editor</title>
  <style>
    :root {
      --bg: #f4f6f8;
      --panel: #ffffff;
      --line: #d4dbe3;
      --text: #203042;
      --muted: #607082;
      --accent: #1d6fd6;
      --danger: #c23b31;
      --ok: #248a3d;
    }
    body { margin: 0; font-family: "Source Sans 3", "Segoe UI", sans-serif; background: var(--bg); color: var(--text); }
    .wrap { max-width: 1300px; margin: 0 auto; padding: 16px; }
    .toolbar { display: flex; gap: 8px; align-items: center; margin-bottom: 12px; }
    button { border: 1px solid var(--line); background: var(--panel); padding: 8px 12px; cursor: pointer; border-radius: 8px; }
    button.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
    .status { margin-left: 8px; color: var(--muted); font-size: 14px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .panel { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 10px; min-height: 70vh; display: flex; flex-direction: column; }
    .title { font-weight: 700; margin-bottom: 6px; font-size: 14px; }
    textarea { flex: 1; width: 100%; resize: vertical; border: 1px solid var(--line); border-radius: 8px; padding: 10px; font-family: "JetBrains Mono", "Fira Code", monospace; font-size: 13px; line-height: 1.4; }
    .meta { font-size: 12px; color: var(--muted); margin-bottom: 8px; word-break: break-all; }
    .ok { color: var(--ok); }
    .err { color: var(--danger); }
    @media (max-width: 980px) { .grid { grid-template-columns: 1fr; } .panel { min-height: 45vh; } }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="toolbar">
      <button onclick="loadFiles()">Reload</button>
      <button class="primary" onclick="saveFiles()">Save</button>
      <span class="status" id="status">Loading...</span>
    </div>
    <div class="grid">
      <div class="panel">
        <div class="title">Templates (*.yaml/*.yml)</div>
        <select id="template-file"></select>
        <div class="meta" id="path-yaml"></div>
        <textarea id="yaml-text"></textarea>
      </div>
      <div class="panel">
        <div class="title">Graphs (*.json)</div>
        <select id="graph-file"></select>
        <div class="meta" id="path-json"></div>
        <textarea id="json-text"></textarea>
      </div>
    </div>
  </div>
  <script>
    const statusEl = document.getElementById("status");
    const yamlEl = document.getElementById("yaml-text");
    const jsonEl = document.getElementById("json-text");
    const pathYamlEl = document.getElementById("path-yaml");
    const pathJsonEl = document.getElementById("path-json");
    const templateFileEl = document.getElementById("template-file");
    const graphFileEl = document.getElementById("graph-file");

    function setStatus(text, cls = "") {
      statusEl.textContent = text;
      statusEl.className = "status " + cls;
    }

    function refreshSelect(selectEl, files, selected) {
      selectEl.innerHTML = "";
      for (const name of files || []) {
        const option = document.createElement("option");
        option.value = name;
        option.textContent = name;
        if (name === selected) option.selected = true;
        selectEl.appendChild(option);
      }
    }

    async function loadFiles() {
      setStatus("Loading...");
      const templateFile = encodeURIComponent(templateFileEl.value || "");
      const graphFile = encodeURIComponent(graphFileEl.value || "");
      const res = await fetch(`/api/files?template_file=${templateFile}&graph_file=${graphFile}`);
      const payload = await res.json();
      if (!res.ok || !payload.ok) {
        setStatus(payload.error || "Load failed", "err");
        return;
      }
      refreshSelect(templateFileEl, payload.templates_files || [], payload.selected_template_file || "");
      refreshSelect(graphFileEl, payload.graphs_files || [], payload.selected_graph_file || "");
      yamlEl.value = payload.template_content || payload.test_box_yaml || "";
      jsonEl.value = payload.graph_content || payload.graph_info_json || "";
      pathYamlEl.textContent = payload.paths?.test_box_yaml || "";
      pathJsonEl.textContent = payload.paths?.graph_info_json || "";
      setStatus("Loaded", "ok");
    }

    async function saveFiles() {
      setStatus("Saving...");
      const res = await fetch("/api/files/save", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({
          template_file: templateFileEl.value || "",
          graph_file: graphFileEl.value || "",
          template_content: yamlEl.value,
          graph_content: jsonEl.value,
        }),
      });
      const payload = await res.json();
      if (!res.ok || !payload.ok) {
        setStatus(payload.error || "Save failed", "err");
        return;
      }
      setStatus("Saved", "ok");
    }

    templateFileEl.addEventListener("change", loadFiles);
    graphFileEl.addEventListener("change", loadFiles);
    loadFiles();
  </script>
</body>
</html>
"""
