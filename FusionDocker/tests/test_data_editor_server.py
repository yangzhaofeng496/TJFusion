from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fusion_docker.cli import _build_parser
from fusion_docker.data_editor_server import (
    ROBOTACTION_GRAPH_FILE,
    ROBOTACTION_TEST_BOX_FILE,
    _validate_graph_info_json,
    _validate_test_box_yaml,
    ensure_robotaction_data_files,
)


class DataEditorServerTest(unittest.TestCase):
    def test_ensure_robotaction_data_files_copies_seed_content(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            seed_dir = root / "seed"
            data_dir = root / "target"
            seed_dir.mkdir(parents=True, exist_ok=True)
            seed_yaml = "templates:\n  x:\n    hold:\n      - action_name: hold\n        pose_relative: [[0,0,0,0,0,0,1]]\n        gripper_state: [0]\n        time: [0]\n"
            seed_json = json.dumps({"nodes": []}, ensure_ascii=False)
            (seed_dir / ROBOTACTION_TEST_BOX_FILE).write_text(seed_yaml, encoding="utf-8")
            (seed_dir / ROBOTACTION_GRAPH_FILE).write_text(seed_json, encoding="utf-8")

            ensure_robotaction_data_files(data_dir=data_dir, seed_dirs=[seed_dir])

            self.assertEqual(
                (data_dir / "templates" / ROBOTACTION_TEST_BOX_FILE).read_text(encoding="utf-8"),
                seed_yaml,
            )
            self.assertEqual(
                (data_dir / "graphs" / ROBOTACTION_GRAPH_FILE).read_text(encoding="utf-8"),
                seed_json,
            )

    def test_validate_methods(self) -> None:
        _validate_test_box_yaml(
            "templates:\n  box:\n    hold:\n      - action_name: hold\n        pose_relative: [[0,0,0,0,0,0,1]]\n        gripper_state: [0]\n        time: [0]\n"
        )
        _validate_graph_info_json(json.dumps({"nodes": []}))

        with self.assertRaises(ValueError):
            _validate_test_box_yaml("not: [valid")
        with self.assertRaises(ValueError):
            _validate_graph_info_json("{bad json")

    def test_cli_parser_supports_serve_data_editor(self) -> None:
        parser = _build_parser()
        args = parser.parse_args(
            [
                "serve-data-editor",
                "--host",
                "0.0.0.0",
                "--port",
                "9010",
                "--data-dir",
                "configs/robotaction_data",
                "--templates-dir",
                "configs/robotaction_data/templates",
                "--graphs-dir",
                "configs/robotaction_data/graphs",
            ]
        )
        self.assertEqual(args.command, "serve-data-editor")
        self.assertEqual(args.host, "0.0.0.0")
        self.assertEqual(args.port, 9010)
        self.assertEqual(args.templates_dir, "configs/robotaction_data/templates")
        self.assertEqual(args.graphs_dir, "configs/robotaction_data/graphs")


if __name__ == "__main__":
    unittest.main()
