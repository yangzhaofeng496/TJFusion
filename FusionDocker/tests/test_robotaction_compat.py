from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fusion_docker.core.action_library import ActionLibrary
from fusion_docker.core.object_registry import ObjectRegistry


class RobotactionCompatTest(unittest.TestCase):
    def test_action_library_accepts_robotaction_block_list(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            yaml_path = Path(tmp_dir) / "test_box.yaml"
            yaml_path.write_text(
                "\n".join(
                    [
                        "templates:",
                        "  green box:",
                        "    hold:",
                        "      - action_name: hold",
                        "        rotation_constraint: [100, 100, 100]",
                        "        pose_relative:",
                        "          - [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]",
                        "        gripper_state: [0.0]",
                        "        time: [0.0]",
                        "      - action_name: hold",
                        "        rotation_constraint: [100, 100, 100]",
                        "        pose_relative:",
                        "          - [0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0]",
                        "        gripper_state: [1.0]",
                        "        time: [0.1]",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            library = ActionLibrary.from_yaml(yaml_path)
            template = library.get("green_box", "hold")
            self.assertIsNotNone(template)
            assert template is not None
            self.assertEqual(len(template.pose_relative), 2)
            self.assertEqual(template.gripper_state, [0.0, 1.0])
            self.assertEqual(template.time, [0.0, 0.1])

    def test_object_registry_builds_rules_from_robotaction_status_json(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            yaml_path = root / "test_box.yaml"
            json_path = root / "graph_info.json"

            yaml_path.write_text(
                "\n".join(
                    [
                        "templates:",
                        "  green small box:",
                        "    hold:",
                        "      - action_name: hold",
                        "        pose_relative:",
                        "          - [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]",
                        "        gripper_state: [0.0]",
                        "        time: [0.0]",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            json_path.write_text(
                json.dumps(
                    {
                        "nodes": [
                            {
                                "state_description": "Box Free and Closed",
                                "next_action": [
                                    {
                                        "target": "green small box",
                                        "action_name": "hold",
                                    }
                                ],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            registry = ObjectRegistry.from_robotaction_files(
                object_yaml_path=yaml_path,
                status_json_path=json_path,
            )
            profile = registry.resolve(object_type="green_small_box")
            self.assertIsNotNone(profile)
            assert profile is not None
            self.assertEqual(profile.template_key, "green_small_box")
            self.assertIn("hold", profile.affordances)
            self.assertEqual(len(profile.action_rules), 1)
            self.assertEqual(profile.action_rules[0].action, "hold")
            self.assertIn("box_free_and_closed", profile.action_rules[0].current_state)


if __name__ == "__main__":
    unittest.main()
