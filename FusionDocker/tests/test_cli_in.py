from __future__ import annotations

import argparse
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fusion_docker.cli import _build_parser, _handle_in


class CliInTest(unittest.TestCase):
    def test_cli_parser_supports_in_command(self) -> None:
        parser = _build_parser()
        args = parser.parse_args(["in", "MarvinDocker"])

        self.assertEqual(args.command, "in")
        self.assertEqual(args.docker_name, "MarvinDocker")

    def test_handle_in_attaches_to_tmux_session(self) -> None:
        args = argparse.Namespace(
            docker_name="MarvinDocker",
            launch_config="/tmp/docker_launch.yaml",
            docker_model_root="/tmp/DockerModel",
        )

        launch_cfg = SimpleNamespace(
            docker_model_root="/tmp/DockerModel",
            docker_targets=[],
            docker_groups={},
        )
        target = SimpleNamespace(folder_name="MarvinDocker", is_remote=False)
        match = SimpleNamespace(target=target)

        with (
            mock.patch("fusion_docker.cli._load_optional_launch_config", return_value=launch_cfg),
            mock.patch("fusion_docker.cli._resolve_docker_model_root_value", return_value="/tmp/DockerModel"),
            mock.patch("fusion_docker.cli._require_docker_model_root", return_value=Path("/tmp/DockerModel")),
            mock.patch("fusion_docker.cli._build_group_lookup", return_value={}),
            mock.patch("fusion_docker.cli._match_dockers_for_runtime", return_value=[match]),
            mock.patch("fusion_docker.cli.shutil.which", return_value="/usr/bin/tmux"),
            mock.patch(
                "fusion_docker.cli.subprocess.run",
                side_effect=[
                    SimpleNamespace(returncode=0, stdout="", stderr=""),
                    SimpleNamespace(returncode=0, stdout="", stderr=""),
                    SimpleNamespace(returncode=0, stdout="", stderr=""),
                    SimpleNamespace(returncode=0, stdout="", stderr=""),
                ],
            ) as run_mock,
        ):
            _handle_in(args)

        self.assertEqual(run_mock.call_args_list[0].args[0], ["/usr/bin/tmux", "has-session", "-t", "MarvinDocker"])
        self.assertEqual(run_mock.call_args_list[1].args[0], ["/usr/bin/tmux", "bind-key", "-n", "Escape", "detach-client"])
        self.assertEqual(run_mock.call_args_list[2].args[0], ["/usr/bin/tmux", "attach-session", "-t", "MarvinDocker"])
        self.assertEqual(run_mock.call_args_list[3].args[0], ["/usr/bin/tmux", "unbind-key", "-n", "Escape"])


if __name__ == "__main__":
    unittest.main()
