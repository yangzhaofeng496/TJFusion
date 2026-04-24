from __future__ import annotations

import argparse
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fusion_docker.cli import _build_parser, _handle_model_download


class CliModelDownloadTest(unittest.TestCase):
    def test_cli_parser_supports_md_command(self) -> None:
        parser = _build_parser()
        args = parser.parse_args(
            [
                "md",
                "Sam3Docker",
                "--docker-model-root",
                "/tmp/DockerModel",
            ]
        )
        self.assertEqual(args.command, "md")
        self.assertEqual(args.docker_name, "Sam3Docker")
        self.assertEqual(args.docker_model_root, "/tmp/DockerModel")

    def test_handle_model_download_runs_download_script_in_model_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            model_dir = root / "Sam3Docker" / "model"
            model_dir.mkdir(parents=True, exist_ok=True)
            (model_dir / "download.sh").write_text("#!/usr/bin/env bash\necho ok\n", encoding="utf-8")

            args = argparse.Namespace(
                docker_name="Sam3Docker",
                docker_model_root=str(root),
            )

            with mock.patch("fusion_docker.cli._run_subprocess") as run_mock:
                _handle_model_download(args)

            run_mock.assert_called_once_with(
                ["bash", "./download.sh"],
                capture_output=False,
                cwd=model_dir,
            )

    def test_handle_model_download_raises_when_script_missing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            model_dir = root / "Sam3Docker" / "model"
            model_dir.mkdir(parents=True, exist_ok=True)
            args = argparse.Namespace(
                docker_name="Sam3Docker",
                docker_model_root=str(root),
            )

            with self.assertRaisesRegex(FileNotFoundError, "请联系开发者获取download.sh脚本"):
                _handle_model_download(args)


if __name__ == "__main__":
    unittest.main()
