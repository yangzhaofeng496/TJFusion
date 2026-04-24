from __future__ import annotations

import argparse
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fusion_docker.cli import _build_parser, _handle_list_zmq_topics, _handle_listen_zmq


class CliZmqTest(unittest.TestCase):
    def test_cli_parser_listen_zmq_uses_default_port_8899(self) -> None:
        parser = _build_parser()
        args = parser.parse_args(["listen-zmq"])

        self.assertEqual(args.command, "listen-zmq")
        self.assertEqual(args.port, 8899)
        self.assertIsNone(args.topic_positional)

    def test_cli_parser_listen_zmq_supports_positional_topic(self) -> None:
        parser = _build_parser()
        args = parser.parse_args(["listen-zmq", "/action"])

        self.assertEqual(args.command, "listen-zmq")
        self.assertEqual(args.topic_positional, "/action")

    def test_cli_parser_supports_list_zmq_topics_defaults(self) -> None:
        parser = _build_parser()
        args = parser.parse_args(["list-zmq-topics"])

        self.assertEqual(args.command, "list-zmq-topics")
        self.assertEqual(args.host, "127.0.0.1")
        self.assertEqual(args.port, 8899)
        self.assertEqual(args.limit, 200)
        self.assertEqual(args.timeout_ms, 2000)

    def test_handle_list_zmq_topics_collects_and_prints_topics(self) -> None:
        args = argparse.Namespace(
            host="127.0.0.1",
            port=8899,
            limit=100,
            timeout_ms=1500,
        )

        def _fake_listen(*, on_message, **_kwargs):
            on_message(SimpleNamespace(rendered='{"part_count":2,"parts":["/tf","{\\"ok\\":true}"]}'))
            on_message(SimpleNamespace(rendered='{"part_count":2,"parts":["/tf","{\\"ok\\":true}"]}'))
            on_message(SimpleNamespace(rendered='{"part_count":2,"parts":["/siglip2/result","{\\"ok\\":true}"]}'))
            return [object(), object(), object()]

        with (
            mock.patch("fusion_docker.cli.listen_zmq_messages", side_effect=_fake_listen) as listen_mock,
            mock.patch("fusion_docker.cli.print_status") as status_mock,
            mock.patch("fusion_docker.cli.print_warning") as warning_mock,
        ):
            _handle_list_zmq_topics(args)

        warning_mock.assert_not_called()
        listen_mock.assert_called_once()
        status_texts = [call.args[1] for call in status_mock.call_args_list if len(call.args) >= 2]
        self.assertTrue(any("Discovered 2 topic(s)" in text for text in status_texts))
        self.assertTrue(any("/tf (messages=2)" in text for text in status_texts))
        self.assertTrue(any("/siglip2/result (messages=1)" in text for text in status_texts))

    def test_handle_listen_zmq_uses_topic_in_status_tag(self) -> None:
        args = argparse.Namespace(
            host="127.0.0.1",
            port=8899,
            topic="",
            topic_positional=None,
            limit=1,
            timeout_ms=None,
        )

        def _fake_listen(*, on_message, **_kwargs):
            on_message(SimpleNamespace(index=1, part_count=1, rendered='/siglip2/result {"ok":true}'))
            return [object()]

        with (
            mock.patch("fusion_docker.cli.listen_zmq_messages", side_effect=_fake_listen),
            mock.patch("fusion_docker.cli.print_status") as status_mock,
        ):
            _handle_listen_zmq(args)

        tags = [call.args[0] for call in status_mock.call_args_list if call.args]
        self.assertIn("ZMQ(/siglip2/result)", tags)


if __name__ == "__main__":
    unittest.main()
