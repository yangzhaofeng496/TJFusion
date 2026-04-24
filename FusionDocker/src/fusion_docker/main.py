from __future__ import annotations

import logging
import os
from pathlib import Path

from fusion_docker.config import get_app_config_path, load_app_config
from fusion_docker.core.action_library import ActionLibrary
from fusion_docker.core.fusion_engine import FusionEngine
from fusion_docker.core.object_registry import ObjectRegistry
from fusion_docker.core.state_matcher import StateMatcher
from fusion_docker.messaging.zmq_bus import ZmqPublisher, ZmqSubscriberBus
from fusion_docker.robotaction_data import resolve_robotaction_data_paths


def run_fusion_service() -> None:
    _configure_logging()
    logger = logging.getLogger("fusion_docker")

    app_config_path = get_app_config_path()
    app_config = load_app_config(app_config_path)

    robotaction_paths = resolve_robotaction_data_paths(project_root=Path.cwd().resolve())
    robotaction_object_yaml = robotaction_paths.test_box_path
    robotaction_status_json = robotaction_paths.graph_info_path
    if not robotaction_object_yaml.exists():
        raise FileNotFoundError(
            f"Robotaction template YAML not found: {robotaction_object_yaml}"
        )
    if not robotaction_status_json.exists():
        raise FileNotFoundError(
            f"Robotaction graph JSON not found: {robotaction_status_json}"
        )

    action_library = ActionLibrary.from_yaml(robotaction_object_yaml)
    object_registry = ObjectRegistry.from_robotaction_files(
        object_yaml_path=robotaction_object_yaml,
        status_json_path=robotaction_status_json,
    )
    logger.info(
        "Using robotaction mode only. object_yaml=%s status_json=%s",
        robotaction_object_yaml,
        robotaction_status_json,
    )
    output_config = app_config.outputs.get("marvin_command") or next(
        iter(app_config.outputs.values())
    )

    engine = FusionEngine(
        object_registry=object_registry,
        action_library=action_library,
        state_matcher=StateMatcher(),
    )
    subscriber = ZmqSubscriberBus(app_config.inputs)
    publisher = ZmqPublisher(output_config)

    logger.info(
        "FusionDocker started with app_config=%s robotaction_object_yaml=%s robotaction_status_json=%s",
        app_config_path,
        robotaction_object_yaml,
        robotaction_status_json,
    )

    try:
        while True:
            for input_name, topic, payload in subscriber.poll(app_config.poll_timeout_ms):
                logger.debug("Received input=%s topic=%s payload=%s", input_name, topic, payload)
                command = engine.handle_event(input_name, payload)
                if command is None:
                    continue
                publisher.send_json(command)
                logger.info(
                    "Dispatched action=%s object_id=%s topic=%s",
                    command.get("action_name"),
                    command.get("object_id"),
                    output_config.topic,
                )
    except KeyboardInterrupt:
        logger.info("FusionDocker interrupted, shutting down")
    finally:
        subscriber.close()
        publisher.close()


def main() -> None:
    run_fusion_service()


def _configure_logging() -> None:
    level_name = os.getenv("FUSION_LOG_LEVEL", "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


if __name__ == "__main__":
    main()
