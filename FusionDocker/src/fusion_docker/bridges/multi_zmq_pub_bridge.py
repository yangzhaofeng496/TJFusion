from __future__ import annotations

from pathlib import Path

from fusion_docker.bridges.base import BridgeDefinition
from fusion_docker.bridges.profiled import BridgeProfile, load_profiled_bridge_config
from fusion_docker.robotaction_data import (
    load_graph_targets,
    load_template_names,
    resolve_robotaction_data_paths,
)


def _mutate_multi_zmq_pub_bridge_config(config):
    config.run_sam3_flowpose = True
    config.flowpose_sidecar_server_addr = ""
    if not config.sam3_server_addr or not config.flowpose_server_addr:
        raise ValueError(
            "multi_zmq_pub_bridge requires bridge.sam3_server_addr and bridge.flowpose_server_addr."
        )
    if not config.siglip2_server_addr:
        raise ValueError(
            "multi_zmq_pub_bridge requires bridge.siglip2_server_addr."
        )
    if not config.result_pub_addr:
        raise ValueError(
            "multi_zmq_pub_bridge requires bridge.result_pub_addr."
        )
    _apply_robotaction_data_config(config)
    return config


def _run_multi_zmq_pub_bridge(config, *, verbose: bool = False, save_json: bool = False) -> None:
    from fusion_docker.bridge_pub import BridgeResultPublisher
    from fusion_docker.bridge_service import run_bridge_service, run_zmq_source_bridge_service_decoupled
    from fusion_docker.console import print_status

    robotaction_runtime = None
    if _has_robotaction_config(config):
        paths = resolve_robotaction_data_paths(
            project_root=Path.cwd(),
            data_dir=(config.robotaction_data_dir or None),
            templates_dir=(config.robotaction_templates_dir or None),
            graphs_dir=(config.robotaction_graphs_dir or None),
            test_box_file=config.robotaction_test_box_file,
            graph_info_file=config.robotaction_graph_info_file,
        )
        print_status("PUB", f"Robotaction test : {paths.test_box_path}", color="cyan")
        print_status("PUB", f"Robotaction graph: {paths.graph_info_path}", color="cyan")
        if bool(getattr(config, "robotaction_auto_run", False)):
            from fusion_docker.robotaction.json_runtime import RobotActionJsonRuntime

            robotaction_runtime = RobotActionJsonRuntime(
                template_path=paths.test_box_path,
                graph_path=paths.graph_info_path,
                stable_frames=2,
            )
            print_status(
                "ROBACTION",
                (
                    "Embedded JSON runtime enabled "
                    f"(action_topic={getattr(config, 'robotaction_action_topic', '/action')})"
                ),
                color="green",
            )

    publisher = BridgeResultPublisher(
        config.result_pub_addr,
        frame_id=config.result_pub_frame_id,
        siglip_topic=config.result_siglip_topic,
        tf_topic=config.result_tf_topic,
        action_topic=str(getattr(config, "robotaction_action_topic", "/action")),
        siglip_vote_window=config.result_siglip_vote_window,
        robotaction_runtime=robotaction_runtime,
    )
    print_status("PUB", f"Result PUB       : {config.result_pub_addr}", color="cyan")
    print_status("PUB", f"TF frame id      : {config.result_pub_frame_id}", color="cyan")
    print_status("PUB", f"Siglip topic     : {config.result_siglip_topic}", color="cyan")
    print_status("PUB", f"Siglip vote k    : {config.result_siglip_vote_window}", color="cyan")
    print_status("PUB", f"TF topic         : {config.result_tf_topic}", color="cyan")

    try:
        if str(config.source_mode).strip().lower() == "zmq_source":
            run_zmq_source_bridge_service_decoupled(
                config,
                verbose=verbose,
                save_json=save_json,
                result_callback=publisher.publish,
            )
        else:
            run_bridge_service(
                config,
                verbose=verbose,
                save_json=save_json,
                result_callback=publisher.publish,
            )
    finally:
        publisher.close()


def _has_robotaction_config(config) -> bool:
    if bool(getattr(config, "robotaction_enabled", False)):
        return True
    return bool(
        str(getattr(config, "robotaction_data_dir", "")).strip()
        or str(getattr(config, "robotaction_templates_dir", "")).strip()
        or str(getattr(config, "robotaction_graphs_dir", "")).strip()
    )


def _apply_robotaction_data_config(config) -> None:
    if not _has_robotaction_config(config):
        return

    paths = resolve_robotaction_data_paths(
        project_root=Path.cwd(),
        data_dir=(config.robotaction_data_dir or None),
        templates_dir=(config.robotaction_templates_dir or None),
        graphs_dir=(config.robotaction_graphs_dir or None),
        test_box_file=config.robotaction_test_box_file,
        graph_info_file=config.robotaction_graph_info_file,
    )
    if not paths.test_box_path.exists():
        raise FileNotFoundError(
            f"multi_zmq_pub_bridge robotaction test_box not found: {paths.test_box_path}"
        )
    if not paths.graph_info_path.exists():
        raise FileNotFoundError(
            f"multi_zmq_pub_bridge robotaction graph_info not found: {paths.graph_info_path}"
        )

    template_names = load_template_names(paths.test_box_path)
    graph_targets = load_graph_targets(paths.graph_info_path)
    merged_names = list(dict.fromkeys([*graph_targets, *template_names]))

    if not config.prompts and template_names:
        config.prompts = template_names
    if not config.obj_id_map and merged_names:
        config.obj_id_map = {name: index + 1 for index, name in enumerate(merged_names)}


MULTI_ZMQ_PUB_BRIDGE = BridgeDefinition(
    kind="multi_zmq_pub_bridge",
    description=(
        "Dual-branch bridge: routes RGB to siglip2 and RGB-D through sam3->flowpose, "
        "then publishes siglip2 results and tf-style poses over ZMQ PUB."
    ),
    load_config=lambda config_path: load_profiled_bridge_config(
        config_path,
        BridgeProfile(
            kind="multi_zmq_pub_bridge",
            description="multi_zmq_pub_bridge",
            aliases=("zmq_pub_dual", "siglip2_flowpose_pub"),
            mutate_config=_mutate_multi_zmq_pub_bridge_config,
        ),
    ),
    run=_run_multi_zmq_pub_bridge,
    aliases=("zmq_pub_dual", "siglip2_flowpose_pub"),
)
