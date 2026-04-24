from __future__ import annotations

import json
import threading
from collections import Counter, deque
from typing import Any

from fusion_docker.bridge_pose import build_tf_payload_from_flowpose_result
from fusion_docker.console import print_status

try:
    import zmq
except ImportError:  # pragma: no cover
    zmq = None


def require_zmq() -> Any:
    if zmq is None:
        raise RuntimeError("ZMQ result publishing requires pyzmq.")
    return zmq


class BridgeResultPublisher:
    def __init__(
        self,
        addr: str,
        *,
        frame_id: str = "camera_rgb_link",
        siglip_topic: str = "/siglip2/result",
        tf_topic: str = "/tf",
        action_topic: str = "/action",
        siglip_vote_window: int = 1,
        robotaction_runtime: Any | None = None,
    ) -> None:
        zmq_module = require_zmq()
        self._frame_id = frame_id
        self._siglip_topic = siglip_topic
        self._tf_topic = tf_topic
        self._action_topic = action_topic
        self._siglip_vote_window = max(1, int(siglip_vote_window))
        self._siglip_recent_categories: deque[str] = deque(maxlen=self._siglip_vote_window)
        self._cached_siglip_payload: dict[str, Any] | None = None
        self._robotaction_runtime = robotaction_runtime
        self._publish_lock = threading.Lock()
        self._context = zmq_module.Context.instance()
        self._socket = self._context.socket(zmq_module.PUB)
        self._socket.setsockopt(zmq_module.SNDHWM, 1)
        self._socket.setsockopt(zmq_module.LINGER, 0)
        self._socket.bind(addr)
        self.addr = addr

    def _select_smoothed_best_category(self, current: Any) -> Any:
        if not isinstance(current, str):
            return current
        normalized = current.strip()
        if not normalized:
            return current
        if self._siglip_vote_window <= 1:
            return normalized

        self._siglip_recent_categories.append(normalized)
        counts = Counter(self._siglip_recent_categories)
        max_count = max(counts.values(), default=0)
        winners = {name for name, freq in counts.items() if freq == max_count}
        for name in reversed(self._siglip_recent_categories):
            if name in winners:
                return name
        return normalized

    def publish(self, result: dict[str, Any]) -> None:
        with self._publish_lock:
            siglip_result = result.get("siglip2", {})
            if not isinstance(siglip_result, dict):
                siglip_result = {}
            frame_id = result.get(
                "frame_id",
                result.get("source_meta", {}).get("frame_id")
                if isinstance(result.get("source_meta"), dict)
                else None,
            )
            tf_payload = {
                "frame_id": frame_id,
                "transforms": build_tf_payload_from_flowpose_result(result, frame_id=self._frame_id),
            }

            siglip_payload: dict[str, Any] | None = None
            if siglip_result:
                best_category = self._select_smoothed_best_category(siglip_result.get("best_category"))
                best_similarity = siglip_result.get("best_similarity")
                siglip_ok = bool(siglip_result.get("ok", False))
                siglip_payload = {
                    "frame_id": frame_id,
                    "ok": siglip_ok,
                    "best_category": best_category,
                    "best_similarity": best_similarity,
                }
                self._socket.send_string(
                    f"{self._siglip_topic} {json.dumps(siglip_payload, ensure_ascii=False)}"
                )
                if siglip_ok and str(best_category or "").strip():
                    # Keep latest valid siglip state and reuse it when later TF arrives.
                    self._cached_siglip_payload = dict(siglip_payload)

            tf_count = len(tf_payload["transforms"])
            if tf_count > 0:
                self._socket.send_string(
                    f"{self._tf_topic} {json.dumps(tf_payload, ensure_ascii=False)}"
                )

            action_payload: dict[str, Any] | None = None
            action_skip_reason = ""
            action_input_siglip = siglip_payload
            using_cached_siglip = False
            if (
                self._robotaction_runtime is not None
                and (action_input_siglip is None or not bool(action_input_siglip.get("ok", False)))
                and tf_count > 0
                and isinstance(self._cached_siglip_payload, dict)
            ):
                action_input_siglip = dict(self._cached_siglip_payload)
                using_cached_siglip = True
                print_status(
                    "ROBACTION",
                    (
                        f"use_cached_siglip_for_action "
                        f"best_category={action_input_siglip.get('best_category')} "
                        f"tf_count={tf_count}"
                    ),
                    color="cyan",
                )
            if self._robotaction_runtime is not None:
                try:
                    action_payload = self._robotaction_runtime.build_action_payload(
                        siglip_payload=action_input_siglip,
                        tf_payload=tf_payload,
                    )
                except Exception as exc:
                    print_status("ROBACTION", f"build_action_payload failed: {exc}", color="red")
                    action_payload = None
            if action_payload is not None:
                self._socket.send_string(
                    f"{self._action_topic} {json.dumps(action_payload, ensure_ascii=False)}"
                )
                action_name = (
                    str(action_payload.get("action", {}).get("name", "")).strip()
                    if isinstance(action_payload.get("action"), dict)
                    else ""
                )
                print_status(
                    "ROBACTION",
                    (
                        f"emit topic={self._action_topic} "
                        f"state={action_payload.get('state')} "
                        f"step={action_payload.get('step_idx')}/{action_payload.get('total_steps')} "
                        f"action={action_name or '<unknown>'} "
                        f"using_cached_siglip={using_cached_siglip}"
                    ),
                    color="green",
                )
            else:
                if self._robotaction_runtime is None:
                    action_skip_reason = "runtime_disabled(auto_run=false_or_no_robotaction_config)"
                elif action_input_siglip is None:
                    action_skip_reason = "siglip_missing"
                elif not bool(action_input_siglip.get("ok", False)):
                    action_skip_reason = "siglip_not_ok"
                elif tf_count <= 0:
                    action_skip_reason = "tf_missing"
                else:
                    action_skip_reason = (
                        "runtime_no_action(state_not_stable_or_not_mapped_or_target_missing)"
                    )
                print_status(
                    "ROBACTION",
                    (
                        f"skip topic={self._action_topic} reason={action_skip_reason} "
                        f"best_category={(action_input_siglip or {}).get('best_category')} "
                        f"tf_count={tf_count}"
                    ),
                    color="yellow",
                )

            if siglip_payload is None and tf_count <= 0:
                return

            print_status(
                "PUB",
                (
                    f"published frame_id={frame_id} "
                    f"siglip_topic={self._siglip_topic} "
                    f"siglip_ok={(siglip_payload or {}).get('ok', False)} "
                    f"best_category={(siglip_payload or {}).get('best_category')} "
                    f"tf_topic={self._tf_topic} "
                    f"tf_count={tf_count}"
                    f" action_topic={self._action_topic} "
                    f"action_emitted={bool(action_payload)}"
                ),
                color="green",
            )

    def close(self) -> None:
        try:
            self._socket.close(0)
        except Exception:
            pass
