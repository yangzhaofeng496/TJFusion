from __future__ import annotations

import json
import threading
from collections import Counter, deque
from typing import Any

import numpy as np

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
            tf_payload = _convert_tf_payload_camera_to_base(tf_payload)

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


CAMERA_IN_BASE_XYZ = np.array([0.0925, 0.0325, 1.2660], dtype=np.float64)
CAMERA_IN_BASE_RPY = np.array([-2.3562, 0.0, -1.5708], dtype=np.float64)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float64,
    )


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _quat_rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    v_quat = np.array([v[0], v[1], v[2], 0.0], dtype=np.float64)
    rotated = _quat_multiply(_quat_multiply(q, v_quat), _quat_conjugate(q))
    return rotated[:3]


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )


def _convert_tf_payload_camera_to_base(tf_payload: dict[str, Any]) -> dict[str, Any]:
    transforms = tf_payload.get("transforms")
    if not isinstance(transforms, list) or not transforms:
        return tf_payload

    q_base_camera = _rpy_to_quaternion(*CAMERA_IN_BASE_RPY)
    q_base_camera = q_base_camera / max(np.linalg.norm(q_base_camera), 1e-12)

    output = []
    for item in transforms:
        if not isinstance(item, dict):
            continue
        t = item.get("translation", {})
        r = item.get("rotation", {})
        p_camera = np.array(
            [float(t.get("x", 0.0)), float(t.get("y", 0.0)), float(t.get("z", 0.0))],
            dtype=np.float64,
        )
        q_camera = np.array(
            [
                float(r.get("x", 0.0)),
                float(r.get("y", 0.0)),
                float(r.get("z", 0.0)),
                float(r.get("w", 1.0)),
            ],
            dtype=np.float64,
        )
        qn = np.linalg.norm(q_camera)
        if qn <= 1e-8:
            q_camera = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        else:
            q_camera = q_camera / qn

        p_base = _quat_rotate_vector(q_base_camera, p_camera) + CAMERA_IN_BASE_XYZ
        q_base = _quat_multiply(q_base_camera, q_camera)
        q_base = q_base / max(np.linalg.norm(q_base), 1e-12)

        output.append(
            {
                "frame_id": "base_link",
                "child_frame_id": item.get("child_frame_id"),
                "translation": {"x": float(p_base[0]), "y": float(p_base[1]), "z": float(p_base[2])},
                "rotation": {"x": float(q_base[0]), "y": float(q_base[1]), "z": float(q_base[2]), "w": float(q_base[3])},
            }
        )
    tf_payload["transforms"] = output
    tf_payload["frame_id"] = "base_link"
    return tf_payload
