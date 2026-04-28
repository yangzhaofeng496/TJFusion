from __future__ import annotations

import json
import re
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml


CAMERA_IN_BASE_XYZ = np.array([0.0925, 0.0325, 1.2660], dtype=np.float64)
CAMERA_IN_BASE_RPY = np.array([-2.3562, 0.0, -1.5708], dtype=np.float64)


def _normalize_text(value: str | None) -> str:
    if not value:
        return ""
    text = str(value).strip().lower()
    text = re.sub(r"^[a-z]\d+\s*:\s*", "", text)
    text = re.sub(r"[.。]+$", "", text)
    text = re.sub(r"(?:_\d+)+$", "", text)
    text = re.sub(r"[_\s]+", " ", text)
    return text.strip()


def _coerce_float_list(values: Any, *, min_len: int = 0, fill: float = 0.0) -> list[float]:
    if isinstance(values, (int, float)):
        values = [values]
    if not isinstance(values, (list, tuple)):
        values = []
    out = [float(item) for item in values]
    while len(out) < min_len:
        out.append(fill)
    return out


def _transform_to_pose(transform: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    t = transform.get("translation", {}) if isinstance(transform, dict) else {}
    q = transform.get("rotation", {}) if isinstance(transform, dict) else {}
    pos = np.array(
        [float(t.get("x", 0.0)), float(t.get("y", 0.0)), float(t.get("z", 0.0))],
        dtype=np.float64,
    )
    quat = np.array(
        [
            float(q.get("x", 0.0)),
            float(q.get("y", 0.0)),
            float(q.get("z", 0.0)),
            float(q.get("w", 1.0)),
        ],
        dtype=np.float64,
    )
    norm = np.linalg.norm(quat)
    if norm <= 1e-8:
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    else:
        quat = quat / norm
    return pos, quat


def _apply_relative_pose(base_pos: np.ndarray, base_quat: np.ndarray, rel_pose: list[float]) -> dict[str, Any]:
    rel = np.array(rel_pose, dtype=np.float64)
    rel_pos = rel[:3]
    rel_quat = rel[3:]
    rel_norm = np.linalg.norm(rel_quat)
    if rel_norm <= 1e-8:
        rel_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    else:
        rel_quat = rel_quat / rel_norm

    out_pos = _quat_rotate_vector(base_quat, rel_pos) + base_pos
    out_quat = _quat_multiply(base_quat, rel_quat)
    out_norm = np.linalg.norm(out_quat)
    if out_norm > 1e-8:
        out_quat = out_quat / out_norm
    else:
        out_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return {
        "position": {"x": float(out_pos[0]), "y": float(out_pos[1]), "z": float(out_pos[2])},
        "orientation": {
            "x": float(out_quat[0]),
            "y": float(out_quat[1]),
            "z": float(out_quat[2]),
            "w": float(out_quat[3]),
        },
    }


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


def _camera_pose_to_base_pose(
    pos_in_camera: np.ndarray,
    quat_in_camera: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    # URDF gives camera pose in base_link (parent=base_link, child=camera_rgb_link).
    # Compose: T_base_target = T_base_camera * T_camera_target.
    q_base_camera = _rpy_to_quaternion(*CAMERA_IN_BASE_RPY)
    q_base_camera = q_base_camera / max(np.linalg.norm(q_base_camera), 1e-12)
    p_base_target = _quat_rotate_vector(q_base_camera, pos_in_camera) + CAMERA_IN_BASE_XYZ
    q_base_target = _quat_multiply(q_base_camera, quat_in_camera)
    q_base_target = q_base_target / max(np.linalg.norm(q_base_target), 1e-12)
    return p_base_target, q_base_target


def _convert_action_poses_camera_to_base(action: dict[str, Any]) -> dict[str, Any]:
    poses = action.get("poses")
    if not isinstance(poses, list):
        return action
    camera_poses = []
    base_poses = []
    for pose in poses:
        if not isinstance(pose, dict):
            continue
        pos_dict = pose.get("position", {})
        ori_dict = pose.get("orientation", {})
        pos = np.array(
            [
                float(pos_dict.get("x", 0.0)),
                float(pos_dict.get("y", 0.0)),
                float(pos_dict.get("z", 0.0)),
            ],
            dtype=np.float64,
        )
        quat = np.array(
            [
                float(ori_dict.get("x", 0.0)),
                float(ori_dict.get("y", 0.0)),
                float(ori_dict.get("z", 0.0)),
                float(ori_dict.get("w", 1.0)),
            ],
            dtype=np.float64,
        )
        quat_norm = np.linalg.norm(quat)
        if quat_norm <= 1e-8:
            quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        else:
            quat = quat / quat_norm
        camera_pose = {
            "position": {"x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])},
            "orientation": {
                "x": float(quat[0]),
                "y": float(quat[1]),
                "z": float(quat[2]),
                "w": float(quat[3]),
            },
        }
        base_pos, base_quat = _camera_pose_to_base_pose(pos, quat)
        base_pose = {
            "position": {
                "x": float(base_pos[0]),
                "y": float(base_pos[1]),
                "z": float(base_pos[2]),
            },
            "orientation": {
                "x": float(base_quat[0]),
                "y": float(base_quat[1]),
                "z": float(base_quat[2]),
                "w": float(base_quat[3]),
            },
        }
        camera_poses.append(camera_pose)
        base_poses.append(base_pose)

    if base_poses:
        action["poses"] = base_poses
    if camera_poses:
        action["poses_camera_rgb_link"] = camera_poses
        action["camera_frame_id"] = "camera_rgb_link"
    action["base_poses"] = base_poses
    action["base_frame_id"] = "base_link"
    action["frame_id"] = "base_link"
    return action


@dataclass(slots=True)
class _Step:
    target: str
    action_name: str
    arm: str
    speed: float
    correction_mode: str


class RobotActionJsonRuntime:
    """
    Pure-Python robotaction runtime.
    Input: siglip/tf payloads from bridge.
    Output: JSON action message for ZMQ topic /action.
    """

    def __init__(
        self,
        *,
        template_path: Path,
        graph_path: Path,
        stable_frames: int = 2,
    ) -> None:
        with template_path.open("r", encoding="utf-8") as handle:
            template_doc = yaml.safe_load(handle) or {}
        self._templates: dict[str, Any] = dict(template_doc.get("templates", {}) or {})

        with graph_path.open("r", encoding="utf-8") as handle:
            graph_doc = json.load(handle) or {}
        self._state_to_actions: dict[str, Any] = {}
        for node in graph_doc.get("nodes", []) or []:
            if not isinstance(node, dict):
                continue
            state = _normalize_text(node.get("state_description"))
            if not state:
                continue
            self._state_to_actions[state] = node.get("next_action")

        self._recent_states: deque[str] = deque(maxlen=max(1, int(stable_frames)))
        self._active_state: str | None = None
        self._active_steps: list[_Step] = []
        self._active_index: int = 0
        self._last_emit_key: tuple[str, int] | None = None

    def _to_steps(self, actions: Any) -> list[_Step]:
        if isinstance(actions, str) and _normalize_text(actions) == "home":
            return [_Step(target="home", action_name="home", arm="both", speed=1.0, correction_mode="forward")]
        if not isinstance(actions, list):
            return []
        out: list[_Step] = []
        for item in actions:
            if not isinstance(item, dict):
                continue
            out.append(
                _Step(
                    target=str(item.get("target", "")).strip(),
                    action_name=str(item.get("action_name", "step")).strip(),
                    arm=str(item.get("arm", "right")).strip() or "right",
                    speed=float(item.get("speed", 1.0)),
                    correction_mode=str(item.get("correction_mode", "none")).strip() or "none",
                )
            )
        return out

    def _update_state(self, siglip_payload: dict[str, Any] | None) -> None:
        if not isinstance(siglip_payload, dict):
            return
        if "ok" in siglip_payload and not bool(siglip_payload.get("ok")):
            return
        state = _normalize_text(siglip_payload.get("best_category"))
        if not state:
            return
        self._recent_states.append(state)
        if len(self._recent_states) < self._recent_states.maxlen:
            return
        values = list(self._recent_states)
        if len(set(values)) != 1:
            return
        stable = values[-1]
        if self._active_state is not None:
            return
        actions = self._state_to_actions.get(stable)
        steps = self._to_steps(actions)
        if not steps:
            return
        self._active_state = stable
        self._active_steps = steps
        self._active_index = 0
        self._last_emit_key = None

    def _select_transform(self, tf_payload: dict[str, Any] | None, target: str) -> dict[str, Any] | None:
        if not isinstance(tf_payload, dict):
            return None
        target_norm = _normalize_text(target)
        transforms = tf_payload.get("transforms")
        if not isinstance(transforms, list):
            return None
        candidates: list[dict[str, Any]] = []
        for item in transforms:
            if not isinstance(item, dict):
                continue
            child = _normalize_text(item.get("child_frame_id"))
            if child == target_norm:
                candidates.append(item)
        if not candidates:
            return None
        candidates.sort(key=lambda item: float((item.get("translation") or {}).get("y", 0.0)))
        return candidates[0]

    def _build_action_from_template(self, step: _Step, transform: dict[str, Any]) -> dict[str, Any] | None:
        template = self._templates.get(step.target)
        if not isinstance(template, dict):
            return None
        blocks = template.get(step.action_name, [])
        if isinstance(blocks, dict):
            blocks = [blocks]
        if not isinstance(blocks, list) or not blocks:
            return None

        base_pos, base_quat = _transform_to_pose(transform)
        poses: list[dict[str, Any]] = []
        constraints = [1.0, 1.0, 1.0]
        gripper_value: list[float] = []
        times: list[float] = []
        for block in blocks:
            if not isinstance(block, dict):
                continue
            rel_poses = block.get("pose_relative", [])
            if isinstance(rel_poses, (list, tuple)) and rel_poses and isinstance(rel_poses[0], (int, float)):
                rel_poses = [rel_poses]
            if not isinstance(rel_poses, list):
                continue
            constraints = _coerce_float_list(block.get("rotation_constraint", [1, 1, 1]), min_len=3, fill=1.0)[:3]
            gripper_value = _coerce_float_list(block.get("gripper_state", []))
            times = _coerce_float_list(block.get("time", []))
            for rel in rel_poses:
                if not isinstance(rel, (list, tuple)) or len(rel) < 7:
                    continue
                poses.append(_apply_relative_pose(base_pos, base_quat, list(rel[:7])))
        if not poses:
            return None
        return {
            "name": step.action_name,
            "arm": step.arm,
            "poses": poses,
            "constraints": constraints,
            "speed": float(step.speed),
            "gripper_value": gripper_value,
            "time": times,
            "target": step.target,
        }

    def build_action_payload(
        self,
        *,
        siglip_payload: dict[str, Any] | None,
        tf_payload: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        self._update_state(siglip_payload)
        if self._active_state is None:
            return None
        if self._active_index >= len(self._active_steps):
            self._active_state = None
            self._active_steps = []
            self._active_index = 0
            return None

        step = self._active_steps[self._active_index]
        emit_key = (self._active_state, self._active_index)
        if self._last_emit_key == emit_key:
            return None

        if _normalize_text(step.action_name) == "home" or _normalize_text(step.target) == "home":
            action = {
                "name": "home",
                "arm": step.arm or "both",
                "poses": [],
                "constraints": [100.0, 100.0, 100.0],
                "speed": float(step.speed),
                "gripper_value": [0.0, 0.0],
                "time": [0.0, 0.1],
                "target": "home",
            }
        else:
            selected = self._select_transform(tf_payload, step.target)
            if selected is None:
                return None
            action = self._build_action_from_template(step, selected)
            if action is None:
                return None
            action = _convert_action_poses_camera_to_base(action)

        payload = {
            "state": self._active_state,
            "step_idx": int(self._active_index + 1),
            "total_steps": int(len(self._active_steps)),
            "action": action,
        }
        self._last_emit_key = emit_key
        self._active_index += 1
        if self._active_index >= len(self._active_steps):
            self._active_state = None
            self._active_steps = []
            self._active_index = 0
        return payload
