from __future__ import annotations

import argparse
import json
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import mujoco
import mujoco.viewer
from loop_rate_limiters import RateLimiter


_HERE = Path(__file__).resolve().parent
_REPO_ROOT = _HERE.parent
_FK_XML = _REPO_ROOT / "DOGlove_meshes" / "DOGlove-v3.xml"
_DEFAULT_CAPTURE = _REPO_ROOT / "recordings" / "udp_capture_20260328_075825.jsonl"

_JOINT_NAMES = [
    "thumb_bend_1",
    "thumb_bend_2",
    "thumb_split",
    "thumb_mcp",
    "index_bend_1",
    "index_bend_2",
    "index_split",
    "middle_bend_1",
    "middle_bend_2",
    "middle_split",
    "ring_bend_1",
    "ring_bend_2",
    "ring_split",
    "pinky_bend_1",
    "pinky_bend_2",
    "pinky_split",
    "thumb_bend_3",
    "index_bend_3",
    "middle_bend_3",
    "ring_bend_3",
    "pinky_bend_3",
]

_FK_SITE_NAMES = [
    "index_tip_site",
    "middle_tip_site",
    "ring_tip_site",
    "pinky_tip_site",
    "thumb_tip_site",
]


@dataclass(frozen=True)
class ReplayEvent:
    t_ns: int
    stream: str
    values: Tuple[float, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay a captured glove+servo UDP JSONL recording in MuJoCo."
    )
    parser.add_argument(
        "--capture",
        type=Path,
        default=_DEFAULT_CAPTURE,
        help="Path to udp_record.py JSONL capture file.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=100.0,
        help="Viewer loop frequency.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier. 1.0 = real-time.",
    )
    parser.add_argument(
        "--loop",
        dest="loop",
        action="store_true",
        default=True,
        help="Loop playback (default: on).",
    )
    parser.add_argument(
        "--no-loop",
        dest="loop",
        action="store_false",
        help="Play once and stop.",
    )
    return parser.parse_args()


def _decode_values(packet: dict) -> Optional[Tuple[float, ...]]:
    stream = packet.get("stream")
    float_values = packet.get("float32_le")
    if isinstance(float_values, list):
        values = tuple(float(v) for v in float_values)
    else:
        payload_hex = packet.get("payload_hex")
        if not isinstance(payload_hex, str):
            return None
        payload = bytes.fromhex(payload_hex)
        if len(payload) % 4 != 0 or len(payload) == 0:
            return None
        values = struct.unpack("<" + ("f" * (len(payload) // 4)), payload)

    if stream == "joints" and len(values) != 16:
        return None
    if stream == "servo" and len(values) != 5:
        return None
    if stream not in {"joints", "servo"}:
        return None
    return tuple(values)


def load_capture(capture_path: Path) -> List[ReplayEvent]:
    if not capture_path.exists():
        raise FileNotFoundError(f"Capture file not found: {capture_path}")

    events: List[ReplayEvent] = []
    with capture_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("type") != "packet":
                continue

            values = _decode_values(obj)
            if values is None:
                continue
            t_ns = int(obj.get("t_ns", 0))
            stream = str(obj.get("stream"))
            events.append(ReplayEvent(t_ns=t_ns, stream=stream, values=values))

    if not events:
        raise RuntimeError(f"No usable packet events in capture: {capture_path}")

    events.sort(key=lambda e: e.t_ns)
    t0 = events[0].t_ns
    return [ReplayEvent(t_ns=e.t_ns - t0, stream=e.stream, values=e.values) for e in events]


def convert_to_urdf_joint(
    joint_angles: Sequence[float], servo_angles: Sequence[float]
) -> Tuple[float, ...]:
    urdf_joint_angles = [0.0] * 21

    urdf_joint_angles[0] = math.radians(-(joint_angles[0]-90)) # 0 - thumb_dip
    urdf_joint_angles[1] = math.radians(joint_angles[1]-270) # 1 - thumb_pip
    urdf_joint_angles[2] = math.radians(joint_angles[2]-180) # 2 - thumb_mcp_s
    urdf_joint_angles[3] = math.radians(joint_angles[3]-270) # 3 - thumb_mcp_r

    urdf_joint_angles[4] = math.radians(-(joint_angles[4]-90)) # 4 - index_dip
    urdf_joint_angles[5] = math.radians(joint_angles[5]-270) # 5 - index_pip
    urdf_joint_angles[6] = math.radians(-(joint_angles[6]-180)) # 6 - index_mcp_s

    urdf_joint_angles[7] = math.radians(-(joint_angles[7]-90)) # 7 - middle_dip
    urdf_joint_angles[8] = math.radians(joint_angles[8]-270) # 8 - middle_pip
    urdf_joint_angles[9] = math.radians(-(joint_angles[9]-180)) # 9 - middle_mcp_s

    urdf_joint_angles[10] = math.radians(-(joint_angles[10]-90)) # 10 - ring_dip
    urdf_joint_angles[11] = math.radians(joint_angles[11]-270) # 11 - ring_pip
    urdf_joint_angles[12] = math.radians(-(joint_angles[12]-180)) # 12 - ring_mcp_s

    urdf_joint_angles[13] = math.radians(-(joint_angles[13]-90)) # 13 - little_dip
    urdf_joint_angles[14] = math.radians(joint_angles[14]-270) # 14 - little_pip
    urdf_joint_angles[15] = math.radians(-(joint_angles[15]-180)) # 15 - little_mcp_s

    urdf_joint_angles[16] = math.radians(-(servo_angles[0]-182.2)) # 16 - thumb_mcp_b
    urdf_joint_angles[17] = math.radians(-(servo_angles[1]-165.2)) # 17 - index_mcp_b
    urdf_joint_angles[18] = math.radians(-(servo_angles[2]-150.29)) # 18 - middle_mcp_b
    urdf_joint_angles[19] = math.radians(-(servo_angles[3]-151.44)) # 19 - ring_mcp_b
    urdf_joint_angles[20] = math.radians(-(servo_angles[4]-145.9)) # 20 - little_mcp_b

    return tuple(urdf_joint_angles)


def resolve_joint_ids(model: mujoco.MjModel) -> List[int]:
    joint_ids = []
    for joint_name in _JOINT_NAMES:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        joint_ids.append(joint_id)
    return joint_ids


def validate_fk_sites(model: mujoco.MjModel) -> None:
    for site_name in _FK_SITE_NAMES:
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)


def main() -> None:
    args = parse_args()
    if args.speed <= 0:
        raise ValueError("--speed must be > 0")

    capture = args.capture.expanduser().resolve()
    events = load_capture(capture)
    duration_ns = events[-1].t_ns

    if not _FK_XML.exists():
        raise FileNotFoundError(f"Model XML not found: {_FK_XML}")
    model = mujoco.MjModel.from_xml_path(_FK_XML.as_posix())
    data = mujoco.MjData(model)
    joint_ids = resolve_joint_ids(model)
    validate_fk_sites(model)
    rate = RateLimiter(frequency=args.fps)

    print(f"Loaded {len(events)} events from {capture}")
    print(f"Capture duration: {duration_ns / 1e9:.3f}s")
    print(f"Playback speed: {args.speed}x, loop={args.loop}")

    latest_joints: Optional[Tuple[float, ...]] = None
    latest_servo: Optional[Tuple[float, ...]] = None

    with mujoco.viewer.launch_passive(model=model, data=data) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        mujoco.mj_forward(model, data)

        replay_start_ns = time.monotonic_ns()
        idx = 0

        while viewer.is_running():
            elapsed_ns = int((time.monotonic_ns() - replay_start_ns) * args.speed)
            target_t_ns = elapsed_ns

            if duration_ns > 0 and target_t_ns > duration_ns:
                if args.loop:
                    replay_start_ns = time.monotonic_ns()
                    target_t_ns = 0
                    idx = 0
                else:
                    break

            while idx < len(events) and events[idx].t_ns <= target_t_ns:
                event = events[idx]
                if event.stream == "joints":
                    latest_joints = event.values
                elif event.stream == "servo":
                    latest_servo = event.values
                idx += 1

            if latest_joints is not None and latest_servo is not None:
                urdf_joints = convert_to_urdf_joint(latest_joints, latest_servo)
                for i, joint_id in enumerate(joint_ids):
                    data.ctrl[joint_id] = urdf_joints[i]
                mujoco.mj_forward(model, data)

            mujoco.mj_step(model, data)
            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
