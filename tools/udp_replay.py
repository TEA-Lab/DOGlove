#!/usr/bin/env python3
"""Replay UDP packets previously captured by tools/udp_record.py."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path
from typing import Dict, List

# Make project-root modules importable when running this file as a script.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from udp_receiver import udp_ip, udp_port_joint, udp_port_servo


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay a JSONL UDP capture to joints/servo UDP ports."
    )
    parser.add_argument("--capture", type=Path, help="Path to JSONL capture file.")
    parser.add_argument("--host", default=udp_ip, help="Replay target host.")
    parser.add_argument("--joints-port", type=int, default=udp_port_joint)
    parser.add_argument("--servo-port", type=int, default=udp_port_servo)
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Replay speed multiplier. 1.0 = real-time, 2.0 = 2x faster.",
    )
    parser.add_argument(
        "--loop",
        type=int,
        default=10,
        help="Number of replay loops.",
    )
    return parser.parse_args()


def load_packets(path: Path) -> List[Dict]:
    packets: List[Dict] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("type") == "packet":
                packets.append(obj)
    packets.sort(key=lambda x: x["t_ns"])
    return packets


def main() -> None:
    args = parse_args()
    if args.speed <= 0:
        raise ValueError("--speed must be > 0")
    if args.loop <= 0:
        raise ValueError("--loop must be > 0")

    packets = load_packets(args.capture)
    if not packets:
        raise RuntimeError(f"No packet records found in {args.capture}")

    port_map = {
        "joints": args.joints_port,
        "servo": args.servo_port,
    }
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Replaying {len(packets)} packets from {args.capture}")
    print(
        f"  target joints={args.host}:{args.joints_port}, "
        f"servo={args.host}:{args.servo_port}, speed={args.speed}x, loop={args.loop}"
    )

    for loop_idx in range(args.loop):
        sent = 0
        t0 = time.monotonic_ns()
        first_t = packets[0]["t_ns"]

        for packet in packets:
            stream = packet["stream"]
            if stream not in port_map:
                continue

            due_ns = int((packet["t_ns"] - first_t) / args.speed)
            now_ns = time.monotonic_ns() - t0
            wait_ns = due_ns - now_ns
            if wait_ns > 0:
                time.sleep(wait_ns / 1e9)

            payload = bytes.fromhex(packet["payload_hex"])
            tx.sendto(payload, (args.host, port_map[stream]))
            sent += 1

        print(f"  loop {loop_idx + 1}/{args.loop}: sent {sent} packets")

    tx.close()
    print("Replay done.")


if __name__ == "__main__":
    main()
