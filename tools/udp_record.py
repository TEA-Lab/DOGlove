#!/usr/bin/env python3
"""Record glove and servo UDP streams to a replay-friendly JSONL file."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import selectors
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

# Make project-root modules importable when running this file as a script.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from udp_receiver import udp_ip, udp_port_joint, udp_port_servo


def utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def make_socket(host: str, port: int, timeout_s: float = 0.2) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    sock.bind((host, port))
    sock.settimeout(timeout_s)
    sock.setblocking(False)
    return sock


def decode_float32_le(data: bytes) -> List[float] | None:
    if len(data) % 4 != 0 or len(data) == 0:
        return None
    count = len(data) // 4
    return list(struct.unpack("<" + ("f" * count), data))


def build_default_output() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("recordings") / f"udp_capture_{stamp}.jsonl"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            f"Record UDP packets from glove ({udp_port_joint}) and servo ({udp_port_servo})."
        )
    )
    parser.add_argument("--host", default=udp_ip, help="UDP bind host.")
    parser.add_argument("--joints-port", type=int, default=udp_port_joint)
    parser.add_argument("--servo-port", type=int, default=udp_port_servo)
    parser.add_argument("--duration", type=float, default=60.0, help="Seconds to record.")
    parser.add_argument(
        "--output",
        type=Path,
        default=build_default_output(),
        help="JSONL output path.",
    )
    parser.add_argument(
        "--max-packet-size",
        type=int,
        default=4096,
        help="Maximum packet size for recvfrom.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    sockets: Dict[str, socket.socket] = {
        "joints": make_socket(args.host, args.joints_port),
        "servo": make_socket(args.host, args.servo_port),
    }

    selector = selectors.DefaultSelector()
    for stream_name, sock in sockets.items():
        selector.register(sock, selectors.EVENT_READ, data=stream_name)

    start_ns = time.monotonic_ns()
    end_ns = start_ns + int(args.duration * 1e9)
    counters = {"joints": 0, "servo": 0}

    print(f"Recording UDP for {args.duration:.1f}s...")
    print(f"  joints: {args.host}:{args.joints_port}")
    print(f"  servo : {args.host}:{args.servo_port}")
    print(f"  output: {args.output}")

    with args.output.open("w", encoding="utf-8") as f:
        meta = {
            "type": "meta",
            "version": 1,
            "created_utc": utc_now_iso(),
            "host": args.host,
            "ports": {
                "joints": args.joints_port,
                "servo": args.servo_port,
            },
            "duration_s": args.duration,
        }
        f.write(json.dumps(meta, separators=(",", ":")) + "\n")

        while True:
            now_ns = time.monotonic_ns()
            if now_ns >= end_ns:
                break

            remaining_s = max(0.0, (end_ns - now_ns) / 1e9)
            timeout_s = min(0.2, remaining_s)
            events = selector.select(timeout=timeout_s)

            for key, _ in events:
                stream_name = key.data
                sock = key.fileobj
                while True:
                    try:
                        payload, src = sock.recvfrom(args.max_packet_size)
                    except (BlockingIOError, TimeoutError):
                        break
                    except socket.timeout:
                        break

                    rel_ns = time.monotonic_ns() - start_ns
                    counters[stream_name] += 1
                    decoded = decode_float32_le(payload)
                    record = {
                        "type": "packet",
                        "stream": stream_name,
                        "seq": counters[stream_name],
                        "t_ns": rel_ns,
                        "src_ip": src[0],
                        "src_port": src[1],
                        "nbytes": len(payload),
                        "payload_hex": payload.hex(),
                    }
                    if decoded is not None:
                        record["float32_le"] = decoded
                    f.write(json.dumps(record, separators=(",", ":")) + "\n")

        summary = {
            "type": "summary",
            "finished_utc": utc_now_iso(),
            "captured_packets": counters,
            "total_packets": counters["joints"] + counters["servo"],
        }
        f.write(json.dumps(summary, separators=(",", ":")) + "\n")

    for sock in sockets.values():
        sock.close()
    selector.close()

    print("Done.")
    print(
        f"  packets: joints={counters['joints']}, servo={counters['servo']}, "
        f"total={counters['joints'] + counters['servo']}"
    )


if __name__ == "__main__":
    main()
