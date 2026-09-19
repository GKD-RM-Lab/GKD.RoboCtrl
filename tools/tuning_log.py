#!/usr/bin/env python3
"""Receive legacy-compatible remote telemetry into time_s,name,value CSV.

Optional workstation receiver: sends no commands. Importing the decoder opens
no socket. Capture is explicit, bounded and never truncates existing logs.
"""

import argparse
import csv
import math
from pathlib import Path
import socket
import struct
import sys
import time


def name_id(name):
    result = 2166136261
    for value in name.encode("utf-8"):
        result = ((result ^ value) * 16777619) & 0xFFFFFFFF
    return result


def decode_datagram(data, names):
    """Validate all frames before committing names; return samples and texts."""
    if not data or len(data) > 65507:
        raise ValueError("empty or oversized telemetry datagram")
    pending = dict(names)
    samples, messages = [], []
    offset = 0
    while offset < len(data):
        if len(data) - offset < 3:
            raise ValueError("truncated frame header")
        length, kind = struct.unpack_from("<HB", data, offset)
        if length < 3 or offset + length > len(data):
            raise ValueError("invalid frame length")
        payload = data[offset + 3:offset + length]
        if kind == 0:
            if len(payload) < 5 or payload[4] == 0 or len(payload) != 5 + payload[4]:
                raise ValueError("invalid registration length")
            ident = struct.unpack_from("<I", payload)[0]
            name = payload[5:].decode("utf-8")
            if any(ord(char) < 32 for char in name) or name_id(name) != ident:
                raise ValueError("invalid telemetry name or hash")
            if ident in pending and pending[ident] != name:
                raise ValueError("telemetry name hash collision")
            if ident not in pending and len(pending) >= 4096:
                raise ValueError("telemetry name limit exceeded")
            pending[ident] = name
        elif kind == 1:
            if len(payload) != 12:
                raise ValueError("invalid value length")
            ident, value = struct.unpack("<Id", payload)
            if ident not in pending or not math.isfinite(value):
                raise ValueError("unregistered or nonfinite telemetry value")
            samples.append((pending[ident], value))
        elif kind in (2, 3):
            if len(payload) < 2 or len(payload) != 2 + struct.unpack_from("<H", payload)[0]:
                raise ValueError("invalid text length")
            messages.append((kind, payload[2:].decode("utf-8")))
        else:
            raise ValueError("unknown telemetry frame type")
        offset += length
    names.clear()
    names.update(pending)
    return samples, messages


def capture(args):
    with Path(args.output).open("x", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(("time_s", "name", "value"))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind((args.bind, args.port))
            receiver.settimeout(0.25)
            start = time.monotonic()
            names = {}
            count = 0
            rejected = 0
            while time.monotonic() - start < args.duration and count < args.max_samples:
                try:
                    data, peer = receiver.recvfrom(65535)
                except socket.timeout:
                    continue
                if args.peer and peer[0] != args.peer:
                    continue
                try:
                    samples, messages = decode_datagram(data, names)
                except (ValueError, UnicodeError):
                    rejected += 1
                    continue
                timestamp = time.monotonic() - start
                for name, value in samples[:args.max_samples - count]:
                    writer.writerow((f"{timestamp:.9f}", name, repr(value)))
                    count += 1
                for kind, message in messages:
                    print(f"telemetry text {kind}: {message!r}", file=sys.stderr)
                output.flush()
            print(f"Captured {count} samples; rejected {rejected} datagrams.", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1", help="choose an interface explicitly for a robot")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--peer", help="accept one robot IPv4 address only")
    parser.add_argument("--duration", type=float, default=30.0, help="capture duration in seconds")
    parser.add_argument("--max-samples", type=int, default=100000)
    parser.add_argument("--output", required=True, help="new CSV path; existing files are never truncated")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not math.isfinite(args.duration) or args.duration <= 0 or args.max_samples <= 0:
        parser.error("port, duration and max-samples must be positive and valid")
    try:
        capture(args)
    except (OSError, KeyboardInterrupt) as error:
        parser.exit(1, f"capture stopped: {error}\n")


if __name__ == "__main__":
    main()
