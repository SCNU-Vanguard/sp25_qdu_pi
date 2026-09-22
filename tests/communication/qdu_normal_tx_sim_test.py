#!/usr/bin/env python3
"""[9.21-QDU-NORMAL-TX] Verify simultaneous quaternion RX and nonzero CBoard TX."""

import errno
import os
import pathlib
import pty
import select
import struct
import subprocess
import sys
import tempfile
import time


def crc8(data: bytes) -> int:
    value = 0xFF
    for item in data:
        value ^= item
        for _ in range(8):
            value = ((value >> 1) ^ 0x8C) if value & 1 else value >> 1
    return value


def crc32(data: bytes) -> int:
    value = 0xFFFFFFFF
    for item in data:
        value ^= item
        for _ in range(8):
            value = ((value >> 1) ^ 0xEDB88320) if value & 1 else value >> 1
    return value


def pack(topic: bytes, payload: bytes) -> bytes:
    timestamp = int(time.monotonic_ns() // 1000) & 0xFFFFFFFFFFFF
    header = bytearray(16)
    header[0] = 0x5A
    header[1:4] = len(payload).to_bytes(3, "little")
    header[4:8] = crc32(topic).to_bytes(4, "little")
    header[8:14] = timestamp.to_bytes(6, "little")
    header[14] = 1
    header[15] = crc8(header[:15])
    body = bytes(header) + payload
    return body + bytes([crc8(body)])


def extract_packets(buffer: bytearray):
    packets = []
    while True:
        try:
            start = buffer.index(0x5A)
        except ValueError:
            buffer.clear()
            return packets
        if start:
            del buffer[:start]
        if len(buffer) < 16:
            return packets
        if crc8(buffer[:15]) != buffer[15] or buffer[14] != 1:
            del buffer[0]
            continue
        payload_size = int.from_bytes(buffer[1:4], "little")
        packet_size = 17 + payload_size
        if len(buffer) < packet_size:
            return packets
        candidate = bytes(buffer[:packet_size])
        if crc8(candidate[:-1]) != candidate[-1]:
            del buffer[0]
            continue
        packets.append((int.from_bytes(candidate[4:8], "little"), candidate[16:-1]))
        del buffer[:packet_size]


def close_process(process):
    if process.poll() is None:
        process.kill()
        process.wait()


def read_master(master: int):
    try:
        return os.read(master, 4096)
    except OSError as error:
        # A PTY master reports EIO before any process has opened the slave and again after close.
        if error.errno == errno.EIO:
            return b""
        raise


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: qdu_normal_tx_sim_test.py QDU_LINK_MONITOR", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory(prefix="sp25-qdu-tx-") as directory:
        root = pathlib.Path(directory)
        master, slave = pty.openpty()
        device = os.ttyname(slave)
        os.close(slave)
        config = root / "qdu.yaml"
        config.write_text(
            "qdu_communication:\n"
            f'  device: "{device}"\n'
            "  baud_rate: 115200\n"
            "  tx_enabled: true\n"
            "  reconnect_interval_ms: 100\n"
            "  read_timeout_ms: 10\n"
            "  stale_timeout_ms: 250\n"
            '  default_mode: "auto_aim"\n'
            "  default_bullet_speed: 21.7\n"
            '  quaternion_topics: ["ahrs_quaternion", "gimbal_quat"]\n',
            encoding="utf-8",
        )
        process = subprocess.Popen(
            [sys.argv[1], str(config), "--seconds", "3", "--send-command", "0.25", "-0.125"],
            cwd=root,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        received = bytearray()
        parsed = []
        deadline = time.monotonic() + 3.5
        try:
            while time.monotonic() < deadline and process.poll() is None:
                os.write(master, pack(b"ahrs_quaternion", struct.pack("<ffff", 0, 0, 0, 1)))
                readable, _, _ = select.select([master], [], [], 0.01)
                if readable:
                    received.extend(read_master(master))
                    parsed.extend(extract_packets(received))
            output, _ = process.communicate(timeout=5)
            while True:
                readable, _, _ = select.select([master], [], [], 0)
                if not readable:
                    break
                chunk = read_master(master)
                if not chunk:
                    break
                received.extend(chunk)
            parsed.extend(extract_packets(received))
        finally:
            close_process(process)
            os.close(master)

        print(output)
        if process.returncode != 0:
            raise RuntimeError(f"monitor exit code {process.returncode}")
        target_key = crc32(b"target_euler")
        fire_key = crc32(b"fire_notify")
        targets = [payload for key, payload in parsed if key == target_key]
        fires = [payload for key, payload in parsed if key == fire_key]
        if not targets or not fires:
            raise RuntimeError("missing target_euler or fire_notify")
        if any(payload != b"\0" for payload in fires):
            raise RuntimeError("fire_notify was not false")
        expected = (-0.125, -0.125, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        decoded = [struct.unpack("<9f", payload) for payload in targets if len(payload) == 36]
        if expected not in decoded:
            raise RuntimeError(f"nonzero production command not observed: {decoded}")
        print(f"verified target_packets={len(targets)} fire_packets={len(fires)} nonzero={expected}")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
