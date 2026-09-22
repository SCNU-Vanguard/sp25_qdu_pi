#!/usr/bin/env python3
"""[9.21-QDU] Exercise the Linux serial worker across a PTY unplug/replug cycle."""

import os
import pathlib
import pty
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


def quaternion_packet() -> bytes:
    # QDU/Eigen raw ABI is x,y,z,w. This is the identity quaternion.
    payload = struct.pack("<ffff", 0.0, 0.0, 0.0, 1.0)
    timestamp = int(time.monotonic_ns() // 1000) & 0xFFFFFFFFFFFF
    header = bytearray(16)
    header[0] = 0x5A
    header[1:4] = len(payload).to_bytes(3, "little")
    header[4:8] = crc32(b"ahrs_quaternion").to_bytes(4, "little")
    header[8:14] = timestamp.to_bytes(6, "little")
    header[14] = 1
    header[15] = crc8(header[:15])
    packet = bytes(header) + payload
    return packet + bytes([crc8(packet)])


def create_device(link: pathlib.Path):
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    link.symlink_to(slave_name)
    os.close(slave)
    return master


def send_for(master: int, duration: float):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        os.write(master, quaternion_packet())
        time.sleep(0.01)


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: qdu_hotplug_sim_test.py QDU_LINK_MONITOR", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory(prefix="sp25-qdu-") as directory:
        root = pathlib.Path(directory)
        device = root / "devc-usb"
        config = root / "qdu.yaml"
        config.write_text(
            "qdu_communication:\n"
            f'  device: "{device}"\n'
            "  baud_rate: 115200\n"
            "  tx_enabled: false\n"
            "  reconnect_interval_ms: 100\n"
            "  read_timeout_ms: 10\n"
            "  stale_timeout_ms: 250\n"
            '  default_mode: "auto_aim"\n'
            "  default_bullet_speed: 21.7\n"
            '  quaternion_topics: ["ahrs_quaternion", "gimbal_quat"]\n',
            encoding="utf-8",
        )

        first_master = create_device(device)
        process = subprocess.Popen(
            [sys.argv[1], str(config), "--seconds", "6", "--expect-reconnect"],
            cwd=root,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            send_for(first_master, 1.8)
            os.close(first_master)
            device.unlink(missing_ok=True)
            time.sleep(1.0)
            second_master = create_device(device)
            try:
                send_for(second_master, 3.2)
                output, _ = process.communicate(timeout=10)
            finally:
                os.close(second_master)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()

        print(output)
        if process.returncode != 0:
            print(f"monitor exit code: {process.returncode}", file=sys.stderr)
            return process.returncode
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
