#!/usr/bin/env python3
"""[9.21-QDU-SIM] Verify the QDU link declares attitude stale instead of silently reusing it.

The emulator publishes live quaternions for a while and then stops publishing while keeping the
serial device open.  A correct link must flip to stale on its own: the consumer must stop trusting
the last quaternion, and a read-only consumer must still transmit nothing while stale.

This is the safety property behind the "no bytes without fresh IMU" rule.  It is a wire-level test,
not auto-aim acceptance, and nothing here commands a gimbal.
"""

import pathlib
import subprocess
import sys
import tempfile
import time


CONFIG_TEMPLATE = (
    "qdu_communication:\n"
    '  device: "{device}"\n'
    "  baud_rate: 115200\n"
    "  tx_enabled: false\n"
    "  reconnect_interval_ms: 100\n"
    "  read_timeout_ms: 10\n"
    "  stale_timeout_ms: 250\n"
    '  default_mode: "auto_aim"\n'
    "  default_bullet_speed: 21.7\n"
    '  quaternion_topics: ["ahrs_quaternion", "gimbal_quat"]\n'
)

STALE_TIMEOUT_MS = 250
STALL_AFTER_S = 2.0
CONSUMER_SECONDS = 6
EMULATOR_SECONDS = 12


def wait_for_device(path: pathlib.Path, process: subprocess.Popen, timeout: float = 20.0) -> str:
    """Read the pseudo-terminal path the emulator published for us to consume."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                f"qdu_board_simulator exited with code {process.returncode} before publishing")
        try:
            text = path.read_text(encoding="utf-8").strip()
        except OSError:
            text = ""
        if text.startswith("/dev/"):
            return text
        time.sleep(0.05)
    raise RuntimeError(f"timed out waiting for {path} to contain a pseudo-terminal path")


def parse_fields(line: str) -> dict:
    """Return the `key=value` fields of a monitor line as a dict."""
    return dict(item.split("=", 1) for item in line.split() if "=" in item)


def status_lines(output: str):
    """Every per-second monitor line, oldest first."""
    return [line for line in output.splitlines() if line.startswith("serial_open=")]


def require(condition: bool, message: str, failures: list):
    if not condition:
        failures.append(message)


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: qdu_stall_sim_test.py QDU_BOARD_SIMULATOR QDU_LINK_MONITOR", file=sys.stderr)
        return 1
    simulator, monitor = sys.argv[1], sys.argv[2]

    with tempfile.TemporaryDirectory(prefix="sp25-qdu-stall-") as directory:
        root = pathlib.Path(directory)
        device_file = root / "pty-path.txt"
        config = root / "qdu.yaml"

        emulator = subprocess.Popen(
            [simulator, "--device-file", str(device_file), "--seconds", str(EMULATOR_SECONDS),
             "--stall-after", str(STALL_AFTER_S)],
            cwd=root,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True)
        consumer = None
        try:
            device = wait_for_device(device_file, emulator)
            print(f"emulated device: {device}")
            config.write_text(CONFIG_TEMPLATE.format(device=device), encoding="utf-8")

            consumer = subprocess.Popen(
                [monitor, str(config), "--seconds", str(CONSUMER_SECONDS)],
                cwd=root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True)
            try:
                consumer_output, _ = consumer.communicate(timeout=30)
            finally:
                if consumer.poll() is None:
                    consumer.kill()
                    consumer.wait()

            emulator_output, _ = emulator.communicate(timeout=EMULATOR_SECONDS + 10)
        finally:
            for process in (consumer, emulator):
                if process is not None and process.poll() is None:
                    process.kill()
                    process.wait()

        print("--- qdu_link_monitor ---")
        print(consumer_output.strip())
        print("--- qdu_board_simulator ---")
        print(emulator_output.strip())

        failures = []
        if consumer is not None and consumer.returncode != 0:
            failures.append(f"qdu_link_monitor exited {consumer.returncode}, expected 0")
        if emulator.returncode != 0:
            failures.append(f"qdu_board_simulator exited {emulator.returncode}, expected 0")

        lines = status_lines(consumer_output)
        require(lines, "qdu_link_monitor printed no per-second status lines", failures)

        live = [line for line in lines if parse_fields(line).get("imu_fresh") == "1"]
        stale = [line for line in lines if parse_fields(line).get("imu_fresh") == "0"]
        require(
            live, "the link never reported a fresh attitude, so the live phase was not observed",
            failures)
        require(
            stale,
            "the link never reported a stale attitude, so it would silently reuse the last "
            "quaternion", failures)

        for line in live:
            fields = parse_fields(line)
            require(
                int(fields.get("quaternions", 0)) > 0,
                f"a line reported imu_fresh=1 with no decoded quaternion: {line}", failures)

        if stale:
            # The last stale line must show the age growing past the configured bound, proving the
            # gate is time-based rather than a one-shot flag.
            last_age = float(parse_fields(stale[-1]).get("att_age_ms", -1))
            require(
                last_age > STALE_TIMEOUT_MS,
                f"last stale attitude age {last_age} ms is not past the {STALE_TIMEOUT_MS} ms "
                "bound, so staleness is not time-based", failures)

        result = {}
        for line in consumer_output.splitlines():
            fields = line.split()
            if fields and fields[0] == "result":
                result = dict(item.split("=", 1) for item in fields[1:] if "=" in item)
        require(
            result.get("tx_commands") == "0",
            f"qdu_link_monitor transmitted {result.get('tx_commands')} commands while RX-only",
            failures)

        emulator_result = {}
        for line in emulator_output.splitlines():
            fields = line.split()
            if fields and fields[0] == "result":
                emulator_result = dict(item.split("=", 1) for item in fields[1:] if "=" in item)
        require(
            emulator_result.get("rx_bytes") == "0",
            f"the read-only consumer wrote {emulator_result.get('rx_bytes')} bytes back",
            failures)

        if failures:
            print("\nFAILURES:", file=sys.stderr)
            for failure in failures:
                print(f"  - {failure}", file=sys.stderr)
            return 1
        print(
            f"\nPASS: live attitude decoded, then declared stale past {STALE_TIMEOUT_MS} ms "
            "with zero bytes transmitted")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
