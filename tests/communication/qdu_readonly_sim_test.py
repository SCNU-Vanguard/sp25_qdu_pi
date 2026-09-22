#!/usr/bin/env python3
"""[9.21-QDU-SIM] Prove the read-only QDU link receives live attitude from the emulated C board.

The real QDU-Future board currently returns zero application bytes to the Pi, so Tracker, Solver
and Aimer have never executed against live attitude.  This test pairs the pseudo-terminal emulator
with the *production* serial worker and asserts the whole receive path: packets decode, quaternions
arrive, no CRC or validity error occurs, and the read-only consumer transmits nothing at all.

It is a wire-level test, not auto-aim acceptance.  Nothing here commands a gimbal or a launcher,
and a `fire_notify=true` byte on the wire is a hard safety failure.
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

CONSUMER_SECONDS = 6
EMULATOR_SECONDS = 14


def wait_for_device(path: pathlib.Path, process: subprocess.Popen, timeout: float = 10.0) -> str:
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


def parse_result(output: str) -> dict:
    """Return the `result key=value ...` line as a dict, or an empty dict when absent."""
    for line in output.splitlines():
        fields = line.split()
        if fields and fields[0] == "result":
            return dict(item.split("=", 1) for item in fields[1:] if "=" in item)
    return {}


def require(results: dict, key: str, source: str, failures: list, equals=None, minimum=None):
    """Assert one field of a `result` line, recording a readable message on mismatch."""
    if key not in results:
        failures.append(f"{source}: result line has no {key}= field")
        return
    try:
        value = int(results[key])
    except ValueError:
        failures.append(f"{source}: {key}={results[key]!r} is not an integer")
        return
    if equals is not None and value != equals:
        failures.append(f"{source}: {key}={value}, expected exactly {equals}")
    if minimum is not None and value < minimum:
        failures.append(f"{source}: {key}={value}, expected >= {minimum}")


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: qdu_readonly_sim_test.py QDU_BOARD_SIMULATOR QDU_LINK_MONITOR", file=sys.stderr)
        return 1
    simulator, monitor = sys.argv[1], sys.argv[2]

    with tempfile.TemporaryDirectory(prefix="sp25-qdu-sim-") as directory:
        root = pathlib.Path(directory)
        device_file = root / "pty-path.txt"
        config = root / "qdu.yaml"

        # [9.21-QDU-SIM] The emulator outlives the consumer on purpose: it must still be publishing
        # when the consumer exits, so that "zero bytes received" really means the read-only entry
        # never opened its write path rather than the link being torn down underneath it.
        emulator = subprocess.Popen(
            [simulator, "--device-file", str(device_file), "--seconds", str(EMULATOR_SECONDS),
             "--motion", "yaw-sweep", "--motion-rate", "0.8", "--expect-silent"],
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
            failures.append(
                f"qdu_link_monitor exited {consumer.returncode}, expected 0")
        if emulator.returncode != 0:
            failures.append(f"qdu_board_simulator exited {emulator.returncode}, expected 0")

        consumer_result = parse_result(consumer_output)
        emulator_result = parse_result(emulator_output)

        # The production worker must decode live attitude cleanly.
        require(consumer_result, "quaternions", "qdu_link_monitor", failures, minimum=1)
        require(consumer_result, "ahrs_q", "qdu_link_monitor", failures, minimum=1)
        require(consumer_result, "packets", "qdu_link_monitor", failures, minimum=1)
        require(consumer_result, "invalid_q", "qdu_link_monitor", failures, equals=0)
        require(consumer_result, "crc_header", "qdu_link_monitor", failures, equals=0)
        require(consumer_result, "crc_payload", "qdu_link_monitor", failures, equals=0)

        # A read-only consumer must put nothing on the wire at all.
        require(consumer_result, "tx_commands", "qdu_link_monitor", failures, equals=0)
        require(emulator_result, "rx_bytes", "qdu_board_simulator", failures, equals=0)
        require(emulator_result, "rx_target_euler", "qdu_board_simulator", failures, equals=0)
        require(emulator_result, "rx_fire_notify", "qdu_board_simulator", failures, equals=0)
        require(emulator_result, "rx_fire_true", "qdu_board_simulator", failures, equals=0)
        require(emulator_result, "rx_bad_size", "qdu_board_simulator", failures, equals=0)

        # The emulator itself must have published a usable stream for the whole run.
        require(emulator_result, "sent_quaternions", "qdu_board_simulator", failures, minimum=100)

        if failures:
            print("\nFAILURES:", file=sys.stderr)
            for failure in failures:
                print(f"  - {failure}", file=sys.stderr)
            return 1
        print("\nPASS: live quaternions decoded, no CRC/validity error, zero bytes transmitted")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
