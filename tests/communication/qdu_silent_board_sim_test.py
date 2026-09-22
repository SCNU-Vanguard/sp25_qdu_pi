#!/usr/bin/env python3
"""[9.21-QDU-SIM] Cover the "attitude never arrived" path of the QDU link.

The stall test proves the link declares a *stalled* stream stale.  This one covers the other half of
imu_age_ms(): a port that opens cleanly and then never delivers a single quaternion.  That is the
state the real QDU-Future board is in right now, so it deserves an explicit assertion instead of
being inferred from the -1 branch being "obviously correct".

What must hold:
  * the port opens and stays open (serial_open=1, disconnects=0) — so the missing attitude is a
    silent board, not a dead link;
  * imu_age_ms() reports -1 on every sample, which is distinguishable from both a fresh sample and
    a stalled one that reports a large positive age;
  * imu_fresh stays 0, so the consumer never trusts a quaternion it does not have;
  * the read-only consumer still transmits nothing.

qdu_link_monitor deliberately exits 2 when it decodes zero quaternions, so that is the expected
exit code here, not a failure.
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

CONSUMER_SECONDS = 4
EMULATOR_SECONDS = 10


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


def parse_fields(line: str) -> dict:
    """Return the `key=value` fields of a monitor line as a dict."""
    return dict(item.split("=", 1) for item in line.split() if "=" in item)


def parse_result(output: str) -> dict:
    """Return the `result key=value ...` line as a dict, or an empty dict when absent."""
    for line in output.splitlines():
        fields = line.split()
        if fields and fields[0] == "result":
            return dict(item.split("=", 1) for item in fields[1:] if "=" in item)
    return {}


def status_lines(output: str):
    """Every per-second monitor line, oldest first."""
    return [line for line in output.splitlines() if line.startswith("serial_open=")]


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


def require_true(condition: bool, message: str, failures: list):
    """Assert a plain condition, recording a readable message when it does not hold."""
    if not condition:
        failures.append(message)


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: qdu_silent_board_sim_test.py QDU_BOARD_SIMULATOR QDU_LINK_MONITOR",
            file=sys.stderr)
        return 1
    simulator, monitor = sys.argv[1], sys.argv[2]

    with tempfile.TemporaryDirectory(prefix="sp25-qdu-silent-") as directory:
        root = pathlib.Path(directory)
        device_file = root / "pty-path.txt"
        config = root / "qdu.yaml"

        # [9.21-QDU-SIM] --stall-after 0 makes the emulator open the port and never publish a single
        # quaternion, which is exactly the state the real board is in.
        emulator = subprocess.Popen(
            [simulator, "--device-file", str(device_file), "--seconds", str(EMULATOR_SECONDS),
             "--stall-after", "0", "--expect-silent"],
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
        # qdu_link_monitor exits 2 by design when it decodes zero quaternions.
        if consumer is not None and consumer.returncode != 2:
            failures.append(
                f"qdu_link_monitor exited {consumer.returncode}, expected 2 (no quaternions)")
        if emulator.returncode != 0:
            failures.append(f"qdu_board_simulator exited {emulator.returncode}, expected 0")

        lines = status_lines(consumer_output)
        require_true(lines, "qdu_link_monitor printed no per-second status lines", failures)
        for line in lines:
            fields = parse_fields(line)
            require_true(
                fields.get("imu_fresh") == "0",
                f"attitude was reported fresh although nothing was published: {line}", failures)
            require_true(
                fields.get("att_age_ms") == "-1.0000",
                f"att_age_ms should be -1 when no quaternion ever arrived, got "
                f"{fields.get('att_age_ms')!r}: {line}", failures)
            require_true(
                fields.get("serial_open") == "1",
                f"the port should stay open while the board stays silent: {line}", failures)
            require_true(
                fields.get("disconnects") == "0",
                f"the link should not report a disconnect while the board stays silent: {line}",
                failures)

        consumer_result = parse_result(consumer_output)
        emulator_result = parse_result(emulator_output)

        require(consumer_result, "quaternions", "qdu_link_monitor", failures, equals=0)
        require(consumer_result, "ahrs_q", "qdu_link_monitor", failures, equals=0)
        require(consumer_result, "packets", "qdu_link_monitor", failures, equals=0)
        require(consumer_result, "invalid_q", "qdu_link_monitor", failures, equals=0)
        require(consumer_result, "opens", "qdu_link_monitor", failures, equals=1)
        require(consumer_result, "disconnects", "qdu_link_monitor", failures, equals=0)
        require(consumer_result, "tx_commands", "qdu_link_monitor", failures, equals=0)

        require(emulator_result, "sent_quaternions", "qdu_board_simulator", failures, equals=0)
        require(emulator_result, "rx_bytes", "qdu_board_simulator", failures, equals=0)
        require(emulator_result, "rx_fire_true", "qdu_board_simulator", failures, equals=0)

        if failures:
            print("\nFAILURES:", file=sys.stderr)
            for failure in failures:
                print(f"  - {failure}", file=sys.stderr)
            return 1
        print(
            "\nPASS: port stayed open with zero quaternions, imu_age_ms reported -1 throughout, "
            "and nothing was transmitted")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
