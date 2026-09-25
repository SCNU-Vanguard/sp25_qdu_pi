"""Exercise circle detection + calibration on known synthetic camera views, without a display.

Usage: python3 tests/calibration/intrinsics_headless_test.py /path/to/calibrate_intrinsics
Requires the distribution's Python OpenCV and numpy (test dependencies only).
"""
import ast
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import socket
import time
import urllib.request

import cv2
import numpy as np


def run(binary, folder, *options):
    result = subprocess.run([binary, str(folder), *options], text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            env={**os.environ, "DISPLAY": ""}, timeout=120)
    return result


def main():
    binary = str(Path(sys.argv[1]).resolve())
    camera = np.array([[820., 0., 360.], [0., 810., 270.], [0., 0., 1.]])
    distortion = np.array([-.04, .012, .001, -.001, 0.])
    angles = np.linspace(0, 2 * math.pi, 80, endpoint=False)
    rng = np.random.default_rng(20260925)
    with tempfile.TemporaryDirectory(prefix="sp25-intrinsics-test-") as tmp:
        folder = Path(tmp)
        (folder / "pattern-used.yaml").write_text(
            "pattern_cols: 10\npattern_rows: 7\ncenter_distance_mm: 18\n")
        for index in range(24):
            # Planar black disks, projected with lens distortion and supersampled for subpixels.
            image = np.full((540 * 3, 720 * 3), 255, dtype=np.uint8)
            rotation = rng.uniform([-.5, -.5, -.25], [.5, .5, .25])
            translation = np.array([-.081 + rng.uniform(-.07, .07),
                                    -.054 + rng.uniform(-.06, .06),
                                    rng.uniform(.48, .82)])
            for row in range(7):
                for col in range(10):
                    disk = np.column_stack((col * .018 + .0045 * np.cos(angles),
                                            row * .018 + .0045 * np.sin(angles),
                                            np.zeros(len(angles))))
                    projected, _ = cv2.projectPoints(disk, rotation, translation, camera, distortion)
                    polygon = np.rint(projected[:, 0] * 3).astype(np.int32)
                    cv2.fillConvexPoly(image, polygon, 0)
            image = cv2.resize(image, (720, 540), interpolation=cv2.INTER_AREA)
            # Number gaps intentionally verify that one missing file does not truncate the dataset.
            assert cv2.imwrite(str(folder / f"{index * 2 + 1:04d}.png"), image)
        (folder / "not-an-original_overlay.png").write_bytes(b"not an input image")
        result = run(binary, folder)
        print(result.stdout)
        assert result.returncode == 0, "synthetic calibration failed"
        report = (folder / "intrinsics.yaml").read_text()
        fields = dict(line.split(": ", 1) for line in report.splitlines()
                      if ": " in line and not line.startswith(" "))
        actual = np.array(ast.literal_eval(fields["camera_matrix"])).reshape(3, 3)
        assert int(fields["accepted_views"]) >= 20
        assert int(fields["image_width"]) == 720 and int(fields["image_height"]) == 540
        assert abs(actual[0, 0] / camera[0, 0] - 1) < .05, actual
        assert abs(actual[1, 1] / camera[1, 1] - 1) < .05, actual
        assert np.linalg.norm(actual[:2, 2] - camera[:2, 2]) < 20, actual
        assert float(fields["rms_px"]) < .6, fields["rms_px"]
        assert len(list((folder / "review").glob("*.png"))) == 24
        # Existing results must survive a repeated invocation byte for byte.
        assert run(binary, folder).returncode != 0
        assert (folder / "intrinsics.yaml").read_text() == report
        # Incorrect sizes and too few usable views must not produce a calibration result.
        for case in ("mixed", "few"):
            bad = folder / case
            bad.mkdir()
            (bad / "pattern-used.yaml").write_bytes((folder / "pattern-used.yaml").read_bytes())
            (bad / "0001.png").write_bytes((folder / "0001.png").read_bytes())
            if case == "mixed":
                cv2.imwrite(str(bad / "0002.png"), np.full((200, 200), 255, np.uint8))
            failure = run(binary, bad)
            assert failure.returncode != 0, failure.stdout
            assert not (bad / "intrinsics.yaml").exists()
            expected = "mixed image sizes" if case == "mixed" else "valid views"
            assert expected in failure.stdout, failure.stdout
        print("PASS: headless recovery, numbered gaps, overlay exclusion, no overwrite, size/count rejection")
        if len(sys.argv) > 2:
            capture = str(Path(sys.argv[2]).resolve())
            config = folder / "test-camera.yaml"
            config.write_text(f'test_image: "{folder / "0001.png"}"\n')
            session = folder / "capture-session"
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            command = [capture, str(config), f'--pattern-config={folder / "pattern-used.yaml"}',
                       f'--output-folder={session}', f'--preview-port={port}',
                       '--interval=1', '--max-samples=2', '--seconds=10']
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, env={**os.environ, "DISPLAY": ""})
            try:
                page = b""
                for _ in range(50):
                    try:
                        page = urllib.request.urlopen(f'http://127.0.0.1:{port}/', timeout=1).read()
                        break
                    except OSError:
                        time.sleep(.05)
                assert b'/frame.jpg' in page
                output, _ = process.communicate(timeout=20)
                assert process.returncode == 0, output
                for filename in ("0001.png", "0002.png"):
                    assert np.array_equal(cv2.imread(str(session / filename)),
                                          cv2.imread(str(folder / "0001.png")))
                assert (session / "camera-used.yaml").read_bytes() == config.read_bytes()
                assert (session / "pattern-used.yaml").exists()
                assert len((session / "frames.csv").read_text().splitlines()) == 3
                repeated = subprocess.run(command, capture_output=True, text=True, timeout=10)
                assert repeated.returncode != 0
                assert 'already exists' in repeated.stderr
                print("PASS: camera-only capture, browser page, original pixels, snapshots, no overwrite")
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    main()
