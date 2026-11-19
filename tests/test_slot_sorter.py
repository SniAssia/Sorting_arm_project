"""Exercise the production CLI with a scripted serial peer and an isolated state file."""
import io
import json
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack, redirect_stderr, redirect_stdout
from itertools import count
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Original_Source_Code" / "RaspberryPi"))
import sort_with_slots as ss

START = "STEP 1/8  AP"
DONE = "Color route complete (gripper close/release commanded)."


class FakeArduino:
    def __init__(self):
        self.commands = []
        self.queue = []
        self.check = ["Route model check OK (NOT a collision check)."]
        self.arm = ["ARMED: joints 105/73/60; gripper 90"]
        self.route = [START, DONE]
        self.on_write = lambda command: None

    def write(self, data):
        command = data.decode().strip()
        self.commands.append(command)
        self.on_write(command)
        self.queue.extend({"CHECK": self.check, "ARM": self.arm,
                           "RUN": self.route}[command.split()[0]])

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.queue.clear()

    def readline(self):
        return (self.queue.pop(0) + "\n").encode() if self.queue else b""

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass


class BinCapacityTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.path = Path(tmp.name) / "occupancy.json"
        self.fake = FakeArduino()
        self.serial = Mock()
        self.serial.Serial.return_value = self.fake
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch.dict(sys.modules, serial=self.serial))
        stack.enter_context(patch.object(ss.time, "sleep"))
        stack.enter_context(patch.object(ss.time, "monotonic", side_effect=count(step=0.25)))
        self.answer = stack.enter_context(patch("builtins.input", return_value="yes"))
        self.output = stack.enter_context(redirect_stdout(io.StringIO()))
        stack.enter_context(redirect_stderr(self.output))

    def cli(self, *args):
        return ss.main(["--state-file", str(self.path), *args])

    def state(self):
        return ss.load_state(self.path)

    def test_capacity_and_fixed_firmware_destinations(self):
        bins = ss.load_slots()["bins"]
        expected = dict.fromkeys(ss.COLORS, 0)
        for color, dest in zip(ss.COLORS, ("DR", "DG", "DB")):
            self.assertEqual(bins[color], {"capacity": 3, "destination": dest})
            for _ in range(3):
                self.assertEqual(ss.validate_capacity(self.state(), {"bins": bins}, color), dest)
                self.fake.commands.clear()
                self.assertEqual(self.cli("--color", color), 0)
                self.assertEqual(self.fake.commands, [f"CHECK {color}", "ARM", f"RUN {color}"])
                expected[color] += 1
                self.assertEqual(self.state()["counts"], expected)
            self.serial.Serial.reset_mock()
            self.assertEqual(self.cli("--color", color), 2)
            self.assertIn(f"{color} bin FULL", self.output.getvalue())
            self.serial.Serial.assert_not_called()

    def test_camera_dispatch_and_unknown(self):
        with patch.object(ss, "detect_color", side_effect=[("UNKNOWN", {}), ("BLUE", {})]):
            self.assertEqual(self.cli(), 0)
            self.serial.Serial.assert_not_called()
            self.assertEqual(self.cli(), 0)
        self.assertEqual(self.fake.commands, ["CHECK BLUE", "ARM", "RUN BLUE"])

    def test_restart_and_duplicate_completion(self):
        self.fake.route.append(DONE)
        self.assertEqual(self.cli("--color", "RED"), 0)
        self.assertEqual(self.cli("--resolve-pending", "placed", "--confirm"), 2)
        result = subprocess.run([sys.executable, ss.__file__, "--state-file", str(self.path),
                                 "--status"], check=True, text=True, capture_output=True)
        self.assertRegex(result.stdout, r"RED\s+1/3 at DR")
        self.assertIsNone(self.state()["pending"])

    def test_cancel_rejection_and_check_timeout(self):
        self.answer.return_value = "no"
        for response, code in [(self.fake.check, 0), (["ERR: route rejected"], 2), ([], 2)]:
            self.fake.commands.clear()
            self.fake.check = response
            self.assertEqual(self.cli("--color", "GREEN"), code)
            self.assertEqual(self.fake.commands, ["CHECK GREEN"])
            self.assertEqual(self.state(), ss.new_state())

    def test_uncertain_runs_require_explicit_inspection(self):
        for replies in ([START], [DONE], [START, "ERR: movement"], [START, "partial " + DONE]):
            with self.subTest(replies=replies):
                self.fake.route = replies
                self.assertEqual(self.cli("--color", "GREEN"), 2)
                self.assertEqual(self.state()["counts"]["GREEN"], 0)
                self.assertEqual(self.state()["pending"]["color"], "GREEN")
                self.assertEqual(self.cli("--color", "BLUE"), 2)
                self.assertEqual(self.cli("--reset", "ALL", "--confirm"), 2)
                self.assertEqual(self.cli("--resolve-pending", "placed"), 2)
                self.assertEqual(self.cli("--resolve-pending", "not-placed", "--confirm"), 0)

    def test_pending_precedes_arm_and_survives_disconnect(self):
        def disconnect(command):
            if command == "ARM":
                self.assertEqual(self.state()["pending"]["color"], "BLUE")
                raise OSError("USB disconnected")
        self.fake.on_write = disconnect
        self.assertEqual(self.cli("--color", "BLUE"), 2)
        self.assertEqual(self.state()["counts"]["BLUE"], 0)
        self.assertEqual(self.cli("--resolve-pending", "placed", "--confirm"), 0)
        self.assertEqual(self.state()["counts"]["BLUE"], 1)
        self.assertEqual(self.cli("--resolve-pending", "placed", "--confirm"), 2)
        self.assertEqual(self.state()["counts"]["BLUE"], 1)

    def test_failed_commit_leaves_pending_for_recovery(self):
        original_save = ss.save_state

        def fail_commit(path, state):
            if state["pending"] is None:
                raise OSError("Storage unavailable after sequence completion")
            original_save(path, state)

        with patch.object(ss, "save_state", side_effect=fail_commit):
            self.assertEqual(self.cli("--color", "RED"), 2)
        self.assertEqual(self.state()["counts"]["RED"], 0)
        self.assertEqual(self.state()["pending"]["color"], "RED")
        self.assertEqual(self.cli("--resolve-pending", "placed", "--confirm"), 0)
        self.assertEqual(self.state()["counts"]["RED"], 1)

    def test_other_process_cannot_modify_state_during_sort(self):
        def concurrent(command):
            if command.startswith("CHECK") or command.startswith("RUN"):
                for args in (["--reset", "ALL", "--confirm"],
                             ["--resolve-pending", "placed", "--confirm"]):
                    other = subprocess.run([sys.executable, ss.__file__, "--state-file",
                                            str(self.path), *args], capture_output=True, text=True)
                    self.assertEqual(other.returncode, 2)
                    self.assertIn("operation is active", other.stderr)
        self.fake.on_write = concurrent
        self.assertEqual(self.cli("--color", "RED"), 0)
        self.assertEqual(self.state()["counts"]["RED"], 1)

    def test_only_confirmed_reset_affects_selected_bin(self):
        self.cli("--color", "BLUE")
        self.cli("--color", "RED")
        self.assertEqual(self.cli("--reset", "RED"), 2)
        self.assertEqual(self.state()["counts"]["RED"], 1)
        self.assertEqual(self.cli("--reset", "RED", "--confirm"), 0)
        self.assertEqual(self.state()["counts"], {"RED": 0, "GREEN": 0, "BLUE": 1})

    def test_legacy_pending_and_invalid_snapshot(self):
        legacy = {"schema_version": 1, "counts": {"RED": 2, "GREEN": 0, "BLUE": 1},
                  "pending": {"color": "RED", "slot": 3, "started_utc": "2026-01-01T00:00:00Z"}}
        self.path.write_text(json.dumps(legacy))
        self.assertEqual(self.cli("--color", "BLUE"), 2)
        self.assertEqual(self.cli("--resolve-pending", "placed", "--confirm"), 0)
        self.assertEqual(self.state()["counts"], {"RED": 3, "GREEN": 0, "BLUE": 1})
        legacy["pending"]["slot"] = 1
        invalid = json.dumps(legacy)
        self.path.write_text(invalid)
        self.assertEqual(self.cli("--reset", "ALL", "--confirm"), 2)
        self.assertEqual(self.path.read_text(), invalid)

    def test_preview_and_bad_config(self):
        self.assertEqual(self.cli("--color", "BLUE", "--dry-run"), 0)
        self.assertFalse(self.path.exists())
        config = ss.load_slots()
        config["bins"]["BLUE"]["destination"] = "DR"
        file = self.path.with_name("bins.json")
        file.write_text(json.dumps(config))
        self.assertEqual(self.cli("--verify-config", "--slots", str(file)), 2)
        self.serial.Serial.assert_not_called()


if __name__ == "__main__":
    unittest.main()
