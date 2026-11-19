"""Hardware-free tests of the optional Raspberry Pi three-slot controller."""
import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Original_Source_Code" / "RaspberryPi"))
import sort_with_slots as ss  # noqa: E402


class FakeArduino:
    def __init__(self):
        self.commands = []
        self.responses = []

    def write(self, payload):
        command = payload.decode().strip()
        self.commands.append(command)
        bits = command.split()
        if bits[0] == "SET":
            self.responses.append(f"SAVED {bits[1]} b=105.0 s=73.0 e=60.0\n".encode())
        elif bits[0] == "CHECK":
            self.responses.append(b"Route model check OK (NOT a collision check).\n")
        elif command == "ARM":
            self.responses.append(b"ARMED: joints 105/73/60; gripper 90\n")
        elif bits[0] == "RUN":
            self.responses += [b"STEP 1/8 AP\n", b"MOVING toward model angles: b=15\n",
                               b"Color route complete (gripper close/release commanded).\n"]
        else:
            self.responses.append(b"ERR: unexpected command\n")

    def flush(self):
        pass

    def readline(self):
        return self.responses.pop(0) if self.responses else b""


def fully_calibrated(slots):
    """Synthetic test points, never to be used on actual hardware."""
    result = copy.deepcopy(slots)
    for color in ss.COLORS:
        for i in (1, 2):
            original = result["bins"][color][0]
            # Small Y offsets within the software IK bounds. Synthetic only.
            above = list(original["above"])
            drop = list(original["drop"])
            above[1] += 0.2 * i
            drop[1] += 0.2 * i
            result["bins"][color][i] = {"above": above, "drop": drop}
            for point in (above, drop):
                ss.validate_workspace(point, "synthetic test")
    return result


class SlotSorterTests(unittest.TestCase):
    def setUp(self):
        self.slots = ss.load_slots()

    def test_new_counts_and_first_slots(self):
        state = ss.new_state()
        self.assertEqual(state["counts"], dict.fromkeys(ss.COLORS, 0))
        for color in ss.COLORS:
            slot, positions = ss.select_slot(state, self.slots, color)
            self.assertEqual(slot, 1)
            self.assertEqual(len(positions["above"]), 3)

    def test_slots_2_and_3_locked_without_physical_calibration(self):
        state = ss.new_state()
        state["counts"]["RED"] = 1
        with self.assertRaisesRegex(ss.SafetyError, "NOT calibrated"):
            ss.select_slot(state, self.slots, "RED")
        state["counts"]["RED"] = 3
        with self.assertRaisesRegex(ss.SafetyError, "FULL"):
            ss.select_slot(state, self.slots, "RED")

    def test_three_per_color_and_fourth_blocked(self):
        slots = fully_calibrated(self.slots)
        state = ss.new_state()
        for color in ss.COLORS:
            for spot in (1, 2, 3):
                selected, _ = ss.select_slot(state, slots, color)
                self.assertEqual(spot, selected)
                ss.mark_pending(state, color, selected)
                with self.assertRaisesRegex(ss.SafetyError, "unresolved"):
                    ss.select_slot(state, slots, color)
                ss.resolve_pending(state, "placed")
            self.assertEqual(state["counts"][color], 3)
            with self.assertRaisesRegex(ss.SafetyError, "FULL"):
                ss.select_slot(state, slots, color)

    def test_not_placed_does_not_advance(self):
        state = ss.new_state()
        ss.mark_pending(state, "GREEN", 1)
        ss.resolve_pending(state, "not-placed")
        self.assertEqual(state["counts"]["GREEN"], 0)
        self.assertIsNone(state["pending"])

    def test_pending_state_survives_reload(self):
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp) / "runtime" / "occupancy.json"
            state = ss.new_state()
            ss.mark_pending(state, "BLUE", 1)
            ss.save_state(file, state)
            reloaded = ss.load_state(file)
            self.assertEqual(reloaded["pending"]["color"], "BLUE")
            ss.resolve_pending(reloaded, "placed")
            ss.save_state(file, reloaded)
            self.assertEqual(ss.load_state(file)["counts"]["BLUE"], 1)

    def test_fake_serial_protocol_set_before_check_arm_run(self):
        fake = FakeArduino()
        approved = ss.execute_route(fake, "RED", self.slots["bins"]["RED"][0],
                                    confirm=lambda _: "yes")
        self.assertTrue(approved)
        self.assertTrue(ss.start_and_monitor(fake, "RED", timeout_s=1.0))
        self.assertEqual([p.split()[0] for p in fake.commands],
                         ["SET", "SET", "CHECK", "ARM", "RUN"])
        self.assertEqual(fake.commands[0].split()[1], "AR")
        self.assertEqual(fake.commands[1].split()[1], "DR")

    def test_fake_serial_user_declines(self):
        fake = FakeArduino()
        approved = ss.execute_route(fake, "GREEN", self.slots["bins"]["GREEN"][0],
                                    confirm=lambda _: "no")
        self.assertFalse(approved)
        self.assertEqual([p.split()[0] for p in fake.commands], ["SET", "SET", "CHECK"])

    def test_cli_dry_run_does_not_write_state(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp) / "state.json"
            self.assertEqual(ss.main(["--color", "RED", "--dry-run", "--state-file", str(state)]), 0)
            self.assertFalse(state.exists())

    def test_malformed_coordinate_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            bad = copy.deepcopy(self.slots)
            bad["bins"]["RED"][1] = {"above": [100, 100, 100], "drop": [100,100,95]}
            file = Path(tmp) / "bad.json"
            file.write_text(json.dumps(bad))
            with self.assertRaises(ss.SafetyError):
                ss.load_slots(file)


if __name__ == "__main__":
    unittest.main()
