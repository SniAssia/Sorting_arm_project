"""SEVEN capacity-aware color sorter (new optional controller).

Original_Source_Code/RaspberryPi/color_sorter_original.py is preserved unchanged.
This controller adds persistent 0..3 occupancy counters, a pending-operation lock,
per-slot approach/drop coordinates and operator-verified placement. Run --help.

The Arduino offers NO measured joint-position, grasp, or bin-occupancy feedback.
A serial 'Color route complete' message means commands ended, not proof of placement.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SLOTS = ROOT / "config" / "slots.json"
DEFAULT_STATE = ROOT / "runtime" / "occupancy.json"
COLORS = ("RED", "GREEN", "BLUE")
DESTINATIONS = {"RED": ("AR", "DR"), "GREEN": ("AG", "DG"), "BLUE": ("AB", "DB")}
HSV_RANGES = {
    "RED": (((0, 100, 80), (10, 255, 255)), ((170, 100, 80), (179, 255, 255))),
    "GREEN": (((35, 70, 70), (85, 255, 255)),),
    "BLUE": (((90, 70, 70), (135, 255, 255)),),
}
ROI = (220, 140, 420, 340)
MIN_COVERAGE = 20.0  # pixel coverage (%), not a probability


class SafetyError(RuntimeError):
    """Raised when a physical action cannot safely proceed."""


def load_slots(path: Path = DEFAULT_SLOTS) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or data.get("units") != "cm":
        raise SafetyError("Slot configuration must be schema_version=1, units=cm")
    bins = data.get("bins")
    if not isinstance(bins, dict) or set(bins) != set(COLORS):
        raise SafetyError("Configuration requires exactly RED, GREEN and BLUE bins")
    for color in COLORS:
        entries = bins[color]
        if not isinstance(entries, list) or len(entries) != 3:
            raise SafetyError(f"{color}: exactly three slots are required")
        for index, item in enumerate(entries, 1):
            if item is None:
                continue  # physically uncalibrated; blocked if selected
            if not isinstance(item, dict) or set(item) != {"above", "drop"}:
                raise SafetyError(f"{color} slot {index}: expected 'above' and 'drop' coordinates")
            for name in ("above", "drop"):
                point = item[name]
                if (not isinstance(point, list) or len(point) != 3 or
                    any(isinstance(v, bool) or not isinstance(v, (float, int)) or
                        not math.isfinite(v) for v in point)):
                    raise SafetyError(f"{color} slot {index}: invalid {name} [x,y,z] coordinates")
                validate_workspace(point, f"{color} slot {index} {name}")
            if item["above"][2] <= item["drop"][2]:
                raise SafetyError(f"{color} slot {index}: approach must be above drop height")
    return data


def fk(b: float, s: float, e: float) -> tuple[float, float, float]:
    br, sr, er = (math.radians(v) for v in (b, s, e))
    h = 12.0 * math.cos(sr) + 12.0 * math.cos(sr + er - math.pi)
    return (h * math.sin(br), h * math.cos(br),
            12.0 * math.sin(sr) + 12.0 * math.sin(sr + er - math.pi))


def validate_workspace(point: list[float], label: str) -> None:
    """Match the Arduino's conservative IK rule; NOT a collision check."""
    x, y, z = point
    if x < 1.0 or z < -6.0:
        raise SafetyError(f"{label}: violates Arduino's x>=1, z>=-6 cm guards")
    r = math.hypot(x, y)
    d = math.hypot(r, z)
    if d < 3 or d > 23:
        raise SafetyError(f"{label}: distance {d:.2f} cm outside Arduino's [3,23]")
    ce = (12**2 + 12**2 - d**2) / (2 * 12 * 12)
    ca = (12**2 + d**2 - 12**2) / (2 * 12 * d)
    b = math.degrees(math.atan2(x, y))
    s = math.degrees(math.atan2(z, r) + math.acos(max(-1.0, min(1.0, ca))))
    e = math.degrees(math.acos(max(-1.0, min(1.0, ce))))
    if any(v < 0 or v > 180 for v in (b, s, e)):
        raise SafetyError(f"{label}: inverse kinematics violates 0..180 degree software limits")
    if math.dist(fk(b, s, e), (x, y, z)) >= 0.1:
        raise SafetyError(f"{label}: FK/IK roundtrip failed")


def new_state() -> dict[str, Any]:
    return {"schema_version": 1, "counts": {color: 0 for color in COLORS}, "pending": None}


def load_state(path: Path = DEFAULT_STATE) -> dict[str, Any]:
    if not path.exists():
        return new_state()
    state = json.loads(path.read_text(encoding="utf-8"))
    counts = state.get("counts")
    if (state.get("schema_version") != 1 or not isinstance(counts, dict) or
        set(counts) != set(COLORS) or
        any(type(v) is not int or not 0 <= v <= 3 for v in counts.values())):
        raise SafetyError("Invalid occupancy file; do not reset it without checking the physical bins")
    pending = state.get("pending")
    if pending is not None:
        if (not isinstance(pending, dict) or pending.get("color") not in COLORS or
            type(pending.get("slot")) is not int or
            pending["slot"] != counts[pending["color"]] + 1 or
            "started_utc" not in pending):
            raise SafetyError("Inconsistent pending sort: inspect the physical arm/bin before proceeding")
    return state


def save_state(path: Path, state: dict[str, Any]) -> None:
    """Atomic replacement so a process interruption cannot truncate JSON."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tempname = tempfile.mkstemp(prefix=".occupancy-", suffix=".json", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            json.dump(state, output, indent=2, sort_keys=True)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(tempname, path)
    finally:
        if os.path.exists(tempname):
            os.unlink(tempname)


def select_slot(state: dict[str, Any], slots: dict[str, Any], color: str) -> tuple[int, dict[str, list[float]]]:
    if state["pending"] is not None:
        raise SafetyError("A previous sort is unresolved; inspect it and use --resolve-pending")
    if color not in COLORS:
        raise SafetyError(f"Unknown color {color}")
    count = state["counts"][color]
    if count >= 3:
        raise SafetyError(f"{color} bin FULL (3/3): empty the bin and reset its counter")
    selected = slots["bins"][color][count]
    if selected is None:
        raise SafetyError(f"{color} slot {count+1} is NOT calibrated. Enter measured above/drop coordinates in config/slots.json")
    return count + 1, selected


def mark_pending(state: dict[str, Any], color: str, slot: int) -> None:
    if state["pending"] is not None or state["counts"][color] + 1 != slot:
        raise SafetyError("Invalid pending-operation transition")
    state["pending"] = {"color": color, "slot": slot,
                        "started_utc": dt.datetime.now(dt.timezone.utc).isoformat()}


def resolve_pending(state: dict[str, Any], result: str) -> None:
    pending = state.get("pending")
    if pending is None:
        raise SafetyError("No pending placement to resolve")
    if result not in ("placed", "not-placed"):
        raise SafetyError("Resolve using 'placed' or 'not-placed'")
    if result == "placed":
        color = pending["color"]
        if state["counts"][color] != pending["slot"] - 1:
            raise SafetyError("Occupancy changed while pending; manual inspection required")
        state["counts"][color] += 1
    state["pending"] = None


def print_status(state: dict[str, Any], slots: dict[str, Any]) -> None:
    print("\nPhysical placement counter (operator-confirmed):")
    for color in COLORS:
        count = state["counts"][color]
        ready = sum(v is not None for v in slots["bins"][color])
        print(f"  {color:<5} {count}/3 counted occupied; {ready}/3 drop positions configured")
    if state["pending"]:
        p = state["pending"]
        print(f"  PENDING: {p['color']} slot {p['slot']}. Inspect the hardware; resolve before next run.")
    else:
        print("  Pending operation: none")


def detect_color(capture_dir: Path) -> tuple[str, dict[str, float]]:
    """Same single-frame HSV thresholds and ROI as the preserved Pi script."""
    try:
        import cv2
        import numpy as np
        from picamera2 import Picamera2
    except ImportError as exc:
        raise SafetyError("Requires Raspberry Pi OS with Picamera2, OpenCV and NumPy") from exc
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
    cam.start()
    time.sleep(2)
    try:
        frame = cam.capture_array()
    finally:
        cam.stop()
    x1, y1, x2, y2 = ROI
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        raise SafetyError("ROI is empty")
    # Intentionally matches the operator-tested original script's conversion.
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    percentages = {}
    for color, intervals in HSV_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in intervals:
            mask |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8),
                               np.array(upper, dtype=np.uint8))
        percentages[color] = 100.0 * cv2.countNonZero(mask) / roi.shape[0] / roi.shape[1]
    winner = max(percentages, key=percentages.get)
    if percentages[winner] < MIN_COVERAGE:
        winner = "UNKNOWN"
    capture_dir.mkdir(parents=True, exist_ok=True)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.imwrite(str(capture_dir / "classification_result.jpg"), frame)
    cv2.imwrite(str(capture_dir / "roi.jpg"), roi)
    return winner, percentages


def send_command(arduino: Any, command: str, expected: str, timeout_s: float = 3.0) -> list[str]:
    arduino.write((command + "\n").encode("utf-8"))
    arduino.flush()
    messages = []
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        raw = arduino.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        print("Arduino:", line)
        messages.append(line)
        if line.startswith("ERR:"):
            raise SafetyError(f"Arduino rejected {command}: {line}")
        if (expected in line if isinstance(expected, str) else any(v in line for v in expected)):
            return messages
    raise SafetyError(f"Timed out awaiting '{expected}' after '{command}'")


def execute_route(arduino: Any, color: str, selected: dict[str, list[float]],
                  confirm: Any = input, timeout_s: float = 180.0) -> bool:
    """Return True only for serial command completion, NOT physical success."""
    above_id, drop_id = DESTINATIONS[color]
    for pose_id, point in ((above_id, selected["above"]), (drop_id, selected["drop"])):
        command = "SET " + pose_id + " " + " ".join(f"{v:.4f}" for v in point)
        send_command(arduino, command, "SAVED " + pose_id)
    send_command(arduino, f"CHECK {color}", "Route model check OK")
    print("\nRoute model check passed; this is NOT physical collision validation.")
    answer = confirm("Verify surroundings, supply and gripper clearance. Run route? (yes/no): ").strip().lower()
    if answer != "yes":
        print("Cancelled: no movement commanded.")
        return False
    # Caller must mark pending and persist it *before* any ARM/RUN command.
    return True


def start_and_monitor(arduino: Any, color: str, timeout_s: float = 180.0) -> bool:
    send_command(arduino, "ARM", ("ARMED:", "Already armed"), timeout_s=5.0)
    arduino.write(f"RUN {color}\n".encode("utf-8"))
    arduino.flush()
    deadline = time.monotonic() + timeout_s
    saw_started = False
    while time.monotonic() < deadline:
        raw = arduino.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        print("Arduino:", line)
        if line.startswith("ERR:"):
            raise SafetyError(f"Arduino route error: {line}")
        if "STEP " in line or "MOVING toward model angles" in line:
            saw_started = True
        if "Color route complete" in line:
            if not saw_started:
                raise SafetyError("Received completion without a route-start indication")
            return True
    raise SafetyError("Monitoring timeout. Robot may still be moving; no occupancy count updated")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--status", action="store_true", help="Show counts and configured slots (no hardware)")
    group.add_argument("--verify-config", action="store_true", help="Check reachability of all configured slot targets")
    group.add_argument("--reset", choices=[*COLORS, "ALL"], help="Reset counts only after physically emptying bin(s)")
    group.add_argument("--resolve-pending", choices=["placed", "not-placed"],
                       help="Reconcile interrupted/uncertain physical placement")
    parser.add_argument("--confirm", action="store_true", help="Required with --reset/--resolve-pending")
    parser.add_argument("--color", choices=COLORS, help="Use supplied color (skips camera; intended for controlled testing)")
    parser.add_argument("--dry-run", action="store_true", help="Preview selected slot without serial or state changes")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--slots", type=Path, default=DEFAULT_SLOTS)
    parser.add_argument("--state-file", type=Path, default=DEFAULT_STATE)
    args = parser.parse_args(argv)
    try:
        slots = load_slots(args.slots)
        state = load_state(args.state_file)
        if args.status:
            print_status(state, slots)
            return 0
        if args.verify_config:
            print_status(state, slots)
            if any(v is None for color in COLORS for v in slots["bins"][color]):
                print("WARNING: One or more slots remain uncalibrated. Those slots are blocked.")
            else:
                print("All slot coordinates satisfy SOFTWARE reachability tests; check physical clearance.")
            return 0
        if args.reset:
            if not args.confirm:
                raise SafetyError("Use --confirm only AFTER physically emptying the bin(s)")
            if state["pending"] is not None:
                raise SafetyError("Resolve pending operation before resetting a counter")
            for color in (COLORS if args.reset == "ALL" else (args.reset,)):
                state["counts"][color] = 0
            save_state(args.state_file, state)
            print_status(state, slots)
            return 0
        if args.resolve_pending:
            if not args.confirm:
                raise SafetyError("Use --confirm only after inspecting the actual bin and gripper")
            resolve_pending(state, args.resolve_pending)
            save_state(args.state_file, state)
            print_status(state, slots)
            return 0
        if args.color:
            color = args.color
        else:
            if args.dry_run:
                raise SafetyError("For hardware-free --dry-run, specify --color RED/GREEN/BLUE")
            color, coverage = detect_color(ROOT / "runtime" / "captures")
            for name, percent in coverage.items():
                print(f"{name}: {percent:.1f}% ROI coverage")
            print("Detected:", color)
        if color == "UNKNOWN":
            print("No dominant supported color: no route requested")
            return 0
        slot, selected = select_slot(state, slots, color)
        print(f"{color} => slot {slot}/3 | approach={selected['above']} cm | drop={selected['drop']} cm")
        if args.dry_run:
            print("DRY RUN ONLY: no camera, serial port or occupancy file changed")
            return 0
        try:
            import serial
        except ImportError as exc:
            raise SafetyError("Install pyserial on Raspberry Pi to communicate with Arduino") from exc
        with serial.Serial(args.port, args.baud, timeout=0.2) as arduino:
            # Opening USB serial commonly resets the Arduino; SETs must be re-sent each session.
            time.sleep(3)
            arduino.reset_input_buffer()
            if not execute_route(arduino, color, selected):
                return 0
            mark_pending(state, color, slot)
            save_state(args.state_file, state)
            print(f"PENDING {color} slot {slot} persisted before movement.")
            complete = start_and_monitor(arduino, color)
        if complete:
            print("\nArduino reports command completion, not physically verified placement.")
            response = input("Visually confirmed object placed in intended slot? (yes/no): ").strip().lower()
            if response == "yes":
                resolve_pending(state, "placed")
                save_state(args.state_file, state)
                print(f"Confirmed: {color} occupancy is now {state['counts'][color]}/3")
            else:
                print("Counter unchanged. Operation remains PENDING until inspected.")
                print("Run --resolve-pending placed|not-placed --confirm after inspection.")
        return 0
    except (SafetyError, OSError, ValueError, json.JSONDecodeError) as exc:
        print("SAFETY STOP:", exc, file=sys.stderr)
        print("No automatic counter increment. Inspect hardware and --status before retrying.", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
