"""SEVEN capacity-aware color sorter (new optional controller).

Original_Source_Code/RaspberryPi/color_sorter_original.py is preserved unchanged.
Each bin holds three objects at one firmware-defined destination. Counters track
completed command sequences and explicit operator reconciliation. Run --help.

The Arduino offers NO measured joint-position, grasp, or bin-occupancy feedback.
A serial 'Color route complete' message means commands ended, not proof of placement.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys
import tempfile
import time
from contextlib import contextmanager, nullcontext
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SLOTS = ROOT / "config" / "slots.json"
DEFAULT_STATE = ROOT / "runtime" / "occupancy.json"
COLORS = ("RED", "GREEN", "BLUE")
DESTINATIONS = {"RED": "DR", "GREEN": "DG", "BLUE": "DB"}
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
    if not isinstance(data, dict) or data.get("schema_version") != 2:
        raise SafetyError("Use the supplied schema_version=2 bin configuration")
    bins = data.get("bins")
    if not isinstance(bins, dict) or set(bins) != set(COLORS):
        raise SafetyError("Configuration requires exactly RED, GREEN and BLUE bins")
    for color, entry in bins.items():
        if (entry != {"capacity": 3, "destination": DESTINATIONS[color]} or
                type(entry["capacity"]) is not int):
            raise SafetyError(f"{color} requires capacity 3 and firmware destination {DESTINATIONS[color]}")
    return data


@contextmanager
def operation_lock(path: Path):
    """Hold one OS-managed lock across read, motion and commit, including recovery."""
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_name(path.name + ".lock").open("a+b") as handle:
        handle.seek(0)
        try:
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise SafetyError("Another controller operation is active for this occupancy file") from exc
        yield  # Closing the handle releases the lock, including after process exit.


def new_state() -> dict[str, Any]:
    return {"schema_version": 1, "counts": {color: 0 for color in COLORS}, "pending": None}


def load_state(path: Path = DEFAULT_STATE) -> dict[str, Any]:
    if not path.exists():
        return new_state()
    state = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(state, dict) or "pending" not in state:
        raise SafetyError("Invalid occupancy file: missing transaction state")
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
            pending["slot"] > 3 or not isinstance(pending.get("started_utc"), str)):
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
        if os.name == "posix":
            # Flush the directory entry as well as the contents before issuing ARM.
            directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
    finally:
        if os.path.exists(tempname):
            os.unlink(tempname)


def validate_capacity(state: dict[str, Any], config: dict[str, Any], color: str) -> str:
    if state["pending"] is not None:
        raise SafetyError("A previous sort is unresolved; inspect it and use --resolve-pending")
    if color not in COLORS:
        raise SafetyError(f"Unknown color {color}")
    entry = config["bins"][color]
    if state["counts"][color] >= entry["capacity"]:
        raise SafetyError(f"{color} bin FULL (3/3): empty the bin and reset its counter")
    return entry["destination"]


def mark_pending(state: dict[str, Any], color: str) -> None:
    if state["pending"] is not None or state["counts"][color] >= 3:
        raise SafetyError("Invalid pending-operation transition")
    # Schema v1 calls the expected post-commit count "slot". It is an ordinal only.
    state["pending"] = {"color": color, "slot": state["counts"][color] + 1,
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
    print("\nEstimated bin occupancy (completed routes and operator reconciliation):")
    for color in COLORS:
        count = state["counts"][color]
        print(f"  {color:<5} {count}/3 at {slots['bins'][color]['destination']}")
    if state["pending"]:
        p = state["pending"]
        print(f"  PENDING: {p['color']} operation. Inspect the hardware; resolve before next run.")
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


def send_command(arduino: Any, command: str, expected: str | tuple[str, ...], timeout_s: float = 3.0) -> list[str]:
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
        if line.startswith(expected):
            return messages
    raise SafetyError(f"Timed out awaiting '{expected}' after '{command}'")


def execute_route(arduino: Any, color: str, confirm: Any = None) -> bool:
    """Validate the firmware route and request approval before motion."""
    if confirm is None:
        confirm = input
    send_command(arduino, f"CHECK {color}", "Route model check OK")
    print("\nRoute model check passed; this is NOT physical collision validation.")
    answer = confirm("Verify surroundings, supply and gripper clearance. Run route? (yes/no): ").strip().lower()
    if answer != "yes":
        print("Cancelled: no movement commanded.")
        return False
    # Caller must mark pending and persist it *before* any ARM/RUN command.
    return True


def start_and_monitor(arduino: Any, color: str, timeout_s: float = 180.0) -> None:
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
        if line.split() == ["STEP", "1/8", "AP"]:
            saw_started = True
        if line == "Color route complete (gripper close/release commanded).":
            if not saw_started:
                raise SafetyError("Received completion without a route-start indication")
            return
    raise SafetyError("Monitoring timeout. Robot may still be moving; no occupancy count updated")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--dry-run", action="store_true", help="Preview fixed destination without serial or state changes")
    group.add_argument("--status", action="store_true", help="Show counts and fixed bin destinations (no hardware)")
    group.add_argument("--verify-config", action="store_true", help="Validate bin capacities and firmware waypoint names")
    group.add_argument("--reset", choices=[*COLORS, "ALL"], help="Reset counts only after physically emptying bin(s)")
    group.add_argument("--resolve-pending", choices=["placed", "not-placed"],
                       help="Reconcile interrupted/uncertain physical placement")
    parser.add_argument("--confirm", action="store_true", help="Required with --reset/--resolve-pending")
    parser.add_argument("--color", choices=COLORS, help="Use supplied color (skips camera; intended for controlled testing)")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--slots", type=Path, default=DEFAULT_SLOTS)
    parser.add_argument("--state-file", type=Path, default=DEFAULT_STATE)
    args = parser.parse_args(argv)
    try:
        read_only = args.status or args.verify_config or args.dry_run
        lock = nullcontext() if read_only else operation_lock(args.state_file)
        with lock:
            return run(args)
    except (SafetyError, OSError, ValueError, EOFError) as exc:
        print("SAFETY STOP:", exc, file=sys.stderr)
        print("Inspect hardware and --status before retrying.", file=sys.stderr)
        return 2


def run(args: argparse.Namespace) -> int:
    slots = load_slots(args.slots)
    state = load_state(args.state_file)
    if args.status or args.verify_config:
        print_status(state, slots)
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
    destination = validate_capacity(state, slots, color)
    print(f"{color} => {destination}, current occupancy {state['counts'][color]}/3")
    if args.dry_run:
        print("DRY RUN ONLY: no camera, serial port or occupancy file changed")
        return 0
    try:
        import serial
    except ImportError as exc:
        raise SafetyError("Install pyserial on Raspberry Pi to communicate with Arduino") from exc
    with serial.Serial(args.port, args.baud, timeout=0.2) as arduino:
        # Opening USB serial commonly resets the Arduino; wait for firmware startup.
        time.sleep(3)
        arduino.reset_input_buffer()
        if not execute_route(arduino, color):
            return 0
        mark_pending(state, color)
        save_state(args.state_file, state)
        print(f"PENDING {color} persisted before movement.")
        start_and_monitor(arduino, color)
        resolve_pending(state, "placed")
        save_state(args.state_file, state)
        print(f"Command sequence complete. {color} occupancy estimate: {state['counts'][color]}/3.")
        print("Physical deposit has no independent sensor confirmation.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
