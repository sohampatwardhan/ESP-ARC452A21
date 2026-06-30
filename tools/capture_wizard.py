#!/usr/bin/env python3
"""Guided Daikin ARC452A21 IR capture wizard.

The ESP32 firmware must be running the receiver-only capture app. Close
`idf.py monitor` before starting this script because only one process can own
the serial port at a time.

Example:

    python tools/capture_wizard.py --port /dev/cu.usbserial-0001
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

from capture_serial import (
    BEGIN_RE,
    SYMBOL_RE,
    CaptureFormatError,
    LEGACY_CAPTURE_LIMIT,
    write_capture,
)


@dataclass(frozen=True)
class CaptureStep:
    label: str
    setup: str
    button: str
    expected: str
    notes: str = ""


CORE_STEPS = [
    CaptureStep(
        label="power_off",
        setup="Start with operation ON in any stable state.",
        button="Press ON/OFF so the ON icon disappears.",
        expected="Power off frame.",
    ),
    CaptureStep(
        label="cool_64_fan3_hswing_down_at_min",
        setup="Set COOL mode, 64 F, fan speed 3, horizontal swing active.",
        button="Press TEMP DOWN once while already at 64 F.",
        expected="Minimum-temperature DOWN boundary frame.",
        notes="Useful for detecting whether the remote sends an unchanged state at the lower limit.",
    ),
    CaptureStep(
        label="cool_65_fan3_hswing_up_from_min",
        setup="Set COOL mode, 64 F, fan speed 3, horizontal swing active.",
        button="Press TEMP UP once.",
        expected="COOL, 65 F, fan speed 3, horizontal swing active.",
    ),
    CaptureStep(
        label="cool_64_fan3_hswing_down_from_65",
        setup="Set COOL mode, 65 F, fan speed 3, horizontal swing active.",
        button="Press TEMP DOWN once.",
        expected="COOL, 64 F, fan speed 3, horizontal swing active.",
    ),
    CaptureStep(
        label="cool_72_fan3_swing_off",
        setup="Set COOL mode, 71 F, fan speed 3, both swings off.",
        button="Press TEMP UP once.",
        expected="COOL, 72 F, fan speed 3, swings off.",
    ),
    CaptureStep(
        label="cool_90_fan3_swing_off",
        setup="Set COOL mode, 89 F, fan speed 3, both swings off.",
        button="Press TEMP UP once.",
        expected="COOL, 90 F, fan speed 3, swings off.",
    ),
    CaptureStep(
        label="cool_90_fan3_up_at_max",
        setup="Set COOL mode, 90 F, fan speed 3, both swings off.",
        button="Press TEMP UP once while already at 90 F.",
        expected="Maximum-temperature UP boundary frame.",
    ),
    CaptureStep(
        label="cool_18c_fan3_swing_off_down_at_min",
        setup="Switch the remote display to Celsius. Set COOL mode, 18 C, fan speed 3, both swings off.",
        button="Press TEMP DOWN once while already at 18 C.",
        expected="Minimum-temperature DOWN boundary frame in Celsius mode.",
        notes="Validates whether Celsius uses a separate unit flag or a different temperature encoding.",
    ),
    CaptureStep(
        label="cool_19c_fan3_swing_off_up_from_min",
        setup="Set COOL mode, 18 C, fan speed 3, both swings off.",
        button="Press TEMP UP once.",
        expected="COOL, 19 C, fan speed 3, swings off.",
    ),
    CaptureStep(
        label="cool_22c_fan3_swing_off",
        setup="Set COOL mode, 21 C, fan speed 3, both swings off.",
        button="Press TEMP UP once.",
        expected="COOL, 22 C, fan speed 3, swings off.",
    ),
    CaptureStep(
        label="cool_32c_fan3_swing_off",
        setup="Set COOL mode, 31 C, fan speed 3, both swings off.",
        button="Press TEMP UP once.",
        expected="COOL, 32 C, fan speed 3, swings off.",
    ),
    CaptureStep(
        label="cool_32c_fan3_swing_off_up_at_max",
        setup="Set COOL mode, 32 C, fan speed 3, both swings off.",
        button="Press TEMP UP once while already at 32 C.",
        expected="Maximum-temperature UP boundary frame in Celsius mode.",
    ),
    CaptureStep(
        label="mode_auto",
        setup="Cycle the remote until the next MODE press will select AUTO.",
        button="Press MODE once.",
        expected="AUTO mode.",
    ),
    CaptureStep(
        label="mode_dry",
        setup="Set AUTO mode.",
        button="Press MODE once.",
        expected="DRY / dehumidifier mode.",
    ),
    CaptureStep(
        label="mode_cool",
        setup="Set DRY / dehumidifier mode.",
        button="Press MODE once.",
        expected="COOL mode.",
    ),
    CaptureStep(
        label="mode_heat",
        setup="Set COOL mode.",
        button="Press MODE once.",
        expected="HEAT mode.",
    ),
    CaptureStep(
        label="mode_fan",
        setup="Set HEAT mode.",
        button="Press MODE once.",
        expected="FAN-only mode.",
    ),
    CaptureStep(
        label="fan_speed_1",
        setup="Set COOL mode, 72 F, fan NIGHT mode.",
        button="Press FAN once.",
        expected="Fan speed 1.",
    ),
    CaptureStep(
        label="fan_speed_2",
        setup="Set COOL mode, 72 F, fan speed 1.",
        button="Press FAN once.",
        expected="Fan speed 2.",
    ),
    CaptureStep(
        label="fan_speed_3",
        setup="Set COOL mode, 72 F, fan speed 2.",
        button="Press FAN once.",
        expected="Fan speed 3.",
    ),
    CaptureStep(
        label="fan_speed_4",
        setup="Set COOL mode, 72 F, fan speed 3.",
        button="Press FAN once.",
        expected="Fan speed 4.",
    ),
    CaptureStep(
        label="fan_speed_5",
        setup="Set COOL mode, 72 F, fan speed 4.",
        button="Press FAN once.",
        expected="Fan speed 5.",
    ),
    CaptureStep(
        label="fan_auto",
        setup="Set COOL mode, 72 F, fan speed 5.",
        button="Press FAN once.",
        expected="Fan auto.",
    ),
    CaptureStep(
        label="fan_night",
        setup="Set COOL mode, 72 F, fan auto.",
        button="Press FAN once.",
        expected="Fan night mode / indoor unit quiet airflow.",
    ),
    CaptureStep(
        label="swing_vertical_on",
        setup="Set COOL mode, 72 F, vertical swing off.",
        button="Press the vertical SWING button once.",
        expected="Vertical swing on.",
    ),
    CaptureStep(
        label="swing_vertical_off",
        setup="Set COOL mode, 72 F, vertical swing on.",
        button="Press the vertical SWING button once.",
        expected="Vertical swing off.",
    ),
    CaptureStep(
        label="swing_horizontal_on",
        setup="Set COOL mode, 72 F, horizontal swing off.",
        button="Press the horizontal SWING button once.",
        expected="Horizontal swing on.",
    ),
    CaptureStep(
        label="swing_horizontal_off",
        setup="Set COOL mode, 72 F, horizontal swing on.",
        button="Press the horizontal SWING button once.",
        expected="Horizontal swing off.",
    ),
    CaptureStep(
        label="quiet_on",
        setup="Set COOL mode, 72 F, QUIET off.",
        button="Press QUIET once.",
        expected="Quiet mode on.",
    ),
    CaptureStep(
        label="quiet_off",
        setup="Set COOL mode, 72 F, QUIET on.",
        button="Press QUIET once.",
        expected="Quiet mode off.",
    ),
    CaptureStep(
        label="sensor_comfort",
        setup="Set SENSOR state to off.",
        button="Press SENSOR once.",
        expected="Comfort operation active.",
    ),
    CaptureStep(
        label="sensor_eye",
        setup="Set SENSOR state to comfort operation.",
        button="Press SENSOR once.",
        expected="Intelligent eye active.",
    ),
    CaptureStep(
        label="sensor_comfort_and_eye",
        setup="Set SENSOR state to intelligent eye.",
        button="Press SENSOR once.",
        expected="Comfort operation and intelligent eye active.",
    ),
    CaptureStep(
        label="sensor_off",
        setup="Set SENSOR state to comfort operation plus intelligent eye.",
        button="Press SENSOR once.",
        expected="Sensor features off.",
    ),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/cu.usbserial-0001")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--out", default="captures", help="Output directory")
    parser.add_argument("--start-at", help="Start at a specific step label")
    parser.add_argument("--only", help="Capture only a specific step label")
    parser.add_argument("--list", action="store_true", help="List the capture plan and exit")
    parser.add_argument("--dry-run", action="store_true", help="Walk the prompts without opening serial")
    return parser.parse_args()


def selected_steps(args: argparse.Namespace) -> list[CaptureStep]:
    steps = CORE_STEPS
    if args.only:
        steps = [step for step in steps if step.label == args.only]
        if not steps:
            raise SystemExit(f"Unknown step label: {args.only}")
    elif args.start_at:
        labels = [step.label for step in steps]
        if args.start_at not in labels:
            raise SystemExit(f"Unknown start label: {args.start_at}")
        steps = steps[labels.index(args.start_at) :]
    return steps


def print_step(index: int, total: int, step: CaptureStep) -> None:
    print()
    print(f"[{index}/{total}] {step.label}")
    print(f"  Set up:   {step.setup}")
    print(f"  Capture:  {step.button}")
    print(f"  Expect:   {step.expected}")
    if step.notes:
        print(f"  Note:     {step.notes}")


def wait_for_capture(ser: object) -> tuple[int, int | None, list[list[int]]]:
    expected_count: int | None = None
    buffer_capacity: int | None = None
    rows: list[list[int]] = []

    while True:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode(errors="replace").strip()
        begin = BEGIN_RE.search(line)
        if begin:
            expected_count = int(begin.group(1))
            buffer_capacity = int(begin.group(2)) if begin.group(2) else None
            rows = []
            if buffer_capacity is None:
                print(f"  Receiving frame with {expected_count} reported symbols...")
            else:
                print(
                    f"  Receiving frame with {expected_count} reported symbols "
                    f"(buffer {buffer_capacity})..."
                )
            continue

        symbol = SYMBOL_RE.search(line)
        if symbol:
            if expected_count is None:
                continue

            row = [int(group) for group in symbol.groups()]
            symbol_index = row[0]
            if symbol_index != len(rows):
                raise CaptureFormatError(
                    "out-of-order capture symbol "
                    f"{symbol_index}; expected {len(rows)}. "
                    "This usually means serial output from two frames was interleaved."
                )
            rows.append(row)
            continue

        if "IR_CAPTURE_END" in line and expected_count is not None:
            return expected_count, buffer_capacity, rows


def run_wizard(args: argparse.Namespace) -> None:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required. Source ESP-IDF first, or install pyserial in your Python env."
        ) from exc

    steps = selected_steps(args)
    out_dir = Path(args.out)
    session_path = out_dir / "sessions.jsonl"

    out_dir.mkdir(parents=True, exist_ok=True)
    with session_path.open("a") as session_file:
        session_file.write(json.dumps({"port": args.port, "baud": args.baud, "steps": len(steps)}) + "\n")

    print(f"Opening {args.port} at {args.baud} baud.")
    print("Close idf.py monitor before running this wizard.")

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        ser.reset_input_buffer()

        for index, step in enumerate(steps, start=1):
            while True:
                print_step(index, len(steps), step)
                choice = input("Press Enter to arm capture, s to skip, q to quit: ").strip().lower()
                if choice == "q":
                    print("Stopped.")
                    return
                if choice == "s":
                    print("Skipped.")
                    break
                if choice:
                    continue

                print("  Armed. Press the remote button now.")
                try:
                    expected_count, buffer_capacity, rows = wait_for_capture(ser)
                except CaptureFormatError as exc:
                    print(f"  Discarded capture: {exc}")
                    print("  Repeat this step after the receiver is idle.")
                    continue
                metadata: dict[str, object] = {
                    "setup": step.setup,
                    "button": step.button,
                    "expected_state": step.expected,
                    "notes": step.notes,
                }
                if buffer_capacity is not None:
                    metadata["buffer_capacity"] = buffer_capacity
                    metadata["truncated"] = len(rows) >= buffer_capacity
                else:
                    metadata["legacy_format"] = True
                    if expected_count >= LEGACY_CAPTURE_LIMIT:
                        metadata["buffer_capacity"] = expected_count
                        metadata["truncated"] = len(rows) >= expected_count
                        metadata["capture_warning"] = (
                            "Legacy firmware did not report buffer capacity; "
                            "a 256-symbol frame is probably truncated."
                        )
                if len(rows) != expected_count:
                    metadata["symbol_count_mismatch"] = True
                    metadata["capture_warning"] = (
                        f"Firmware reported {expected_count} symbols, "
                        f"but {len(rows)} symbols were parsed."
                    )
                path = write_capture(
                    out_dir,
                    step.label,
                    expected_count,
                    rows,
                    metadata=metadata,
                )

                print(f"  Saved {len(rows)} symbols to {path}")
                if metadata.get("truncated"):
                    print("  Warning: capture filled the firmware buffer and may be truncated.")
                    again = input("  Repeat this step? [y/N]: ").strip().lower()
                    if again == "y":
                        continue
                break


def list_steps(steps: list[CaptureStep]) -> None:
    for index, step in enumerate(steps, start=1):
        print(f"{index:02d}. {step.label}: {step.expected}")


def main() -> None:
    args = parse_args()
    steps = selected_steps(args)

    if args.list:
        list_steps(steps)
        return

    if args.dry_run:
        for index, step in enumerate(steps, start=1):
            print_step(index, len(steps), step)
        return

    run_wizard(args)


if __name__ == "__main__":
    main()
