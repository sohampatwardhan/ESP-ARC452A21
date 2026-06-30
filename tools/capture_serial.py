#!/usr/bin/env python3
"""Save ESP-ARC452A21 IR capture blocks from the serial monitor.

Run this with the ESP-IDF Python environment or any Python that has pyserial:

    python tools/capture_serial.py --port /dev/cu.usbserial-0001 --label cool_22_auto
    python tools/capture_serial.py --input-file monitor.log --label cool_22_auto
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from datetime import datetime
from pathlib import Path

BEGIN_RE = re.compile(r"IR_CAPTURE_BEGIN,(\d+)(?:,(\d+))?")
SYMBOL_RE = re.compile(r"IR_CAPTURE_SYMBOL,(\d+),(\d+),(\d+),(\d+),(\d+)")
LEGACY_CAPTURE_LIMIT = 256


class CaptureFormatError(RuntimeError):
    """Raised when a serial capture block is internally inconsistent."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Serial port, e.g. /dev/cu.usbserial-0001")
    source.add_argument("--input-file", help="Existing monitor log to parse")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--label", required=True, help="Capture label, e.g. cool_22_auto")
    parser.add_argument("--out", default="captures", help="Output directory")
    return parser.parse_args()


def write_capture(
    out_dir: Path,
    label: str,
    expected_count: int,
    rows: list[list[int]],
    metadata: dict[str, object] | None = None,
) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    safe_label = re.sub(r"[^A-Za-z0-9_.-]+", "_", label).strip("_")
    csv_path = out_dir / f"{timestamp}_{safe_label}.csv"

    with csv_path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["index", "level0", "duration0_us", "level1", "duration1_us"])
        writer.writerows(rows)

    manifest_path = out_dir / "manifest.jsonl"
    with manifest_path.open("a") as file:
        entry: dict[str, object] = {
            "timestamp": timestamp,
            "label": label,
            "file": str(csv_path),
            "reported_symbols": expected_count,
            "expected_symbols": expected_count,
            "captured_symbols": len(rows),
        }
        if metadata:
            entry.update(metadata)
        json.dump(entry, file)
        file.write("\n")

    return csv_path


class CaptureParser:
    def __init__(self, out_dir: Path, label: str) -> None:
        self.out_dir = out_dir
        self.label = label
        self.current: list[list[int]] = []
        self.expected_count: int | None = None
        self.buffer_capacity: int | None = None

    def reset(self) -> None:
        self.current = []
        self.expected_count = None
        self.buffer_capacity = None

    def process_line(self, line: str) -> Path | None:
        begin = BEGIN_RE.search(line)
        if begin:
            self.expected_count = int(begin.group(1))
            self.buffer_capacity = int(begin.group(2)) if begin.group(2) else None
            self.current = []
            return None

        symbol = SYMBOL_RE.search(line)
        if symbol:
            if self.expected_count is None:
                return None

            row = [int(group) for group in symbol.groups()]
            symbol_index = row[0]
            if symbol_index != len(self.current):
                expected_index = len(self.current)
                self.reset()
                raise CaptureFormatError(
                    "out-of-order capture symbol "
                    f"{symbol_index}; expected {expected_index}. "
                    "Discarded this capture block."
                )
            self.current.append(row)
            return None

        if "IR_CAPTURE_END" in line and self.expected_count is not None:
            metadata: dict[str, object] = {}
            if self.buffer_capacity is not None:
                metadata["buffer_capacity"] = self.buffer_capacity
                metadata["truncated"] = len(self.current) >= self.buffer_capacity
            else:
                metadata["legacy_format"] = True
                if self.expected_count >= LEGACY_CAPTURE_LIMIT:
                    metadata["buffer_capacity"] = self.expected_count
                    metadata["truncated"] = len(self.current) >= self.expected_count
                    metadata["capture_warning"] = (
                        "Legacy firmware did not report buffer capacity; "
                        "a 256-symbol frame is probably truncated."
                    )
            if len(self.current) != self.expected_count:
                metadata["symbol_count_mismatch"] = True
                metadata["capture_warning"] = (
                    f"Firmware reported {self.expected_count} symbols, "
                    f"but {len(self.current)} symbols were parsed."
                )
            path = write_capture(
                self.out_dir,
                self.label,
                self.expected_count,
                self.current,
                metadata=metadata,
            )
            self.reset()
            return path

        return None


def parse_input_file(args: argparse.Namespace) -> None:
    parser = CaptureParser(Path(args.out), args.label)

    with Path(args.input_file).open() as file:
        for line in file:
            try:
                path = parser.process_line(line.strip())
            except CaptureFormatError as exc:
                print(f"Discarded corrupt capture block: {exc}")
                continue
            if path is not None:
                print(f"Saved capture to {path}")


def parse_serial(args: argparse.Namespace) -> None:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required for serial capture. Source ESP-IDF first, "
            "or install pyserial in your Python env."
        ) from exc

    parser = CaptureParser(Path(args.out), args.label)

    print(f"Listening on {args.port} at {args.baud} baud. Press Ctrl-C to stop.")
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode(errors="replace").strip()
            print(line)

            try:
                path = parser.process_line(line)
            except CaptureFormatError as exc:
                print(f"Discarded corrupt capture block: {exc}")
                continue
            if path is not None:
                print(f"Saved {path}")


def main() -> None:
    args = parse_args()
    if args.input_file:
        parse_input_file(args)
    else:
        parse_serial(args)


if __name__ == "__main__":
    main()
