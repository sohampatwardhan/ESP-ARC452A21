#!/usr/bin/env python3
"""Decode Daikin ARC452A21 capture CSVs and validate known field mappings."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path


SECTION_LENGTHS = (8, 8, 19)
FULL_FRAME_SYMBOLS = 292
BIT_ONE_SPACE_US = 800


@dataclass(frozen=True)
class DecodedCapture:
    path: Path
    section_1: list[int]
    section_2: list[int]
    section_3: list[int]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", nargs="*", type=Path, default=[Path("captures")])
    parser.add_argument("--all", action="store_true", help="Include truncated and checksum-failing frames")
    return parser.parse_args()


def capture_paths(inputs: list[Path]) -> list[Path]:
    paths: list[Path] = []
    for input_path in inputs:
        if input_path.is_dir():
            paths.extend(sorted(input_path.glob("*.csv")))
        else:
            paths.append(input_path)
    return paths


def read_symbols(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def bit_from_symbol(symbol: dict[str, str]) -> int:
    return 1 if int(symbol["duration1_us"]) > BIT_ONE_SPACE_US else 0


def decode_capture(path: Path) -> DecodedCapture | None:
    symbols = read_symbols(path)
    if len(symbols) != FULL_FRAME_SYMBOLS:
        return None

    sections: list[list[int]] = []
    position = 6
    for byte_count in SECTION_LENGTHS:
        position += 1
        section: list[int] = []
        for _ in range(byte_count):
            value = 0
            for bit_index in range(8):
                value |= bit_from_symbol(symbols[position]) << bit_index
                position += 1
            section.append(value)
        sections.append(section)
        if byte_count != SECTION_LENGTHS[-1]:
            position += 1

    return DecodedCapture(path, sections[0], sections[1], sections[2])


def checksum_ok(section: list[int]) -> bool:
    return (sum(section[:-1]) & 0xFF) == section[-1]


def expected_fields(label: str) -> dict[str, int]:
    fields: dict[str, int] = {}

    if "mode_auto" in label:
        fields["mode"] = 0x08
    elif "mode_dry" in label:
        fields.update({"mode": 0x28, "temp": 0xC0, "fan": 0xA0})
    elif "mode_cool" in label:
        fields["mode"] = 0x38
    elif "mode_heat" in label:
        fields["mode"] = 0x48
    elif "mode_fan" in label:
        fields.update({"mode": 0x68, "temp": 0x32})

    if "cool_64" in label:
        fields["temp"] = 0x24
    elif "cool_65" in label:
        fields["temp"] = 0x25
    elif "cool_72" in label:
        fields["temp"] = 0x2C
    elif "cool_90" in label:
        fields["temp"] = 0x40

    if "fan_speed_1" in label:
        fields["fan"] = 0x30
    elif "fan_speed_2" in label:
        fields["fan"] = 0x40
    elif "fan_speed_3" in label:
        fields["fan"] = 0x50
    elif "fan_speed_4" in label:
        fields["fan"] = 0x60
    elif "fan_speed_5" in label:
        fields["fan"] = 0x70
    elif "fan_auto" in label:
        fields["fan"] = 0xA0
    elif "fan_night" in label:
        fields["fan"] = 0xB0

    if "swing_vertical_on" in label:
        fields["fan_low_nibble"] = 0x0F
    elif "swing_vertical_off" in label:
        fields["fan_low_nibble"] = 0x00

    if "swing_horizontal_on" in label:
        fields["hswing"] = 0x0F
    elif "swing_horizontal_off" in label or "swing_off" in label:
        fields["hswing"] = 0x00

    if "quiet_on" in label:
        fields["quiet"] = 0x20
    elif "quiet_off" in label:
        fields["quiet"] = 0x00

    if "sensor_comfort_and_eye" in label:
        fields.update({"fan": 0xA0, "sensor": 0x82})
    elif "sensor_comfort" in label:
        fields.update({"fan": 0xA0, "sensor": 0x80})
    elif "sensor_eye" in label:
        fields["sensor"] = 0x82
    elif "sensor_off" in label:
        fields["sensor"] = 0x80

    return fields


def actual_fields(section_3: list[int]) -> dict[str, int]:
    return {
        "mode": section_3[5],
        "temp": section_3[6],
        "fan": section_3[8],
        "fan_low_nibble": section_3[8] & 0x0F,
        "hswing": section_3[9],
        "quiet": section_3[13],
        "sensor": section_3[16],
    }


def validate(decoded: DecodedCapture) -> list[str]:
    failures: list[str] = []
    if not checksum_ok(decoded.section_3):
        failures.append("section 3 checksum failed")

    actual = actual_fields(decoded.section_3)
    for name, expected in expected_fields(decoded.path.stem).items():
        if actual[name] != expected:
            failures.append(f"{name} expected 0x{expected:02X}, got 0x{actual[name]:02X}")

    label = decoded.path.stem
    if "sensor_comfort" in label:
        if decoded.section_1[6] != 0x10:
            failures.append(f"section 1 comfort flag expected 0x10, got 0x{decoded.section_1[6]:02X}")
    elif "sensor_eye" in label or "sensor_off" in label:
        if decoded.section_1[6] != 0x00:
            failures.append(f"section 1 comfort flag expected 0x00, got 0x{decoded.section_1[6]:02X}")

    return failures


def format_section(section: list[int]) -> str:
    return " ".join(f"{byte:02X}" for byte in section)


def main() -> None:
    args = parse_args()
    decoded_count = 0
    failure_count = 0

    for path in capture_paths(args.captures):
        decoded = decode_capture(path)
        if decoded is None:
            if args.all:
                print(f"SKIP {path}: not exactly {FULL_FRAME_SYMBOLS} symbols")
            continue

        failures = validate(decoded)
        if failures and not args.all:
            failure_count += 1
            print(f"FAIL {path}")
            for failure in failures:
                print(f"  - {failure}")
            print(f"  section 3: {format_section(decoded.section_3)}")
            continue

        decoded_count += 1
        if args.all or failures:
            status = "FAIL" if failures else "OK"
            print(f"{status} {path}")
            print(f"  section 3: {format_section(decoded.section_3)}")
            for failure in failures:
                print(f"  - {failure}")

    if failure_count:
        raise SystemExit(1)

    print(f"Validated {decoded_count} complete capture(s).")


if __name__ == "__main__":
    main()
