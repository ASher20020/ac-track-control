from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path


DEFAULT_CONTROLS = (
    Path.home()
    / "Documents"
    / "Assetto Corsa"
    / "cfg"
    / "controls.ini"
)
X360_PRESET = (
    Path.home()
    / "Documents"
    / "Assetto Corsa"
    / "cfg"
    / "controllers"
    / "presets"
    / "X360Default.ini"
)
BACKUP_SUFFIX = ".actc-backup"


def set_ini_value(text: str, section: str, key: str, value: str) -> str:
    lines = text.splitlines(keepends=True)
    section_header = f"[{section}]"
    in_section = False
    found_section = False
    found_key = False
    output: list[str] = []
    key_pattern = re.compile(rf"^(\s*{re.escape(key)}\s*=).*?(\r?\n)?$")

    for line in lines:
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            if in_section and not found_key:
                output.append(f"{key}={value}\n")
                found_key = True
            in_section = stripped.casefold() == section_header.casefold()
            found_section = found_section or in_section
            output.append(line)
            continue

        if in_section:
            match = key_pattern.match(line)
            if match:
                newline = match.group(2) or "\n"
                output.append(f"{match.group(1)}{value}{newline}")
                found_key = True
                continue
        output.append(line)

    if in_section and not found_key:
        output.append(f"{key}={value}\n")
        found_key = True

    if not found_section:
        if output and not output[-1].endswith("\n"):
            output.append("\n")
        output.append(f"\n[{section}]\n{key}={value}\n")

    return "".join(output)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Enable arrow-key controls alongside a wheel in Assetto Corsa. "
            "Close Content Manager and Assetto Corsa before applying."
        )
    )
    parser.add_argument("--controls", type=Path, default=DEFAULT_CONTROLS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--restore", action="store_true")
    parser.add_argument(
        "--keyboard-only",
        action="store_true",
        help="Also set INPUT_METHOD=KEYBOARD instead of combining with the wheel.",
    )
    parser.add_argument(
        "--x360",
        action="store_true",
        help="Replace the active profile with Assetto Corsa's X360Default preset.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.keyboard_only and args.x360:
        print("Error: --keyboard-only and --x360 cannot be used together.", file=sys.stderr)
        return 2
    controls_path: Path = args.controls
    backup_path = controls_path.with_name(controls_path.name + BACKUP_SUFFIX)

    if not controls_path.exists():
        print(f"Error: controls file not found: {controls_path}", file=sys.stderr)
        return 2

    if args.restore:
        if not backup_path.exists():
            print(f"Error: backup not found: {backup_path}", file=sys.stderr)
            return 3
        if args.dry_run:
            print(f"Would restore {backup_path} -> {controls_path}")
            return 0
        shutil.copy2(backup_path, controls_path)
        print(f"Restored {controls_path}")
        return 0

    if args.x360:
        if not X360_PRESET.exists():
            print(f"Error: X360 preset not found: {X360_PRESET}", file=sys.stderr)
            return 4
        updated = X360_PRESET.read_text(encoding="utf-8-sig")
        for key, value in {
            "STEER_GAMMA": "1.0",
            "STEER_FILTER": "0.0",
            "SPEED_SENSITIVITY": "0.0",
            "STEER_DEADZONE": "0.0",
            "STEER_SPEED": "1.0",
        }.items():
            updated = set_ini_value(updated, "X360", key, value)
    else:
        original = controls_path.read_text(encoding="utf-8-sig")
        updated = original
        values = {
            ("ADVANCED", "COMBINE_WITH_KEYBOARD_CONTROL"): "1",
            ("KEYBOARD", "GAS"): "0x26",
            ("KEYBOARD", "BRAKE"): "0x28",
            ("KEYBOARD", "RIGHT"): "0x27",
            ("KEYBOARD", "LEFT"): "0x25",
            ("KEYBOARD", "MOUSE_STEER"): "0",
            ("KEYBOARD", "MOUSE_ACCELERATOR_BRAKE"): "0",
        }
        if args.keyboard_only:
            values[("HEADER", "INPUT_METHOD")] = "KEYBOARD"
        for (section, key), value in values.items():
            updated = set_ini_value(updated, section, key, value)

    if args.dry_run:
        print(f"Would update {controls_path}")
        print(f"Would create backup {backup_path}")
        return 0

    if not backup_path.exists():
        shutil.copy2(controls_path, backup_path)
    controls_path.write_text(updated, encoding="utf-8")
    print(f"Updated {controls_path}")
    print(f"Original saved as {backup_path}")
    print("Open Content Manager again and select the car, track, and automatic gearbox.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
