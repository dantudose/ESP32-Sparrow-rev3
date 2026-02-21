#!/usr/bin/env python3
import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


def parse_dts_storage_offset(dts_path):
    storage_offset = None
    in_storage = False

    with dts_path.open() as handle:
        for line in handle:
            if "storage_partition:" in line:
                in_storage = True

            if in_storage and "reg = <" in line:
                nums = re.findall(r"0x[0-9a-fA-F]+|\\d+", line)
                if len(nums) >= 1:
                    storage_offset = int(nums[0], 0)
                    break

            if in_storage and line.strip().startswith("};"):
                in_storage = False

    return storage_offset


def parse_esptool_config(config_path):
    flash_mode = "dio"
    flash_freq = "40m"
    flash_size = "detect"

    with config_path.open() as handle:
        for line in handle:
            if line.startswith("CONFIG_ESPTOOLPY_FLASHMODE="):
                flash_mode = line.split("=", 1)[1].strip().strip('"')
            elif line.startswith("CONFIG_ESPTOOLPY_FLASHFREQ="):
                flash_freq = line.split("=", 1)[1].strip().strip('"')
            elif line.startswith("CONFIG_ESPTOOLPY_FLASHSIZE="):
                flash_size = line.split("=", 1)[1].strip().strip('"')

    return flash_mode, flash_freq, flash_size


def parse_runner_args(runners_path):
    if not runners_path.exists():
        return None, None, None

    text = runners_path.read_text()
    mode = None
    freq = None
    size = None

    match = re.search(r"--esp-flash-mode=([^\s]+)", text)
    if match:
        mode = match.group(1)

    match = re.search(r"--esp-flash-freq=([^\s]+)", text)
    if match:
        freq = match.group(1)

    match = re.search(r"--esp-flash-size=([^\s]+)", text)
    if match:
        size = match.group(1)

    return mode, freq, size


def find_esptool():
    override = os.environ.get("ESPTOOL")
    if override:
        return override
    return shutil.which("esptool")


def flash_image(build_dir, image_path, port, baud):
    dts_path = build_dir / "zephyr" / "zephyr.dts"
    config_path = build_dir / "zephyr" / ".config"
    runners_path = build_dir / "zephyr" / "runners.yaml"

    if not dts_path.exists():
        raise FileNotFoundError(f"Missing {dts_path}")
    if not config_path.exists():
        raise FileNotFoundError(f"Missing {config_path}")
    if not image_path.exists():
        raise FileNotFoundError(f"Missing {image_path}")

    storage_offset = parse_dts_storage_offset(dts_path)
    if storage_offset is None:
        raise RuntimeError("Could not find storage partition offset in zephyr.dts")

    flash_mode, flash_freq, flash_size = parse_esptool_config(config_path)
    runner_mode, runner_freq, runner_size = parse_runner_args(runners_path)
    if runner_mode:
        flash_mode = runner_mode
    if runner_freq:
        flash_freq = runner_freq
    if runner_size:
        flash_size = runner_size

    esptool = find_esptool()
    if not esptool:
        raise RuntimeError("esptool not found in PATH (or ESPTOOL override)")

    cmd = [
        esptool,
        "--port",
        port,
        "--baud",
        str(baud),
        "--before",
        "default-reset",
        "--after",
        "no-reset",
        "write-flash",
        "--flash-mode",
        flash_mode,
        "--flash-freq",
        flash_freq,
        "--flash-size",
        flash_size,
        "-u",
        hex(storage_offset),
        str(image_path),
    ]
    subprocess.run(cmd, check=True)

    print(f"Flashed {image_path} to 0x{storage_offset:x}")


def main():
    parser = argparse.ArgumentParser(description="Flash a LittleFS image to storage partition.")
    parser.add_argument("--build-dir", default="build", help="Zephyr build directory")
    parser.add_argument("--image", default=None, help="LittleFS image path")
    parser.add_argument("--port", default=os.environ.get("ESPTOOL_PORT"), help="Serial port")
    parser.add_argument("--baud", default=921600, type=int, help="Serial baud rate")
    args = parser.parse_args()

    if not args.port:
        print("Error: --port or ESPTOOL_PORT is required", file=sys.stderr)
        return 2

    build_dir = Path(args.build_dir).resolve()
    if args.image:
        image_path = Path(args.image).resolve()
    else:
        image_path = build_dir / "littlefs.img"

    try:
        flash_image(build_dir, image_path, args.port, args.baud)
    except (FileNotFoundError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
