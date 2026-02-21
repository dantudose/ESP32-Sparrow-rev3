#!/usr/bin/env python3
import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


def parse_dts(dts_path):
    storage_offset = None
    storage_size = None
    erase_block = None
    in_storage = False

    with dts_path.open() as handle:
        for line in handle:
            if "storage_partition:" in line:
                in_storage = True

            if in_storage and "reg = <" in line:
                nums = re.findall(r"0x[0-9a-fA-F]+|\\d+", line)
                if len(nums) >= 2:
                    storage_offset = int(nums[0], 0)
                    storage_size = int(nums[1], 0)

            if in_storage and line.strip().startswith("};"):
                in_storage = False

            if erase_block is None and "erase-block-size" in line:
                nums = re.findall(r"0x[0-9a-fA-F]+|\\d+", line)
                if nums:
                    erase_block = int(nums[0], 0)

    return storage_offset, storage_size, erase_block


def parse_prog_size(config_path):
    prog_size = None
    with config_path.open() as handle:
        for line in handle:
            if line.startswith("CONFIG_FS_LITTLEFS_PROG_SIZE="):
                prog_size = int(line.split("=", 1)[1], 0)
                break
    return prog_size


def find_mklittlefs():
    override = os.environ.get("MKLITTLEFS")
    if override:
        return override
    return shutil.which("mklittlefs")


def build_image(build_dir, web_dir, output_path):
    dts_path = build_dir / "zephyr" / "zephyr.dts"
    config_path = build_dir / "zephyr" / ".config"

    if not dts_path.exists():
        raise FileNotFoundError(f"Missing {dts_path}")
    if not config_path.exists():
        raise FileNotFoundError(f"Missing {config_path}")

    storage_offset, storage_size, erase_block = parse_dts(dts_path)
    prog_size = parse_prog_size(config_path)

    if storage_offset is None or storage_size is None:
        raise RuntimeError("Could not find storage partition in zephyr.dts")
    if erase_block is None:
        raise RuntimeError("Could not find erase-block-size in zephyr.dts")
    if prog_size is None:
        raise RuntimeError("Could not find CONFIG_FS_LITTLEFS_PROG_SIZE in .config")

    if not web_dir.exists():
        raise FileNotFoundError(f"Missing web assets directory: {web_dir}")

    staging_dir = build_dir / "littlefs_root"
    lfs_root_dir = web_dir.parent / "lfs"
    www_dst = staging_dir / "www"

    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True)

    if lfs_root_dir.exists():
        for path in lfs_root_dir.iterdir():
            dest = staging_dir / path.name
            if path.is_dir():
                shutil.copytree(path, dest, dirs_exist_ok=True)
            else:
                shutil.copy2(path, dest)

    www_dst.mkdir(parents=True, exist_ok=True)

    shutil.copytree(web_dir, www_dst, dirs_exist_ok=True)

    output_path.parent.mkdir(parents=True, exist_ok=True)

    mklittlefs = find_mklittlefs()
    if not mklittlefs:
        raise RuntimeError("mklittlefs not found in PATH (or MKLITTLEFS override)")

    cmd = [
        mklittlefs,
        "-c",
        str(staging_dir),
        "-b",
        str(erase_block),
        "-p",
        str(prog_size),
        "-s",
        str(storage_size),
        str(output_path),
    ]
    subprocess.run(cmd, check=True)

    print(f"Wrote {output_path} ({storage_size} bytes) at 0x{storage_offset:x}")


def main():
    parser = argparse.ArgumentParser(description="Build a LittleFS image from web assets.")
    parser.add_argument("--build-dir", default="build", help="Zephyr build directory")
    parser.add_argument("--web-dir", default="web", help="Web assets directory")
    parser.add_argument("--output", default=None, help="Output image path")
    args = parser.parse_args()

    build_dir = Path(args.build_dir).resolve()
    web_dir = Path(args.web_dir).resolve()
    if args.output:
        output_path = Path(args.output).resolve()
    else:
        output_path = build_dir / "littlefs.img"

    try:
        build_image(build_dir, web_dir, output_path)
    except (FileNotFoundError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
