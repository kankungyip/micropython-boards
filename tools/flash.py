#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import argparse
import os

import yaml

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
dist_dir = os.path.join(root, "dist")

# add arguments
#
parser = argparse.ArgumentParser(description="MicroPython Board flasher.")
parser.add_argument("board", help="board name")
parser.add_argument("-p", "--port", help="device port")
parser.add_argument(
    "-P", action="store_true", default=True, help="use default device port"
)
parser.add_argument("-e", "--erase", action="store_true", help="erase device flash")
parser.add_argument("-f", "--firmware", help="firmware path")
args = parser.parse_args()

# show help
if not args.board:
    parser.print_help()
    exit(0)


def is_exists(path):
    return os.path.exists(path)


def load_yaml(file):
    data = None
    try:
        if is_exists(file):
            with open(file) as yaml_file:
                data = yaml.safe_load(yaml_file)
    except:
        pass
    return data


# esp32 flash firmware
#
def esp32_flash(board_info, firmware_path):
    if args.erase:
        print("[INFO] Cleaning flash...\n")
        if args.P:
            os.system("esptool.py --chip auto erase_flash")
        else:
            os.system(f"esptool.py --chip auto --port {args.port} erase_flash")

    if not firmware_path:
        firmware_path = os.path.join(
            dist_dir, f"{args.board}_{board_info['version']}.bin"
        )

    if not is_exists(firmware_path):
        print(f"[INFO] {firmware_path} does not exist.\n")
        exit(1)

    print(f"[INFO] Uploading firmware...\n")

    flash_offset = (
        hex(board_info["flash offset"]) if "flash offset" in board_info else 0
    )
    if args.port:
        os.system(
            f"esptool.py --chip auto --port {args.port} write_flash -z {flash_offset} {firmware_path}"
        )
    else:
        os.system(
            f"esptool.py --chip auto write_flash -z {flash_offset} {firmware_path}"
        )


if __name__ == "__main__":
    board = args.board.upper()
    board_info = load_yaml(os.path.join(root, "boards", board, "boardinfo.yml"))

    if not board_info:
        print(f"[ERROR] Board {board} not found.\n")
        parser.print_help()
        exit(1)

    if board_info["port"] == "esp32":
        esp32_flash(board_info, args.firmware)
