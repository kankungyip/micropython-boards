#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import argparse
import importlib
import os
import re
import shutil
import sys

import yaml
from gen_lang import gen_lang

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
mpy_dir = os.path.join(root, "lib", "micropython")
dist_dir = os.path.join(root, "dist")

# add arguments
#
parser = argparse.ArgumentParser(description="MicroPython Board builder.")
parser.add_argument("board", help="board name")
parser.add_argument("-c", "--clean", action="store_true", help="clean built")
parser.add_argument("-p", "--port", help="device port")
parser.add_argument("-P", action="store_true", help="use default device port")
parser.add_argument("-e", "--erase", action="store_true", help="erase device flash")
args = parser.parse_args()

# show help
if not args.board:
    parser.print_help()
    exit(0)


# clean
#
def clean(board_info):
    print("\n[INFO] Cleaning...\n")

    board = board_info["id"]
    port = board_info["port"]

    os.chdir(os.path.join(mpy_dir, "ports", port))
    os.system(f"make clean BOARD={board}")


# install idf components from cmoudles.cmake
#
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


def install_idf_comps(components):
    os.chdir(os.path.join(mpy_dir, "ports", "esp32"))
    for comp_id in components:
        os.system(f'idf.py add-dependency "{comp_id}"')


def get_idf_comps(board, file):
    components = []
    pattern = re.compile(
        r"\$\{((?:C_MODULES_DIR|MICROPY_BOARD_DIR))\}(?:/cmodules)?/([^/]*)/?micropython\.cmake\)$"
    )
    with open(file) as f:
        lines = f.readlines()
        for line in lines:
            match = pattern.search(line)
            if match:
                mod_name = match.group(2)
                if match.group(1) == "C_MODULES_DIR":
                    yml_path = os.path.join(
                        root, "cmodules", mod_name, "idf_component.yml"
                    )
                elif mod_name:
                    yml_path = os.path.join(
                        root, "boards", board, "cmodules", mod_name, "idf_component.yml"
                    )
                else:
                    yml_path = os.path.join(
                        root, "boards", board, "cmodules", "idf_component.yml"
                    )
                idf_comps = load_yaml(yml_path)
                if idf_comps:
                    for comp_name, comp_ver in idf_comps["dependencies"].items():
                        comp_id = f"{comp_name}{comp_ver}"
                        if comp_id not in components:
                            components.append(comp_id)
    return components


# build
#
def walk_dir(dir_path, excludes_subdir=False):
    file_list = []
    for root, dirs, files in os.walk(dir_path):
        dirs[:] = [d for d in dirs if not d.startswith(".")]
        files[:] = [f for f in files if not f.startswith(".")]
        if excludes_subdir and dir_path != root:
            break
        for file in files:
            file_list.append(os.path.join(root, file))
    return file_list


def read_partitions_from(files):
    pattern = r"^partitions-[a-zA-Z0-9_-]+\.csv$"
    for filename in files:
        if re.match(pattern, filename.split("/")[-1]) is not None:
            with open(filename, "r") as f:
                rows = []
                for line in f:
                    rows.append(line.strip().split(","))
                return rows


def build(board_info):
    # git restore

    os.chdir(mpy_dir)
    os.system("git restore .")
    os.system("git clean -df")
    os.system("git checkout v1.29.0")

    print("\n[INFO] Building...\n")

    board = board_info["id"]
    port = board_info["port"]
    version = board_info["version"]

    board_dir = os.path.join(root, "boards", board)
    mpy_board_dir = os.path.join(mpy_dir, "ports", port, "boards", board)

    # import board build script
    if is_exists(os.path.join(board_dir, "build.py")):
        sys.path.insert(0, root)
        importlib.import_module(f"boards.{board}.build")

    # coping board files
    board_files = []
    board_files.extend(walk_dir(board_dir))

    for file in board_files:
        destfile = file.replace(root, mpy_dir).replace(
            os.path.join("boards", board), os.path.join("ports", port, "boards", board)
        )
        dir = os.path.dirname(destfile)
        if not is_exists(dir):
            os.makedirs(dir)
        shutil.copy(file, destfile)

    # generate l10n file
    l10n_dir = os.path.join(mpy_board_dir, "l10n")
    l10n_file = os.path.join(mpy_board_dir, "modules", "l10n.py")
    gen_lang(l10n_dir, l10n_file)

    # esp32 install idf components
    cmodules_file = os.path.join(mpy_board_dir, "cmodules.cmake")
    if port == "esp32" and is_exists(cmodules_file):
        print("\n[INFO] Installing IDF components...\n")
        idf_components = get_idf_comps(board, cmodules_file)
        install_idf_comps(idf_components)

    # write MICROPY_BANNER_NAME_AND_VERSION and MICROPY_BANNER_MACHINE
    with open(os.path.join(mpy_board_dir, "mpconfigboard.h"), "a") as f:
        f.write(f"""
#undef MICROPY_VERSION_STRING
#define MICROPY_VERSION_STRING "{version}"

#define MICROPY_BANNER_NAME_AND_VERSION MICROPY_HW_BOARD_NAME " v{version}; MicroPython v" MICROPY_VERSION_STRING_BASE

#ifndef MICROPY_BANNER_MACHINE
#define MICROPY_BANNER_MACHINE "Provided by \x1b[1mPopsicle Team\x1b[0m"
#endif
""")

    # build micropython
    os.chdir(os.path.join(mpy_dir, "ports", port))
    os.system(f"make submodules BOARD={board}")
    os.system(f"make BOARD={board}")

    firmware_path = os.path.join(
        mpy_dir, "ports", port, "build-" + board, "firmware.bin"
    )
    if not is_exists(firmware_path):
        return

    # combine resources to firmware
    resources_dir = os.path.join(board_dir, "resources")
    # out firmware path
    out_firmware = os.path.join(
        dist_dir, f"{board}_{board_info['version']}.bin".lower()
    )

    if not is_exists(dist_dir):
        os.makedirs(dist_dir)

    if is_exists(out_firmware):
        os.remove(out_firmware)

    cmd_str = f"cp {firmware_path} {out_firmware}"
    if is_exists(resources_dir) and port == "esp32":
        partitions = read_partitions_from(board_files)
        if partitions:
            print("\n[INFO] Combining resources...\n")

            for partition in partitions:
                if partition[0] == "resource":
                    combine_path = os.path.join(root, "tools", "combine", "combine.py")
                    cmd_str = f"python3 {combine_path}"
                    cmd_str += f" --dir {resources_dir}"
                    cmd_str += f" --address {partition[3].strip()}"
                    cmd_str += f" --size {partition[4].strip()}"
                    if "flash offset" in board_info:
                        cmd_str += f" --offset {hex(board_info['flash offset'])}"
                    cmd_str += f" {firmware_path} {out_firmware}"
                    break

    os.chdir(root)
    os.system(cmd_str)

    if is_exists(out_firmware):
        print(f"\n[INFO] Generated {out_firmware}")
        return out_firmware


# esp32 flash firmware
#
def esp32_flash(board_info, firmware_path):
    if args.erase:
        print("\n[INFO] Cleaning flash...\n")
        if args.P:
            os.system("esptool.py --chip auto erase_flash")
        else:
            os.system(f"esptool.py --chip auto --port {args.port} erase_flash")

    if not is_exists(firmware_path):
        print(f'[ERROR] "{firmware_path}" does not exist.\n')
        exit(1)

    print("\n[INFO] Uploading firmware...\n")

    flash_offset = (
        hex(board_info["flash offset"]) if "flash offset" in board_info else 0
    )
    if args.P:
        os.system(
            f"esptool.py --chip auto write_flash -z {flash_offset} {firmware_path}"
        )
    else:
        os.system(
            f"esptool.py --chip auto --port {args.port} write_flash -z {flash_offset} {firmware_path}"
        )


if __name__ == "__main__":
    board = args.board.upper()
    board_info = load_yaml(os.path.join(root, "boards", board, "boardinfo.yml"))

    if not board_info:
        print(f"[ERROR] Board {board} not found.\n")
        parser.print_help()
        exit(1)

    if board_info:
        board_info["id"] = board

    if board_info and args.clean:
        clean(board_info)

    firmware_path = None
    if board_info:
        firmware_path = build(board_info)

    if firmware_path and board_info["port"] == "esp32" and (args.port or args.P):
        esp32_flash(board_info, firmware_path)
