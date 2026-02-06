#!/usr/bin/env python3
"""
calibrate_piperx_joint.py - Recalibrate a single PiperX joint's mapping inputs.

Flow per selected joint:
- Record NEUTRAL position for the joint
- Record FIRST and SECOND calibration positions (unordered, define range)

Only the selected joint's `neutral_positions[idx]` and `joint_limits[idx]`
are updated in `piperx_calibration.json`. Other joints are preserved.

Units: PiperX degrees*1000 (integers) as provided by the SDK.
"""

import argparse
import json
import os
import sys
import time
from typing import List


def try_import_piper_sdk(force_version: str = "auto"):
    if force_version in ("auto", "v2"):
        try:
            from piper_sdk import C_PiperInterface_V2  # type: ignore
            return C_PiperInterface_V2, "v2"
        except Exception:
            if force_version == "v2":
                raise
    if force_version in ("auto", "v1"):
        try:
            from piper_sdk import C_PiperInterface_V1  # type: ignore
            return C_PiperInterface_V1, "v1"
        except Exception:
            if force_version == "v1":
                raise
    raise ImportError("piper_sdk not available. Install with: pip install piper-sdk")


def wait_for_ready(piper, sdk_version: str):
    if sdk_version == "v2":
        while True:
            try:
                if piper.EnablePiper():
                    break
            except Exception:
                pass
            time.sleep(0.01)
    else:
        while True:
            try:
                if not piper.DisablePiper():
                    break
            except Exception:
                pass
            time.sleep(0.01)


def read_joint_array(piper) -> List[int]:
    msgs = piper.GetArmJointMsgs()
    js = msgs.joint_state
    return [
        int(js.joint_1),
        int(js.joint_2),
        int(js.joint_3),
        int(js.joint_4),
        int(js.joint_5),
        int(js.joint_6),
    ]


def nonblocking_enter_pressed() -> bool:
    try:
        import select
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
    except Exception:
        return False


def show_joint_until_enter(piper, joint_index: int, joint_name: str, prompt: str) -> int:
    print(f"\n{prompt}")
    last_print = 0.0
    val = 0
    while True:
        vals = read_joint_array(piper)
        val = int(vals[joint_index])
        now = time.time()
        if now - last_print > 0.05:
            print(f"\rJoint {joint_index+1} ({joint_name}): {val:>7d}", end="", flush=True)
            last_print = now
        if nonblocking_enter_pressed():
            try:
                input()
            except Exception:
                pass
            break
        time.sleep(0.02)
    print(f"\nRecorded: {val}")
    return val


def update_calibration_file(calib_path: str, joint_index: int, neutral_val: int, pos1: int, pos2: int) -> bool:
    try:
        with open(calib_path, "r") as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: calibration file not found: {calib_path}")
        return False
    except json.JSONDecodeError as e:
        print(f"Error: invalid JSON in {calib_path}: {e}")
        return False

    if "joint_limits" not in data or not isinstance(data["joint_limits"], list):
        print("Error: calibration file missing 'joint_limits' list")
        return False
    if joint_index < 0 or joint_index >= len(data["joint_limits"]):
        print("Error: joint index out of range in calibration file")
        return False
    if "neutral_positions" not in data or not isinstance(data["neutral_positions"], list):
        print("Error: calibration file missing 'neutral_positions' list")
        return False
    if joint_index >= len(data["neutral_positions"]):
        print("Error: joint index out of range in neutral_positions")
        return False

    data["neutral_positions"][joint_index] = int(neutral_val)
    data["joint_limits"][joint_index] = [int(pos1), int(pos2)]
    data["calibration_date"] = time.strftime("%Y-%m-%d %H:%M:%S")

    try:
        with open(calib_path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"Updated joint {joint_index} calibration in {calib_path}")
        return True
    except Exception as e:
        print(f"Error writing calibration file: {e}")
        return False


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Recalibrate a single PiperX joint")
    p.add_argument("--can", dest="can_name", default="can0", help="CAN interface (default: can0)")
    p.add_argument(
        "--sdk-version", choices=["auto", "v1", "v2"], default="auto", help="Force SDK version"
    )
    p.add_argument(
        "--calib-file",
        default="piperx_calibration.json",
        help="Path to PiperX calibration JSON (default: piperx_calibration.json)",
    )
    return p


def main():
    args = build_arg_parser().parse_args()

    PiperClass, sdk_version = try_import_piper_sdk(args.sdk_version)

    print("=== PiperX Single-Joint Calibration ===")
    print(f"SDK version: {sdk_version}")
    print(f"CAN interface: {args.can_name}")

    # Instantiate interface
    if sdk_version == "v1":
        piper = PiperClass(
            can_name=args.can_name,
            judge_flag=False,
            can_auto_init=True,
            dh_is_offset=1,
            start_sdk_joint_limit=False,
            start_sdk_gripper_limit=False,
        )
    else:
        piper = PiperClass(args.can_name)

    print("Connecting to PiperX...")
    piper.ConnectPort()
    print("Disabling PiperX arm for manual calibration...")
    try:
        while piper.DisablePiper():
            time.sleep(0.01)
        print("Arm disabled successfully")
    except Exception as e:
        print(f"Warning: could not confirm disable state: {e}")

    # Joint list
    joint_names = [f"joint_{i}" for i in range(1, 7)]

    print("\nSelect a joint to recalibrate:")
    for i, name in enumerate(joint_names):
        print(f"  {i}: {name}")
    sel = input("Enter joint index (0-5) or name: ").strip()

    try:
        if sel.isdigit():
            joint_index = int(sel)
            joint_name = joint_names[joint_index]
        else:
            joint_index = joint_names.index(sel)
            joint_name = sel
    except Exception:
        print("Invalid selection")
        sys.exit(1)

    print(f"\nRecalibrating joint {joint_index+1} ({joint_name})")

    # Record neutral for the selected joint
    neutral_val = show_joint_until_enter(
        piper,
        joint_index,
        joint_name,
        f"Move {joint_name} to NEUTRAL position and press Enter...",
    )
    print(f"Neutral captured: {neutral_val} (deg*1000)")

    pos1 = show_joint_until_enter(
        piper,
        joint_index,
        joint_name,
        f"Move {joint_name} to FIRST calibration position and press Enter...",
    )
    pos2 = show_joint_until_enter(
        piper,
        joint_index,
        joint_name,
        f"Move {joint_name} to SECOND calibration position and press Enter...",
    )

    print(f"\nCaptured positions: [{pos1}, {pos2}] (deg*1000)")

    confirm = input("Save to calibration file? (y/N): ").strip().lower()
    if confirm != "y":
        print("Aborted. No changes saved.")
        sys.exit(0)

    if not update_calibration_file(args.calib_file, joint_index, neutral_val, pos1, pos2):
        sys.exit(1)

    print("Done.")

    try:
        piper.DisableArm(motor_num=7, enable_flag=0x01)
    except Exception:
        pass
    try:
        piper.DisconnectPort()
    except Exception:
        pass


if __name__ == "__main__":
    main()


