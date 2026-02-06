#!/usr/bin/env python3
"""
calibrate_widowx_joint.py - Recalibrate a single WidowX joint's mapping inputs.

Flow per selected joint:
- Record NEUTRAL position for the joint
- Record FIRST and SECOND calibration positions (unordered, define range)

Only the selected joint's `neutral_positions[idx]` and `joint_limits[idx]`
are updated in `widowx_calibration.json`. Other joints are preserved.

Units: radians (floating point), as reported by Interbotix.
"""

import argparse
import json
import os
import sys
import time
from typing import List


def nonblocking_enter_pressed() -> bool:
    try:
        import select
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
    except Exception:
        return False


def show_joint_until_enter(arm, joint_index: int, joint_name: str, prompt: str) -> float:
    print(f"\n{prompt}")
    last_print = 0.0
    current_val = 0.0
    while True:
        values = list(arm.dxl.joint_states.position[:6])
        current_val = float(values[joint_index])
        now = time.time()
        if now - last_print > 0.05:
            print(
                f"\rJoint {joint_index} ({joint_name}): {current_val: .6f} rad",
                end="",
                flush=True,
            )
            last_print = now

        if nonblocking_enter_pressed():
            try:
                input()
            except Exception:
                pass
            break
        time.sleep(0.02)
    print(f"\nRecorded: {current_val:.6f} rad")
    return current_val


def update_calibration_file(
    calib_path: str, joint_index: int, neutral_val: float, pos1: float, pos2: float
) -> bool:
    # Load existing calibration (required)
    try:
        with open(calib_path, "r") as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: calibration file not found: {calib_path}")
        return False
    except json.JSONDecodeError as e:
        print(f"Error: invalid JSON in {calib_path}: {e}")
        return False

    # Validate structure
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

    # Update only the selected joint's calibration pair
    data["neutral_positions"][joint_index] = float(neutral_val)
    data["joint_limits"][joint_index] = [float(pos1), float(pos2)]
    data["calibration_date"] = time.strftime("%Y-%m-%d %H:%M:%S")

    # Write back
    try:
        with open(calib_path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"Updated joint {joint_index} calibration in {calib_path}")
        return True
    except Exception as e:
        print(f"Error writing calibration file: {e}")
        return False


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Recalibrate a single WidowX joint")
    p.add_argument(
        "--calib-file",
        default="widowx_calibration.json",
        help="Path to WidowX calibration JSON (default: widowx_calibration.json)",
    )
    return p


def main():
    args = build_arg_parser().parse_args()

    try:
        from interbotix_xs_modules.arm import InterbotixManipulatorXS
    except ImportError:
        print("Error: interbotix_xs_modules not found. Install Interbotix SDK.")
        sys.exit(1)

    print("=== WidowX Single-Joint Calibration ===")

    # Initialize arm (no torque on, manual positioning)
    arm = InterbotixManipulatorXS(
        robot_model="wx250s",
        group_name="arm",
        gripper_name="gripper",
        robot_name="master_left",
        init_node=True,
    )
    time.sleep(0.5)

    # Determine joint names
    joint_names: List[str] = list(arm.arm.group_info.joint_names)
    if len(joint_names) < 6:
        print("Error: expected 6 arm joints")
        sys.exit(1)

    # Prompt user to select joint by number or name
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

    print(f"\nRecalibrating joint {joint_index} ({joint_name})")

    # Record neutral for the selected joint
    neutral_val = show_joint_until_enter(
        arm,
        joint_index,
        joint_name,
        f"Move {joint_name} to NEUTRAL position and press Enter...",
    )
    print(f"Neutral captured: {neutral_val:.6f} rad")

    # Record two calibration positions for the selected joint
    pos1 = show_joint_until_enter(
        arm,
        joint_index,
        joint_name,
        f"Move {joint_name} to FIRST calibration position and press Enter...",
    )
    pos2 = show_joint_until_enter(
        arm,
        joint_index,
        joint_name,
        f"Move {joint_name} to SECOND calibration position and press Enter...",
    )

    print(f"\nCaptured positions: [{pos1:.6f}, {pos2:.6f}] rad")

    # Confirm and write
    confirm = input("Save to calibration file? (y/N): ").strip().lower()
    if confirm != "y":
        print("Aborted. No changes saved.")
        sys.exit(0)

    if not update_calibration_file(args.calib_file, joint_index, neutral_val, pos1, pos2):
        sys.exit(1)

    print("Done.")


if __name__ == "__main__":
    main()


