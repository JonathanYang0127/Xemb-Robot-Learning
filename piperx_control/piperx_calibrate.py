#!/usr/bin/env python3
"""
piperx_calibrate.py - Interactive calibration script for PiperX

This script connects to the PiperX via the Piper SDK, lets you record:
- Neutral joint positions
- Two calibration positions per joint (used as mapping reference, not min/max)
- Gripper closed and open positions

It writes the results to a JSON file matching the format of `piperx_calibration.json`:
{
  "robot_model": "PiperX",
  "calibration_date": "YYYY-MM-DD HH:MM:SS",
  "joint_names": ["joint_1", ..., "joint_6"],
  "gripper_name": "gripper",
  "neutral_positions": [.. six ints in degrees*1000 ..],
  "joint_limits": [[pos1, pos2], ... for six joints],
  "gripper_limits": [closed, open]
}

Notes:
- PiperX units are degrees*1000 (integers). This script records values as-is
  from the SDK without conversion.
- The two joint positions are corresponding calibration poses (pos1, pos2),
  not necessarily ordered min/max.
"""

import argparse
import json
import os
import sys
import time
from typing import List, Tuple


def try_import_piper_sdk(force_version: str = "auto"):
    """Import Piper SDK interface class.

    Returns (PiperClass, sdk_version_str)
    """
    # Optional path hint: if users keep piper_sdk alongside this repo
    sdk = None
    version = None

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

    raise ImportError(
        "piper_sdk not found or neither V2 nor V1 classes are importable.\n"
        "Install with: pip install piper-sdk"
    )


def wait_for_ready(piper, sdk_version: str):
    """Block until the PiperX interface reports ready/enabled.

    V2: loop until EnablePiper() returns True
    V1: loop while DisablePiper() returns True (per user example)
    """
    start = time.time()
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
    elapsed = time.time() - start
    print(f"PiperX is ready (waited {elapsed:.2f}s)")


def read_joint_array(piper) -> List[int]:
    """Read current 6 joint values as a list of ints in PiperX units (deg*1000)."""
    msgs = piper.GetArmJointMsgs()
    # Expect msgs.joint_state.joint_1 .. joint_6
    try:
        js = msgs.joint_state
        values = [
            int(js.joint_1),
            int(js.joint_2),
            int(js.joint_3),
            int(js.joint_4),
            int(js.joint_5),
            int(js.joint_6),
        ]
        return values
    except Exception as e:
        raise RuntimeError(f"Unexpected joint message format: {e}")


def read_gripper_value(piper) -> int:
    """Read current gripper value in PiperX units (deg*1000)."""
    msgs = piper.GetArmGripperMsgs()
    try:
        return int(msgs.gripper_state.grippers_angle)
    except Exception as e:
        raise RuntimeError(f"Unexpected gripper message format: {e}")


def nonblocking_enter_pressed() -> bool:
    """Return True if Enter has been pressed (non-blocking), else False."""
    try:
        import select  # POSIX only
        import sys as _sys
        return select.select([_sys.stdin], [], [], 0) == ([_sys.stdin], [], [])
    except Exception:
        return False


def wait_for_enter_showing_all_joints(piper, joint_names: List[str], prompt: str) -> List[int]:
    """Continuously print all joint values until Enter is pressed, then return values."""
    print(f"\n{prompt}")
    last_print = 0.0
    while True:
        values = read_joint_array(piper)
        now = time.time()
        if now - last_print > 0.05:  # ~20Hz display
            line = "  " + "  ".join(
                f"J{i+1}({name}): {val:>7d}" for i, (name, val) in enumerate(zip(joint_names, values))
            )
            print(f"\r{line}", end="", flush=True)
            last_print = now

        if nonblocking_enter_pressed():
            try:
                input()  # consume
            except Exception:
                pass
            break

        time.sleep(0.02)

    print("\nRecorded all joint positions")
    return values


def wait_for_enter_showing_one_joint(piper, joint_index: int, joint_name: str, prompt: str) -> int:
    """Show one joint value continuously until Enter is pressed, return that value."""
    print(f"\n{prompt}")
    last_print = 0.0
    current_value = 0
    while True:
        values = read_joint_array(piper)
        current_value = int(values[joint_index])
        now = time.time()
        if now - last_print > 0.05:
            print(f"\rJoint {joint_index+1} ({joint_name}): {current_value:>7d}", end="", flush=True)
            last_print = now

        if nonblocking_enter_pressed():
            try:
                input()  # consume
            except Exception:
                pass
            break
        time.sleep(0.02)

    print(f"\nRecorded: {current_value}")
    return current_value


def save_calibration(
    filename: str,
    joint_names: List[str],
    neutral_positions: List[int],
    joint_calibration_positions: List[Tuple[int, int]],
    gripper_limits: Tuple[int, int],
    overwrite_mode: str = "ask",
):
    """Write calibration JSON to filename with backup/overwrite behavior.

    overwrite_mode: "ask" | "overwrite" | "backup"
    """
    # Confirm overwrite if needed
    if os.path.exists(filename):
        if overwrite_mode == "ask":
            print(f"\nWarning: Calibration file '{filename}' already exists!")
            print("Options:")
            print("  y - Overwrite existing file")
            print("  b - Create backup and save new calibration")
            print("  N - Cancel (default)")
            resp = input("Your choice (y/b/N): ").strip().lower()
            if resp == "b":
                overwrite_mode = "backup"
            elif resp == "y":
                overwrite_mode = "overwrite"
            else:
                print("Calibration cancelled. Existing file not modified.")
                return False

        if overwrite_mode == "backup":
            backup_filename = filename.replace(
                ".json", f"_backup_{time.strftime('%Y%m%d_%H%M%S')}.json"
            )
            try:
                import shutil
                shutil.copy2(filename, backup_filename)
                print(f"Backup created: {backup_filename}")
            except Exception as e:
                print(f"Warning: Could not create backup: {e}")
                cont = input("Continue without backup? (y/N): ").strip().lower()
                if cont != "y":
                    print("Calibration cancelled.")
                    return False

    data = {
        "robot_model": "PiperX",
        "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "joint_names": joint_names,
        "gripper_name": "gripper",
        "neutral_positions": [int(v) for v in neutral_positions],
        "joint_limits": [[int(a), int(b)] for (a, b) in joint_calibration_positions],
        "gripper_limits": [int(gripper_limits[0]), int(gripper_limits[1])],  # [closed, open]
    }

    with open(filename, "w") as f:
        json.dump(data, f, indent=2)

    print(f"\nCalibration data saved to: {filename}")
    return True


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PiperX calibration script")
    parser.add_argument(
        "--can",
        dest="can_name",
        default="can0",
        help="CAN interface name (e.g., can0, can1, can_left)",
    )
    parser.add_argument(
        "--sdk-version",
        choices=["auto", "v1", "v2"],
        default="auto",
        help="Force Piper SDK version (default: auto)",
    )
    parser.add_argument(
        "--output",
        default="piperx_calibration.json",
        help="Output calibration JSON path",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite output without prompting",
    )
    parser.add_argument(
        "--backup",
        action="store_true",
        help="Create a timestamped backup if output exists (no prompt)",
    )
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    PiperClass, sdk_version = try_import_piper_sdk(args.sdk_version)

    print("=== PiperX Calibration Script ===")
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

    # Connect and DISABLE arm for manual calibration
    print("Connecting to PiperX...")
    piper.ConnectPort()
    print("Disabling PiperX arm for manual calibration...")
    try:
        # Loop issuing disable until it reports disabled
        while piper.DisablePiper():
            time.sleep(0.01)
        print("Arm disabled successfully")
    except Exception as e:
        print(f"Warning: could not confirm disable state: {e}")

    # Joint/gripper names
    joint_names = [f"joint_{i}" for i in range(1, 7)]
    gripper_name = "gripper"

    try:
        # Optionally place gripper in a disabled/neutral state depending on SDK
        try:
            # Disable gripper torque (best-effort)
            piper.GripperCtrl(
                gripper_angle=0,
                gripper_effort=0,
                gripper_code=0x00,
                set_zero=0x00,
            )
        except Exception:
            pass

        # Step 1: Neutral position
        print("\n" + "=" * 60)
        print("STEP 1: NEUTRAL POSITION")
        print("=" * 60)
        neutral_positions = wait_for_enter_showing_all_joints(
            piper,
            joint_names,
            "Move the robot to NEUTRAL position and press Enter...",
        )
        print("\nNeutral position recorded:")
        for i, (name, pos) in enumerate(zip(joint_names, neutral_positions)):
            print(f"  Joint {i+1} ({name}): {pos}")

        # Step 2: Two calibration positions per joint
        print("\n" + "=" * 60)
        print("STEP 2: JOINT CALIBRATION POSITIONS")
        print("=" * 60)
        print("For each joint, record two corresponding positions used for mapping.")
        print("These are NOT min/max limits, but specific calibration poses.")

        joint_calib: List[Tuple[int, int]] = []
        for j_idx, j_name in enumerate(joint_names):
            print(f"\n--- Joint {j_idx+1}: {j_name} ---")
            pos1 = wait_for_enter_showing_one_joint(
                piper,
                j_idx,
                j_name,
                f"Move Joint {j_idx+1} ({j_name}) to FIRST calibration position and press Enter...",
            )
            pos2 = wait_for_enter_showing_one_joint(
                piper,
                j_idx,
                j_name,
                f"Move Joint {j_idx+1} ({j_name}) to SECOND calibration position and press Enter...",
            )
            joint_calib.append((pos1, pos2))
            print(
                f"Joint {j_idx+1} ({j_name}) calibration positions: [{pos1}, {pos2}] (deg*1000)"
            )

        # Step 3: Gripper limits
        print("\n" + "=" * 60)
        print("STEP 3: GRIPPER LIMITS")
        print("=" * 60)
        print(f"\n--- Gripper: {gripper_name} ---")

        print("Move gripper to OPEN position and press Enter...")
        last_print = 0.0
        while True:
            g_val_open = read_gripper_value(piper)
            now = time.time()
            if now - last_print > 0.05:
                print(f"\rGripper ({gripper_name}) OPEN: {g_val_open:>7d}", end="", flush=True)
                last_print = now
            if nonblocking_enter_pressed():
                try:
                    input()
                except Exception:
                    pass
                break
            time.sleep(0.02)
        print(f"\nGripper OPEN position recorded: {g_val_open}")

        print("Move gripper to CLOSED position and press Enter...")
        last_print = 0.0
        while True:
            g_val_closed = read_gripper_value(piper)
            now = time.time()
            if now - last_print > 0.05:
                print(f"\rGripper ({gripper_name}) CLOSED: {g_val_closed:>7d}", end="", flush=True)
                last_print = now
            if nonblocking_enter_pressed():
                try:
                    input()
                except Exception:
                    pass
                break
            time.sleep(0.02)
        print(f"\nGripper CLOSED position recorded: {g_val_closed}")

        # Step 4: Save
        print("\n" + "=" * 60)
        print("CALIBRATION COMPLETE")
        print("=" * 60)

        overwrite_mode = "ask"
        if args.overwrite:
            overwrite_mode = "overwrite"
        elif args.backup:
            overwrite_mode = "backup"

        ok = save_calibration(
            filename=args.output,
            joint_names=joint_names,
            neutral_positions=neutral_positions,
            joint_calibration_positions=joint_calib,
            gripper_limits=(g_val_closed, g_val_open),
            overwrite_mode=overwrite_mode,
        )
        if not ok:
            sys.exit(1)

        # Echo summary
        print("\nNeutral positions:")
        for i, (name, pos) in enumerate(zip(joint_names, neutral_positions)):
            print(f"  Joint {i+1} ({name:10s}): {pos:>7d}")

        print("\nJoint calibration positions:")
        for i, (name, (a, b)) in enumerate(zip(joint_names, joint_calib)):
            print(f"  Joint {i+1} ({name:10s}): [{a:>7d}, {b:>7d}] (deg*1000)")

        print("\nGripper limits (deg*1000):")
        print(f"  {gripper_name:10s}: [closed={g_val_closed:>7d}, open={g_val_open:>7d}]")

    except KeyboardInterrupt:
        print("\nCalibration interrupted by user.")
        sys.exit(1)
    finally:
        # Best-effort disable/disconnect
        try:
            try:
                # V2 example uses DisableArm at end in repo
                piper.DisableArm(motor_num=7, enable_flag=0x01)
            except Exception:
                pass
            try:
                piper.DisconnectPort()
            except Exception:
                pass
        except Exception:
            pass


if __name__ == "__main__":
    main()


