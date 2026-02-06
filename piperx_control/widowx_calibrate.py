#!/usr/bin/env python3
"""
widowx_calibrate.py - Calibration script for WidowX arm joint limits
"""

import time
import json
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from constants import *
from robot_utils import torque_on

def display_current_joint_value(arm, joint_idx, joint_name, prompt_message):
    """Display current joint value and wait for user input"""
    print(f"\n{prompt_message}")
    
    while True:
        current_values = list(arm.dxl.joint_states.position[:6])
        current_value = current_values[joint_idx]
        print(f"\rJoint {joint_idx} ({joint_name}): {current_value:8.4f} rad", end="", flush=True)
        
        # Check if user pressed enter (non-blocking)
        import select
        import sys
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            input()  # consume the enter
            break
        
        time.sleep(0.1)
    
    print(f"\nRecorded: {current_value:.4f} rad")
    return current_value

def wait_for_enter_simple(arm, prompt_message):
    """Simple wait for enter while showing all joint values"""
    print(f"\n{prompt_message}")
    
    while True:
        current_values = list(arm.dxl.joint_states.position[:6])
        joint_names = arm.arm.group_info.joint_names
        
        print(f"\r", end="")
        for i, (name, val) in enumerate(zip(joint_names, current_values)):
            print(f"J{i}({name}): {val:.3f}  ", end="")
        print("", end="", flush=True)
        
        import select
        import sys
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            input()  # consume the enter
            break
        
        time.sleep(0.1)
    
    print(f"\nRecorded all joint positions")
    return current_values

def save_calibration_data(neutral_positions, joint_calibration_positions, gripper_limits, joint_names, gripper_name, filename="widowx_calibration.json"):
    """Save calibration data to a JSON file"""
    import os
    
    # Check if file already exists
    if os.path.exists(filename):
        print(f"\nWarning: Calibration file '{filename}' already exists!")
        print("Options:")
        print("  y - Overwrite existing file")
        print("  b - Create backup and save new calibration")
        print("  N - Cancel (default)")
        response = input("Your choice (y/b/N): ").strip().lower()
        
        if response == 'b':
            # Create backup
            backup_filename = filename.replace('.json', f'_backup_{time.strftime("%Y%m%d_%H%M%S")}.json')
            try:
                import shutil
                shutil.copy2(filename, backup_filename)
                print(f"Backup created: {backup_filename}")
            except Exception as e:
                print(f"Warning: Could not create backup: {e}")
                backup_response = input("Continue without backup? (y/N): ").strip().lower()
                if backup_response != 'y':
                    print("Calibration cancelled.")
                    return False
        elif response != 'y':
            print("Calibration cancelled. Existing file not modified.")
            return False
    
    calibration_data = {
        "robot_model": "wx250s",
        "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "joint_names": joint_names,
        "gripper_name": gripper_name,
        "neutral_positions": neutral_positions,
        "joint_limits": joint_calibration_positions,  # Two corresponding signed positions for mapping
        "gripper_limits": gripper_limits  # [closed, open]
    }
    
    try:
        with open(filename, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"\nCalibration data saved to: {filename}")
        return True
    except Exception as e:
        print(f"Error saving calibration file: {e}")
        return False

def main():
    import os
    
    print("=== WidowX Calibration Script ===")
    
    # Check if calibration file already exists
    if os.path.exists("widowx_calibration.json"):
        print("\nNote: A calibration file 'widowx_calibration.json' already exists.")
        print("You will be asked whether to overwrite it at the end of the calibration process.")
    
    # Initialize the master left arm
    print("\nInitializing master left arm...")
    
    master_left = InterbotixManipulatorXS(
        robot_model="wx250s", 
        group_name="arm", 
        gripper_name="gripper", 
        robot_name='master_left', 
        init_node=True
    )
    
    # Keep arm relaxed for manual positioning
    # Do not torque on - we want to move it manually
    
    # Get joint information
    joint_names = master_left.arm.group_info.joint_names
    num_joints = len(joint_names)
    gripper_name = 'gripper_joint'

    print(f"\nWidowX Arm Calibration")
    print(f"Number of arm joints: {num_joints}")
    print(f"Arm joint names: {joint_names}")
    print(f"Gripper joint name: {gripper_name}")
    
    # Wait for arm to be ready
    time.sleep(1)
    
    try:
        # Step 1: Record neutral position
        print("\n" + "="*60)
        print("STEP 1: NEUTRAL POSITION")
        print("="*60)
        
        neutral_positions = wait_for_enter_simple(
            master_left,
            "Move the robot to NEUTRAL position and press Enter..."
        )
        
        print(f"\nNeutral position recorded:")
        for i, (name, pos) in enumerate(zip(joint_names, neutral_positions)):
            print(f"  Joint {i} ({name}): {pos:.4f} rad")
        
        # Step 2: Record calibration positions for each joint
        print("\n" + "="*60)
        print("STEP 2: JOINT CALIBRATION POSITIONS")
        print("="*60)
        print("For each joint, you will record two corresponding positions that")
        print("will be used to map between master and puppet robots.")
        print("These are NOT min/max limits, but specific calibration poses.")
        
        joint_limits = []
        
        for i, joint_name in enumerate(joint_names):
            print(f"\n--- Joint {i}: {joint_name} ---")
            
            # Record first calibration position
            pos1_value = display_current_joint_value(
                master_left, i, joint_name,
                f"Move Joint {i} ({joint_name}) to FIRST calibration position and press Enter..."
            )
            
            # Record second calibration position
            pos2_value = display_current_joint_value(
                master_left, i, joint_name,
                f"Move Joint {i} ({joint_name}) to SECOND calibration position and press Enter..."
            )
            
            joint_limits.append([pos1_value, pos2_value])
            
            print(f"Joint {i} ({joint_name}) calibration positions: [{pos1_value:.4f}, {pos2_value:.4f}] rad")
        
        # Step 3: Record gripper limits
        print("\n" + "="*60)
        print("STEP 3: GRIPPER LIMITS")
        print("="*60)
        
        print(f"\n--- Gripper: {gripper_name} ---")
        
        print(f"\nMove gripper to OPEN position and press Enter...")
        while True:
            gripper_value = master_left.dxl.joint_states.position[6]
            print(f"\rGripper ({gripper_name}): {gripper_value:8.4f} rad", end="", flush=True)
            
            import select
            import sys
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                input()  # consume the enter
                break
            
            time.sleep(0.1)
        
        gripper_open = gripper_value
        print(f"\nGripper OPEN position recorded: {gripper_open:.4f} rad")
        
        # Record gripper closed value
        print(f"\nMove gripper to CLOSED position and press Enter...")
        while True:
            gripper_value = master_left.dxl.joint_states.position[6]
            print(f"\rGripper ({gripper_name}): {gripper_value:8.4f} rad", end="", flush=True)
            
            import select
            import sys
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                input()  # consume the enter
                break
            
            time.sleep(0.1)
        
        gripper_closed = gripper_value
        print(f"\nGripper CLOSED position recorded: {gripper_closed:.4f} rad")
        
        gripper_limits = [gripper_closed, gripper_open]  # [closed, open]
        
        # Step 4: Save and display results
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE")
        print("="*60)
        
        # Save to file
        if not save_calibration_data(neutral_positions, joint_limits, gripper_limits, joint_names, gripper_name):
            print("Calibration data was not saved.")
            return
        
        print(f"\nNeutral positions:")
        for i, (name, pos) in enumerate(zip(joint_names, neutral_positions)):
            print(f"  Joint {i} ({name:15s}): {pos:8.4f} rad")
        
        print(f"\nJoint calibration positions:")
        for i, (name, positions) in enumerate(zip(joint_names, joint_limits)):
            print(f"  Joint {i} ({name:15s}): [{positions[0]:8.4f}, {positions[1]:8.4f}] rad")
        
        print(f"\nGripper limits:")
        print(f"  {gripper_name:15s}: [{gripper_limits[0]:8.4f}, {gripper_limits[1]:8.4f}] rad (closed, open)")
        
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user.")
    
    finally:
        print("\nShutting down...")

if __name__ == '__main__':
    main()

