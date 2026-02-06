#!/usr/bin/env python3
"""
Test script to move PiperX robot to neutral position.
This script demonstrates basic PiperX SDK usage and tests the neutral position calibration.
"""

import json
import time
import sys
import os

# Add the parent directory to the path to import piper SDK
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Error: piper_sdk not found. Please install with: pip install piper-sdk")
    sys.exit(1)

def load_calibration():
    """Load PiperX calibration data"""
    try:
        with open('piperx_calibration.json', 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print("Error: piperx_calibration.json not found")
        sys.exit(1)
    except json.JSONDecodeError:
        print("Error: Invalid JSON in piperx_calibration.json")
        sys.exit(1)

def test_piperx_neutral():
    """Test moving PiperX to neutral position"""
    print("Loading PiperX calibration...")
    calib = load_calibration()
    
    # Extract neutral positions
    neutral_positions = calib['neutral_positions']
    joint_names = calib['joint_names']
    
    print(f"Robot model: {calib['robot_model']}")
    print(f"Joint names: {joint_names}")
    print(f"Neutral positions: {neutral_positions}")
    
    # Initialize PiperX interface
    print("\nInitializing PiperX interface...")
    try:
        piper = C_PiperInterface_V2("can0")
        piper.ConnectPort()
        
        # Wait for robot to enable (alternative method)
        while not piper.EnablePiper():
            time.sleep(0.01)
        
        print("PiperX interface initialized and enabled successfully")
    except Exception as e:
        print(f"Error initializing PiperX interface: {e}")
        print("Make sure CAN interfaces are configured properly")
        return False
    
    # Initialize gripper
    print("\nInitializing gripper...")
    try:
        piper.GripperCtrl(0, 1000, 0x01, 0)
        print("Gripper initialized successfully")
    except Exception as e:
        print(f"Error initializing gripper: {e}")
        return False
    
    # Move to neutral position
    print("\nMoving to neutral position...")
    try:
        # The calibration file stores positions in PiperX units (degrees*1000)
        # These are already in the correct format for PiperX
        joint_positions = []
        for i, pos in enumerate(neutral_positions):
            joint_positions.append(int(pos))
            print(f"  {joint_names[i]}: {pos} (degrees*1000)")
        
        # Move joints to neutral position
        print("\nSending joint commands...")
        
        # Set motion control parameters (based on user example)
        piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        
        # Send joint command
        piper.JointCtrl(*joint_positions)
        for i, pos in enumerate(joint_positions):
            joint_id = i + 1  # PiperX joints are 1-indexed
            print(f"  Joint {joint_id}: {pos}")
        
        # Wait for movement to complete
        print("\nWaiting for movement to complete...")
        time.sleep(3.0)
        
        # Get current joint positions to verify
        print("\nVerifying final positions...")
        try:
            current_joints = piper.GetArmJointMsgs()
            print(f"Current joint positions: {current_joints}")
            
            # Compare with expected positions if possible
            if hasattr(current_joints, 'joint_state'):
                joint_values = [
                    current_joints.joint_state.joint_1,
                    current_joints.joint_state.joint_2,
                    current_joints.joint_state.joint_3,
                    current_joints.joint_state.joint_4,
                    current_joints.joint_state.joint_5,
                    current_joints.joint_state.joint_6
                ]
                for i, (current, expected) in enumerate(zip(joint_values, joint_positions)):
                    error = abs(current - expected)
                    print(f"  Joint {i+1}: Current={current}, Expected={expected}, Error={error}")
        except Exception as e:
            print(f"Could not read current positions: {e}")
        
        print("\nNeutral position test completed successfully!")
        return True
        
    except Exception as e:
        print(f"Error during movement: {e}")
        return False
    
    finally:
        # Disable the robot
        print("\nDisabling robot...")
        try:
            piper.DisableArm(motor_num=7, enable_flag=0x01)
            piper.DisconnectPort()
            print("Robot disabled successfully")
        except Exception as e:
            print(f"Error disabling robot: {e}")

def main():
    """Main function"""
    print("=== PiperX Neutral Position Test ===")
    print("This script will move the PiperX robot to its neutral position")
    print("Make sure the robot is properly connected and CAN interfaces are configured")
    print()
    
    # Ask for confirmation
    response = input("Continue? (y/N): ").strip().lower()
    if response != 'y':
        print("Test cancelled")
        return
    
    # Run the test
    success = test_piperx_neutral()
    
    if success:
        print("\n✅ Test completed successfully!")
    else:
        print("\n❌ Test failed!")
        sys.exit(1)

if __name__ == "__main__":
    main() 