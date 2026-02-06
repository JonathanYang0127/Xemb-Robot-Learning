#!/usr/bin/env python3
"""
Simple PiperX demo using the proper SDK
"""

import time
import numpy as np
from piper_sdk import C_PiperInterface_V2

def main():
    print("Initializing PiperX robot...")
    
    # Initialize PiperX robot
    piper = C_PiperInterface_V2(can_name="can0")
    
    # Connect to robot
    piper.ConnectPort()
    
    # Wait for connection
    while not piper.get_connect_status():
        time.sleep(0.01)
    
    print("Connected to PiperX robot")
    
    # Set to CAN command control mode with joint movement
    piper.ModeCtrl(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=50)
    
    # Enable all motors
    piper.EnableArm(motor_num=7, enable_flag=0x02)
    
    # Enable gripper
    piper.GripperCtrl(gripper_angle=0, gripper_effort=1000, gripper_code=0x01, set_zero=0x00)
    
    print("Robot enabled. Starting demo...")
    
    # Demo positions (in degrees * 1000)
    positions = [
        [0, 90000, -85000, 0, -5000, 0],      # Neutral position
        [30000, 90000, -85000, 0, -5000, 0],  # Move joint 1
        [0, 120000, -85000, 0, -5000, 0],     # Move joint 2
        [0, 90000, -50000, 0, -5000, 0],      # Move joint 3
        [0, 90000, -85000, 0, -5000, 0],      # Back to neutral
    ]
    
    gripper_positions = [0, 35000, 0, 35000, 0]  # Alternate gripper open/close
    
    try:
        for i, (pos, grip) in enumerate(zip(positions, gripper_positions)):
            print(f"Moving to position {i+1}: {pos}")
            print(f"Gripper: {grip}")
            
            # Send joint command
            piper.JointCtrl(*pos)
            
            # Send gripper command
            piper.GripperCtrl(gripper_angle=grip, gripper_effort=1000, gripper_code=0x01)
            
            # Wait for movement to complete
            time.sleep(2.0)
            
            # Print current joint positions
            current_joints = piper.GetArmJointMsgs()
            print(f"Current joints: {current_joints}")
            
            # Print current gripper position
            current_gripper = piper.GetArmGripperMsgs()
            print(f"Current gripper: {current_gripper}")
            
            print("-" * 40)
    
    except KeyboardInterrupt:
        print("\nDemo stopped by user")
    
    finally:
        print("Disabling robot...")
        # Disable all motors
        piper.DisableArm(motor_num=7, enable_flag=0x01)
        
        # Disconnect
        piper.DisconnectPort()
        
        print("Demo complete")

if __name__ == "__main__":
    main() 