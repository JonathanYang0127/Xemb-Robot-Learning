#!/usr/bin/env python3
"""
Continuous PiperX demo based on user example
Shows alternative control approach with EnablePiper() and MotionCtrl_2()
"""

import time
from piper_sdk import C_PiperInterface_V2

def main():
    print("Initializing PiperX robot (continuous demo)...")
    
    # Initialize PiperX robot
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    
    # Wait for robot to enable
    while not piper.EnablePiper():
        time.sleep(0.01)
    
    print("Robot enabled successfully")
    
    # Initialize gripper
    piper.GripperCtrl(0, 1000, 0x01, 0)
    
    # Conversion factor: 1000 * 180 / π (for converting from radians if needed)
    factor = 57295.7795
    
    # Define demo positions in radians (will be converted to PiperX units)
    positions = [
        [0, 0, 0, 0, 0, 0, 0],           # Neutral position
        [0.2, 0.2, -0.2, 0.3, -0.2, 0.5, 0.08],  # Demo position 1
        [0, 0, 0, 0, 0, 0, 0],           # Back to neutral
    ]
    
    position_names = ["Neutral", "Demo Position", "Back to Neutral"]
    
    print("Starting continuous demo...")
    print("Press Ctrl+C to stop")
    
    try:
        count = 0
        current_pos_idx = 0
        
        while True:
            count += 1
            
            # Switch positions every 300 cycles
            if count % 300 == 0:
                current_pos_idx = (current_pos_idx + 1) % len(positions)
                print(f"Moving to: {position_names[current_pos_idx]}")
            
            # Get current position
            position = positions[current_pos_idx]
            
            # Convert to PiperX units
            joint_0 = round(position[0] * factor)
            joint_1 = round(position[1] * factor)
            joint_2 = round(position[2] * factor)
            joint_3 = round(position[3] * factor)
            joint_4 = round(position[4] * factor)
            joint_5 = round(position[5] * factor)
            joint_6 = round(position[6] * 1000 * 1000)  # Gripper in different units
            
            # Set motion control parameters
            piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            
            # Send joint commands
            piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
            
            # Control gripper
            piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
            
            # Print status every 100 cycles
            if count % 100 == 0:
                try:
                    status = piper.GetArmStatus()
                    print(f"Count: {count}, Status: {status}")
                    print(f"Target position: {position}")
                except Exception as e:
                    print(f"Could not get status: {e}")
            
            time.sleep(0.005)  # 200Hz control loop
            
    except KeyboardInterrupt:
        print("\nDemo stopped by user")
    
    finally:
        print("Shutting down robot...")
        try:
            # Move to neutral position before shutdown
            piper.JointCtrl(0, 0, 0, 0, 0, 0)
            piper.GripperCtrl(0, 1000, 0x01, 0)
            time.sleep(1.0)
            
            # Disable robot (method may vary)
            # piper.DisablePiper()  # If this method exists
            
        except Exception as e:
            print(f"Error during shutdown: {e}")
        
        print("Demo complete")

if __name__ == "__main__":
    main() 