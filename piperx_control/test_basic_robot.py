#!/usr/bin/env python3
"""
Basic PiperX Robot Test Script
Test direct joint control without IK to verify robot functionality
"""

import sys
import time
import numpy as np
import json
import argparse

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Error: piper_sdk not found. Please install with: pip install piper-sdk")
    sys.exit(1)

class BasicRobotTest:
    def __init__(self, can_interface='can0'):
        """Initialize basic robot test."""
        self.can_interface = can_interface
        
        # Load calibration
        self.load_calibration()
        
        # Initialize PiperX robot
        print(f"🔧 TEST: Connecting to PiperX on {can_interface}...")
        try:
            self.piper = C_PiperInterface_V2(can_interface)
            self.piper.ConnectPort()
            
            # Wait for connection
            timeout = 5.0
            start_time = time.time()
            while not self.piper.get_connect_status():
                if time.time() - start_time > timeout:
                    print(f"❌ TEST: Connection timeout for {can_interface}")
                    sys.exit(1)
                time.sleep(0.01)
            
            # Enable the robot
            start_time = time.time()
            while not self.piper.EnablePiper():
                if time.time() - start_time > timeout:
                    print(f"❌ TEST: Enable timeout for {can_interface}")
                    sys.exit(1)
                time.sleep(0.01)
            
            print(f"✅ TEST: PiperX connected on {can_interface}")
            
        except Exception as e:
            print(f"❌ TEST: Failed to connect to PiperX: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
    
    def load_calibration(self):
        """Load PiperX calibration data."""
        try:
            with open('piperx_calibration.json', 'r') as f:
                calib = json.load(f)
            self.neutral_positions = calib['neutral_positions']
            self.gripper_closed = calib['gripper_limits'][0]
            self.gripper_open = calib['gripper_limits'][1]
            print(f"✅ TEST: Loaded calibration: neutral={self.neutral_positions}")
        except Exception as e:
            print(f"⚠️  TEST: Could not load calibration: {e}")
            print("🔧 TEST: Using default values")
            self.neutral_positions = [0, 0, 0, 0, 0, 0]
            self.gripper_closed = 0
            self.gripper_open = 30000
    
    def move_to_position(self, joint_positions, speed=100):
        """Move to specific joint positions."""
        try:
            print(f"🔧 TEST: Moving to position: {joint_positions}")
            
            # Send motion control command
            self.piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
            
            # Send joint command
            result = self.piper.JointCtrl(*[int(pos) for pos in joint_positions])
            print(f"🔧 TEST: Joint command result: {result}")
            
            return True
            
        except Exception as e:
            print(f"❌ TEST: Error moving to position: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def test_gripper(self):
        """Test gripper functionality."""
        print("🔧 TEST: Testing gripper...")
        
        # Test gripper open
        print("🔧 TEST: Opening gripper...")
        self.piper.GripperCtrl(int(self.gripper_open), 1000, 0x01, 0)
        time.sleep(2.0)
        
        # Test gripper close
        print("🔧 TEST: Closing gripper...")
        self.piper.GripperCtrl(int(self.gripper_closed), 1000, 0x01, 0)
        time.sleep(2.0)
        
        print("✅ TEST: Gripper test completed")
    
    def test_individual_joints(self):
        """Test each joint individually."""
        print("🔧 TEST: Testing individual joints...")
        
        # Test each joint with small movements
        base_positions = list(self.neutral_positions)
        
        for joint_idx in range(6):
            print(f"🔧 TEST: Testing joint {joint_idx}...")
            
            # Move joint positive
            test_positions = base_positions.copy()
            test_positions[joint_idx] += 5000  # 5000 millidegrees
            
            print(f"🔧 TEST: Moving joint {joint_idx} positive: {test_positions}")
            if self.move_to_position(test_positions):
                time.sleep(1.0)
                
                # Move joint negative
                test_positions[joint_idx] -= 10000  # -10000 millidegrees from neutral
                
                print(f"🔧 TEST: Moving joint {joint_idx} negative: {test_positions}")
                if self.move_to_position(test_positions):
                    time.sleep(1.0)
                    
                    # Return to neutral
                    print(f"🔧 TEST: Returning joint {joint_idx} to neutral")
                    if self.move_to_position(base_positions):
                        time.sleep(1.0)
                        print(f"✅ TEST: Joint {joint_idx} test completed")
                    else:
                        print(f"❌ TEST: Joint {joint_idx} failed to return to neutral")
                else:
                    print(f"❌ TEST: Joint {joint_idx} failed negative move")
            else:
                print(f"❌ TEST: Joint {joint_idx} failed positive move")
    
    def test_small_movements(self):
        """Test small incremental movements."""
        print("🔧 TEST: Testing small incremental movements...")
        
        # Start from neutral
        current_pos = list(self.neutral_positions)
        
        # Test small movements on joint 0 (base rotation)
        for i in range(10):
            current_pos[0] += 1000  # 1000 millidegrees per step
            print(f"🔧 TEST: Small movement {i+1}: joint 0 = {current_pos[0]}")
            
            if self.move_to_position(current_pos):
                time.sleep(0.5)
            else:
                print(f"❌ TEST: Small movement {i+1} failed")
                break
        
        # Return to neutral
        print("🔧 TEST: Returning to neutral from small movements")
        self.move_to_position(self.neutral_positions)
        time.sleep(1.0)
        
        print("✅ TEST: Small movements test completed")
    
    def test_rapid_commands(self):
        """Test rapid command sending."""
        print("🔧 TEST: Testing rapid command sending...")
        
        base_pos = list(self.neutral_positions)
        
        # Send rapid commands
        for i in range(20):
            test_pos = base_pos.copy()
            test_pos[0] = base_pos[0] + 2000 * np.sin(i * 0.5)  # Sinusoidal motion
            
            print(f"🔧 TEST: Rapid command {i+1}: {test_pos[0]:.0f}")
            
            if self.move_to_position(test_pos, speed=200):
                time.sleep(0.1)  # Very short delay
            else:
                print(f"❌ TEST: Rapid command {i+1} failed")
                break
        
        # Return to neutral
        print("🔧 TEST: Returning to neutral from rapid commands")
        self.move_to_position(self.neutral_positions)
        time.sleep(1.0)
        
        print("✅ TEST: Rapid commands test completed")
    
    def run_all_tests(self):
        """Run all tests."""
        print("🚀 TEST: Starting comprehensive robot tests...")
        
        # Initialize to neutral
        print("🔧 TEST: Moving to neutral position...")
        self.move_to_position(self.neutral_positions)
        time.sleep(2.0)
        
        # Test gripper
        self.test_gripper()
        
        # Test individual joints
        self.test_individual_joints()
        
        # Test small movements
        self.test_small_movements()
        
        # Test rapid commands
        self.test_rapid_commands()
        
        # Final return to neutral
        print("🔧 TEST: Final return to neutral position...")
        self.move_to_position(self.neutral_positions)
        self.piper.GripperCtrl(int(self.gripper_closed), 1000, 0x01, 0)
        time.sleep(1.0)
        
        print("✅ TEST: All tests completed successfully!")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Basic PiperX Robot Test')
    parser.add_argument('--can', default='can0', help='CAN interface for PiperX (default: can0)')
    parser.add_argument('--test', choices=['all', 'gripper', 'joints', 'small', 'rapid'], 
                        default='all', help='Which test to run (default: all)')
    args = parser.parse_args()
    
    try:
        print("🔧 TEST: Starting basic robot test...")
        
        # Create test instance
        test = BasicRobotTest(can_interface=args.can)
        
        # Run selected test
        if args.test == 'all':
            test.run_all_tests()
        elif args.test == 'gripper':
            test.test_gripper()
        elif args.test == 'joints':
            test.test_individual_joints()
        elif args.test == 'small':
            test.test_small_movements()
        elif args.test == 'rapid':
            test.test_rapid_commands()
            
    except KeyboardInterrupt:
        print("\n🔧 TEST: Test stopped by user.")
    except Exception as e:
        print(f"❌ TEST: Error in test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main() 
