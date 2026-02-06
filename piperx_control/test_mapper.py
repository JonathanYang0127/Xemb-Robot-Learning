#!/usr/bin/env python3
"""
Test script for the JointMapper class
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from robot_mapper import JointMapper

def test_joint_mapping():
    """Test the joint mapping functionality"""
    print("Testing Joint Mapping...")
    
    try:
        # Initialize the mapper
        mapper = JointMapper(
            master_calib_path='widowx_calibration.json',
            puppet_calib_path='piperx_calibration.json'
        )
        
        # Show joint calibration positions information
        mapper.get_joint_limits_info()
        
        # Test mapping range
        mapper.test_mapping_range()
        
        # Test gripper mapping specifically
        mapper.test_gripper_mapping()
        
        # Test with neutral positions
        print("\n=== Testing with neutral positions ===")
        master_neutral, puppet_neutral = mapper.get_neutral_positions()
        print(f"Master neutral positions: {master_neutral}")
        print(f"Puppet neutral positions: {puppet_neutral}")
        
        # Map master neutral to puppet space
        mapped_joints = mapper.map_joints(master_neutral)
        print(f"Mapped joint positions (rad): {[f'{x:.3f}' for x in mapped_joints]}")
        
        # Test gripper mapping
        print("\n=== Testing gripper mapping ===")
        master_gripper_test = [-1.0, -1.2, -1.4, -0.8]  # Test values
        for grip_val in master_gripper_test:
            mapped_grip = mapper.map_gripper(grip_val)
            print(f"Master gripper {grip_val:.3f} rad -> Puppet gripper {mapped_grip:.3f} rad")
        
        # Test with some arbitrary joint values
        print("\n=== Testing with arbitrary joint values ===")
        test_joints = [0.5, -0.5, 1.0, -1.0, 0.25, -0.25]
        mapped_test = mapper.map_joints(test_joints)
        print(f"Test joints (rad): {[f'{x:.3f}' for x in test_joints]}")
        print(f"Mapped joints (rad): {[f'{x:.3f}' for x in mapped_test]}")
        
        # Note: The mapper outputs radians (consistent with WidowX)
        # The puppet script will convert these back to degrees*1000 for PiperX
        mapped_test_deg1000 = [angle * 180.0 / 3.14159 * 1000 for angle in mapped_test]
        print(f"Mapped joints (PiperX units): {[f'{x:.0f}' for x in mapped_test_deg1000]}")
        
        print("\n✓ All tests passed!")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_joint_mapping() 