#!/usr/bin/env python3
"""
Test script to validate PiperX IK setup and diagnose potential issues
"""

import sys
import numpy as np
import json
from inverse_kinematics.piper_pinocchio_ik import PiperIK

def test_ik_setup():
    """Test the IK setup and joint limits."""
    print("🔍 Testing PiperX IK Setup...")
    
    try:
        # Initialize Pinocchio IK solver
        print("\n1. Initializing Pinocchio IK solver...")
        ik = PiperIK(max_iterations=100, tolerance=1e-3, damping=1e-3)
        print("✅ Pinocchio IK solver initialized successfully")
        
        # Load calibration data
        print("\n2. Loading calibration data...")
        try:
            with open('piperx_calibration.json', 'r') as f:
                calib = json.load(f)
            neutral_positions = calib['neutral_positions']
            print(f"✅ Loaded neutral positions: {neutral_positions}")
        except Exception as e:
            print(f"❌ Could not load calibration: {e}")
            return False
        
        # Test neutral position
        print("\n3. Testing neutral position...")
        neutral_rad = np.array(neutral_positions) * np.pi / 180000.0
        print(f"Neutral positions in radians: {neutral_rad}")
        
        # Check if neutral position is within IK limits
        full_config = ik.get_full_configuration(neutral_rad)
        within_limits = ik.check_joint_limits(full_config)
        print(f"Neutral position within IK limits: {within_limits}")
        
        if not within_limits:
            print("❌ Neutral position is outside IK joint limits!")
            print(f"Joint limits lower: {ik.joint_limits_lower[:6]}")
            print(f"Joint limits upper: {ik.joint_limits_upper[:6]}")
            return False
        
        # Test forward kinematics
        print("\n4. Testing forward kinematics...")
        fk_result = ik.forward_kinematics(full_config)
        if fk_result:
            pos = fk_result['position']
            ori = fk_result['orientation']
            print(f"✅ Forward kinematics successful")
            print(f"End-effector position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
            print(f"End-effector orientation: [{ori[0]:.3f}, {ori[1]:.3f}, {ori[2]:.3f}, {ori[3]:.3f}]")
        else:
            print("❌ Forward kinematics failed!")
            return False
        
        # Test Jacobian computation
        print("\n5. Testing Jacobian computation...")
        jacobian = ik.compute_jacobian(full_config)
        if jacobian is not None:
            print(f"✅ Jacobian computation successful")
            print(f"Jacobian shape: {jacobian.shape}")
            
            # Check for singularities
            manipulability = ik.compute_manipulability(neutral_rad)
            print(f"Manipulability index: {manipulability:.6f}")
            
            if manipulability < 1e-5:
                print("⚠️ Warning: Robot is in or near a singular configuration")
            
        else:
            print("❌ Jacobian computation failed!")
            return False
        
        # Test small velocity command
        print("\n6. Testing velocity IK...")
        test_twist = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])  # Small forward motion
        joint_velocities = ik.velocity_ik(test_twist, full_config)
        
        if joint_velocities is not None:
            print(f"✅ Velocity IK successful")
            print(f"Joint velocities: {joint_velocities[:6]}")
        else:
            print("❌ Velocity IK failed!")
            return False
        
        # Test joint limits for small motions
        print("\n7. Testing joint limit checking...")
        dt = 0.05
        new_joints = neutral_rad + joint_velocities[:6] * dt
        new_full_config = ik.get_full_configuration(new_joints)
        
        if ik.check_joint_limits(new_full_config):
            print("✅ Small motion within joint limits")
        else:
            print("❌ Small motion violates joint limits")
            return False
        
        print("\n🎉 All tests passed! IK setup is working correctly.")
        return True
        
    except Exception as e:
        print(f"❌ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main function."""
    success = test_ik_setup()
    
    if success:
        print("\n✅ Setup validation completed successfully.")
        print("You can now run the keyboard teleoperation with:")
        print("  python keyboard_piper_teleop.py")
        print("  python keyboard_piper_teleop.py --debug  # for verbose output")
    else:
        print("\n❌ Setup validation failed.")
        print("Please check your calibration files and IK setup.")
    
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main() 