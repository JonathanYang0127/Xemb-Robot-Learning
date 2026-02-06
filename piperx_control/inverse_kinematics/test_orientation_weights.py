#!/usr/bin/env python3

import numpy as np
import sys
import os

# Add the current directory to the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from piper_pinocchio_ik import PiperIK
import pinocchio as pin

def test_orientation_weights():
    """Test different orientation weights with a challenging target pose."""
    print("Testing Orientation Weight Effects...")
    
    # Create IK solver with verbose output
    ik = PiperIK(max_iterations=50, tolerance=1e-3, damping=1e-3, verbose=True)
    
    # Use a starting configuration that's far from the target
    start_joints = np.array([0.5, 1.0, -1.0, 0.8, -0.5, 1.0])
    start_config_full = ik.get_full_configuration(start_joints)
    
    # Get the forward kinematics for this starting position
    fk_start = ik.forward_kinematics(start_config_full)
    print(f"Starting position: {fk_start['position']}")
    print(f"Starting orientation: {fk_start['orientation']}")
    
    # Create a challenging target with significant position AND orientation change
    target_position = np.array(fk_start['position']) + np.array([0.08, -0.05, 0.06])  # 8cm, 5cm, 6cm offset
    
    # Create a significant orientation change (~20 degrees in each axis)
    pert_quat = pin.Quaternion(pin.utils.rpyToMatrix(0.35, -0.25, 0.30))  # ~20 deg rotations
    start_ori_quat = pin.Quaternion(np.array(fk_start['orientation']))
    target_ori_quat = pert_quat * start_ori_quat
    target_orientation = target_ori_quat.coeffs()  # [x, y, z, w]
    
    print(f"Target position: {target_position}")
    print(f"Target orientation: {target_orientation}")
    
    # Test different orientation weights
    weights_to_test = [1.0, 0.5, 0.2, 0.1]
    
    for weight in weights_to_test:
        print(f"\n{'='*60}")
        print(f"TESTING ORIENTATION WEIGHT: {weight}")
        print(f"{'='*60}")
        
        ik.set_solver_params(orientation_weight=weight)
        
        # Reset to starting position for each test
        current_joints = start_config_full.copy()
        
        # Solve IK
        solution = ik.inverse_kinematics(
            target_position, 
            target_orientation=target_orientation, 
            seed_joints=current_joints,
            arm_only=True
        )
        
        if solution is not None:
            # Check final accuracy
            final_config = ik.get_full_configuration(solution)
            final_fk = ik.forward_kinematics(final_config)
            
            pos_error_final = np.linalg.norm(np.array(target_position) - np.array(final_fk['position']))
            
            # Compute orientation error
            final_quat = pin.Quaternion(np.array(final_fk['orientation']))
            target_quat = pin.Quaternion(target_orientation)
            ori_error_quat = target_quat * final_quat.inverse()
            ori_error_final = np.linalg.norm(pin.log3(ori_error_quat.matrix()))
            
            print(f"FINAL RESULT - Weight {weight}:")
            print(f"  Final Position Error: {pos_error_final:.4f} m")
            print(f"  Final Orientation Error: {ori_error_final:.4f} rad ({ori_error_final * 180/np.pi:.1f} deg)")
            print(f"  Joint Solution: {solution}")
        else:
            print(f"❌ FAILED - Weight {weight}: No solution found")

if __name__ == '__main__':
    test_orientation_weights() 