#!/usr/bin/env python3

import numpy as np
import time
from piper_pinocchio_ik import PiperIK

def debug_ik_failure():
    """Debug why IK is failing on simple targets."""
    print("🔍 Debugging IK Failures...")
    print("=" * 50)
    
    # Create IK solver
    ik = PiperIK(max_iterations=100, tolerance=1e-3, damping=1e-3)
    
    # Test with a known good configuration
    test_joints = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0])
    test_joints_full = ik.get_full_configuration(test_joints)
    
    print(f"Test joints: {test_joints}")
    print(f"Joint limits: {ik.joint_limits_lower[:6]} to {ik.joint_limits_upper[:6]}")
    
    # Get FK position
    fk_result = ik.forward_kinematics(test_joints_full)
    if not fk_result:
        print("❌ FK failed!")
        return
    
    base_pos = fk_result['position']
    print(f"Base position: {base_pos}")
    
    # Test very small movements
    test_deltas = [0.001, 0.005, 0.01, 0.02, 0.05]  # 1mm to 5cm
    
    for delta in test_deltas:
        print(f"\n🎯 Testing {delta*1000:.1f}mm movement in X:")
        
        target_pos = [base_pos[0] + delta, base_pos[1], base_pos[2]]
        print(f"  Target: {target_pos}")
        
        # Test if target is reachable by checking if any valid config can reach it
        workspace = ik.get_workspace_limits(500)
        if workspace:
            x_range = workspace['x']
            y_range = workspace['y'] 
            z_range = workspace['z']
            
            in_workspace = (x_range[0] <= target_pos[0] <= x_range[1] and
                          y_range[0] <= target_pos[1] <= y_range[1] and
                          z_range[0] <= target_pos[2] <= z_range[1])
            
            print(f"  In workspace bounds: {in_workspace}")
            
            if not in_workspace:
                print(f"  ❌ Target outside workspace!")
                print(f"     X: {target_pos[0]:.3f} (range: {x_range[0]:.3f} to {x_range[1]:.3f})")
                print(f"     Y: {target_pos[1]:.3f} (range: {y_range[0]:.3f} to {y_range[1]:.3f})")
                print(f"     Z: {target_pos[2]:.3f} (range: {z_range[0]:.3f} to {z_range[1]:.3f})")
                continue
        
        # Try IK with debugging
        start_time = time.time()
        solution = debug_single_ik(ik, target_pos, test_joints, workspace)
        solve_time = time.time() - start_time
        
        if solution is not None:
            print(f"  ✅ Solved in {solve_time*1000:.2f}ms")
            
            # Verify solution
            verify_fk = ik.forward_kinematics(ik.get_full_configuration(solution))
            if verify_fk:
                actual_pos = verify_fk['position']
                error = np.linalg.norm(np.array(target_pos) - np.array(actual_pos))
                print(f"  Position error: {error*1000:.2f}mm")
                print(f"  Joint solution: {solution}")
            else:
                print(f"  ❌ Verification failed!")
        else:
            print(f"  ❌ Failed after {solve_time*1000:.2f}ms")

def debug_single_ik(ik, target_pos, seed_joints, workspace):
    """Debug a single IK solve with detailed output."""
    target_position = np.asarray(target_pos)
    
    # Try the smart seed first
    if workspace:
        smart_seed = ik.generate_good_seed(target_position, workspace)
        if smart_seed is not None:
            print(f"    Trying smart seed...")
            result = debug_ik_with_seed(ik, target_position, smart_seed)
            if result is not None:
                return result
    
    # Try user seed
    if seed_joints is not None:
        print(f"    Trying user seed...")
        seed_full = ik.get_full_configuration(seed_joints)
        result = debug_ik_with_seed(ik, target_position, seed_full)
        if result is not None:
            return result
    
    # Try neutral seed
    print(f"    Trying neutral seed...")
    neutral_seed = ik.get_full_configuration(np.zeros(6))
    result = debug_ik_with_seed(ik, target_position, neutral_seed)
    if result is not None:
        return result
    
    return None

def debug_ik_with_seed(ik, target_position, q_seed):
    """Debug IK with a specific seed."""
    import pinocchio as pin
    
    q = q_seed.copy()
    target_pose = pin.SE3.Identity()
    target_pose.translation = target_position
    
    print(f"      Seed: {q[:6]}")
    
    # Check initial error
    pin.framesForwardKinematics(ik.model, ik.data, q)
    current_pose = ik.data.oMf[ik.ee_frame_id]
    initial_error = np.linalg.norm(target_position - current_pose.translation)
    print(f"      Initial error: {initial_error*1000:.2f}mm")
    
    if initial_error < 0.01:  # Already close enough
        return q[:6]
    
    # Try a few iterations with detailed output
    for i in range(10):  # Just show first 10 iterations
        pin.framesForwardKinematics(ik.model, ik.data, q)
        current_pose = ik.data.oMf[ik.ee_frame_id]
        
        error_pose = target_pose.inverse() * current_pose
        error = pin.log(error_pose).vector[:3]  # Position only
        error_norm = np.linalg.norm(error)
        
        if error_norm < ik.tolerance:
            print(f"      ✅ Converged at iteration {i+1}")
            return q[:6]
        
        # Compute Jacobian
        J_full = pin.computeFrameJacobian(ik.model, ik.data, q, ik.ee_frame_id, pin.LOCAL_WORLD_ALIGNED)
        J = J_full[:3, :6]  # Position part, arm joints only
        
        # Check manipulability
        manipulability = np.sqrt(np.linalg.det(J @ J.T))
        
        if i < 5:  # Show detail for first few iterations
            print(f"      Iter {i+1}: error={error_norm*1000:.2f}mm, manip={manipulability:.6f}")
        
        if manipulability < 1e-6:
            print(f"      ❌ Singular at iteration {i+1}")
            break
        
        # Compute update
        damping = 0.01 if manipulability > 1e-4 else 0.1
        JJt = J @ J.T + damping * np.eye(3)
        
        try:
            dq = J.T @ np.linalg.solve(JJt, error)
        except:
            print(f"      ❌ Matrix inversion failed at iteration {i+1}")
            break
        
        # Adaptive step size
        step_size = min(0.1, 0.05 / max(error_norm, 1e-6))
        q_new = q[:6] + step_size * dq
        
        # Check joint limits
        q_new = np.clip(q_new, ik.joint_limits_lower[:6], ik.joint_limits_upper[:6])
        
        # Check if we made progress
        if np.linalg.norm(q_new - q[:6]) < 1e-6:
            print(f"      ❌ No progress at iteration {i+1}")
            break
        
        q[:6] = q_new
    
    print(f"      ❌ Failed to converge")
    return None

if __name__ == '__main__':
    debug_ik_failure() 
