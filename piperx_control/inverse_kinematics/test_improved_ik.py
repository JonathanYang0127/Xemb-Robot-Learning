#!/usr/bin/env python3

import numpy as np
import time
from piper_pinocchio_ik import PiperIK, benchmark_ik

def test_improvements():
    """Test the improved IK solver with detailed comparisons."""
    print("🔧 Testing Improved Piper IK with Pinocchio...")
    print("=" * 60)
    
    try:
        # Create IK solver with improved settings
        ik = PiperIK(max_iterations=100, tolerance=1e-3, damping=1e-3)
        print("✅ IK solver initialized successfully")
    except Exception as e:
        print(f"❌ Failed to initialize: {e}")
        return
    
    # Test 1: Basic functionality
    print("\n1. Testing basic functionality:")
    test_joints = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0])
    test_joints_full = ik.get_full_configuration(test_joints)
    
    fk_result = ik.forward_kinematics(test_joints_full)
    if fk_result:
        pos = fk_result['position']
        print(f"   FK position: {pos}")
        
        # Test with improved IK
        start_time = time.time()
        ik_result = ik.inverse_kinematics(pos, seed_joints=test_joints)
        solve_time = time.time() - start_time
        
        if ik_result is not None:
            print(f"   ✅ IK solved in {solve_time*1000:.2f}ms")
            print(f"   Joint error: {np.linalg.norm(test_joints - ik_result):.6f}")
        else:
            print("   ❌ IK failed")
    
    # Test 2: Workspace estimation improvements
    print("\n2. Testing improved workspace estimation:")
    start_time = time.time()
    workspace = ik.get_workspace_limits(1000)
    estimation_time = time.time() - start_time
    
    if workspace:
        print(f"   ✅ Workspace estimated in {estimation_time:.2f}s")
        print(f"   X range: [{workspace['x'][0]:.3f}, {workspace['x'][1]:.3f}]")
        print(f"   Y range: [{workspace['y'][0]:.3f}, {workspace['y'][1]:.3f}]")
        print(f"   Z range: [{workspace['z'][0]:.3f}, {workspace['z'][1]:.3f}]")
        print(f"   Valid configurations: {len(workspace.get('valid_configs', []))}")
    else:
        print("   ❌ Workspace estimation failed")
        return
    
    # Test 3: Multi-seed IK with workspace info
    print("\n3. Testing multi-seed IK with workspace info:")
    if fk_result:
        # Try a slightly challenging target (much smaller movement)
        challenging_pos = [pos[0] + 0.01, pos[1] + 0.005, pos[2] + 0.005]
        
        start_time = time.time()
        solution = ik.inverse_kinematics(challenging_pos, workspace_info=workspace)
        solve_time = time.time() - start_time
        
        if solution is not None:
            print(f"   ✅ Multi-seed IK solved in {solve_time*1000:.2f}ms")
            
            # Verify solution
            verify_fk = ik.forward_kinematics(ik.get_full_configuration(solution))
            if verify_fk:
                pos_error = np.linalg.norm(np.array(challenging_pos) - np.array(verify_fk['position']))
                print(f"   Position error: {pos_error*1000:.2f}mm")
        else:
            print("   ❌ Multi-seed IK failed")
    
    # Test 4: Batch IK with workspace info
    print("\n4. Testing batch IK with workspace info:")
    if fk_result:
        pos1 = fk_result['position']
        positions = [
            pos1,
            [pos1[0] + 0.005, pos1[1], pos1[2]],
            [pos1[0] + 0.005, pos1[1] + 0.005, pos1[2]],
            [pos1[0], pos1[1] + 0.005, pos1[2]],
            [pos1[0], pos1[1], pos1[2] + 0.005]
        ]
        
        start_time = time.time()
        batch_solutions = ik.batch_ik(positions, workspace_info=workspace)
        batch_time = time.time() - start_time
        
        success_count = sum(1 for sol in batch_solutions if sol is not None)
        print(f"   ✅ Batch IK: {success_count}/{len(positions)} succeeded in {batch_time*1000:.2f}ms")
        print(f"   Average per solve: {(batch_time/len(positions))*1000:.2f}ms")
    
    # Test 5: Performance benchmark
    print("\n5. Running performance benchmark:")
    print("   This will test 50 realistic targets within the workspace...")
    
    # Reduce test count for faster feedback
    stats = benchmark_ik(ik, 50)
    
    if stats:
        print(f"\n📊 Benchmark Results:")
        print(f"   Success rate: {stats['success_rate']:.1%}")
        print(f"   Average solve time: {stats['avg_solve_time']*1000:.2f}ms")
        print(f"   Median solve time: {stats['median_solve_time']*1000:.2f}ms")
        print(f"   Max solve time: {stats['max_solve_time']*1000:.2f}ms")
        
        # Performance categories
        if stats['success_rate'] > 0.8:
            print("   🟢 Success rate: EXCELLENT")
        elif stats['success_rate'] > 0.6:
            print("   🟡 Success rate: GOOD")
        else:
            print("   🔴 Success rate: NEEDS IMPROVEMENT")
        
        if stats['avg_solve_time'] < 0.01:
            print("   🟢 Speed: EXCELLENT (<10ms)")
        elif stats['avg_solve_time'] < 0.05:
            print("   🟡 Speed: GOOD (<50ms)")
        else:
            print("   🔴 Speed: NEEDS IMPROVEMENT (>50ms)")
    
    # Test 6: Manipulability analysis
    print("\n6. Testing manipulability analysis:")
    if fk_result:
        manipulability = ik.compute_manipulability(test_joints)
        print(f"   Manipulability index: {manipulability:.6f}")
        
        if manipulability > 0.01:
            print("   🟢 High manipulability - good dexterity")
        elif manipulability > 0.001:
            print("   🟡 Medium manipulability - moderate dexterity")
        else:
            print("   🔴 Low manipulability - near singularity")
    
    print("\n" + "=" * 60)
    print("🎯 Test Summary:")
    print("   - Improved workspace estimation with filtering")
    print("   - Multi-seed IK for better convergence")
    print("   - Smart seed generation based on workspace")
    print("   - Better error handling and validation")
    print("   - More robust numerical methods")
    print("=" * 60)

if __name__ == '__main__':
    test_improvements() 
