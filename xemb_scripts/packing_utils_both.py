from evdev import UInput, ecodes as e
import numpy as np

def scale_value(value, min_val=-3, max_val=3):
    """Scale a value to signed 16-bit integer"""
    normalized = (value - min_val) / (max_val - min_val)
    return int(normalized * 65535) - 32768

def unscale_value(value, min_val=-3, max_val=3):
    """Convert signed 16-bit integer back to original range"""
    normalized = (value + 32768) / 65535
    return normalized * (max_val - min_val) + min_val

def pack_values_into_axes(joint_values_left, joint_values_right, gripper_value_left, gripper_value_right, heartbeat):
    """Pack joint and gripper values into axis state values alternating based on heartbeat.
    
    Args:
        joint_values_left: List of 6 joint values for left arm in range [-3, 3]
        joint_values_right: List of 6 joint values for right arm in range [-3, 3]
        gripper_value_left: Left gripper value in range [-3, 3]
        gripper_value_right: Right gripper value in range [-3, 3]
        heartbeat: Integer that determines which set of values to send (0-3)
        
    Returns:
        Dict mapping axis codes to values
    """
    # Common heartbeat value
    packed = {e.ABS_Z: heartbeat * 50}  # Using smaller steps for more heartbeats
    
    if heartbeat == 0:
        # Left arm: joints 1-3
        packed.update({
            e.ABS_X: scale_value(joint_values_left[0]),    # Joint 1
            e.ABS_Y: scale_value(joint_values_left[1]),    # Joint 2
            e.ABS_RX: scale_value(joint_values_left[2]),   # Joint 3
            e.ABS_RY: 30000  # Placeholder
        })
    elif heartbeat == 1:
        # Left arm: joints 4-6 + gripper
        packed.update({
            e.ABS_X: scale_value(joint_values_left[3]),    # Joint 4
            e.ABS_Y: scale_value(joint_values_left[4]),    # Joint 5
            e.ABS_RX: scale_value(joint_values_left[5]),   # Joint 6
            e.ABS_RY: scale_value(gripper_value_left)      # Left gripper
        })
    elif heartbeat == 2:
        # Right arm: joints 1-3
        packed.update({
            e.ABS_X: scale_value(joint_values_right[0]),   # Joint 1
            e.ABS_Y: scale_value(joint_values_right[1]),   # Joint 2
            e.ABS_RX: scale_value(joint_values_right[2]),  # Joint 3
            e.ABS_RY: 30000  # Placeholder
        })
    else:  # heartbeat == 3
        # Right arm: joints 4-6 + gripper
        packed.update({
            e.ABS_X: scale_value(joint_values_right[3]),   # Joint 4
            e.ABS_Y: scale_value(joint_values_right[4]),   # Joint 5
            e.ABS_RX: scale_value(joint_values_right[5]),  # Joint 6
            e.ABS_RY: scale_value(gripper_value_right)     # Right gripper
        })
    
    return packed

def get_heartbeat(axis_value, max_heartbeat=4):
    heartbeat = -1
    dist = 10000
    
    for i in range(0, max_heartbeat):
        axis_dist = np.abs(i * 50 - axis_value) 
        if axis_dist < dist:
            dist = axis_dist
            heartbeat = i
    return heartbeat


def unpack_values_from_axes(axis_values):
    """Unpack joint and gripper values from axis state values.
    
    Args:
        axis_values: Dict mapping axis codes to values
        
    Returns:
        tuple (joint_values_left, joint_values_right, gripper_value_left, gripper_value_right)
        where joint values are incomplete based on heartbeat
    """
    if e.ABS_Z not in axis_values:
        return None, None, None, None
    
    # Determine heartbeat (0-3)
    heartbeat = get_heartbeat(axis_values[e.ABS_Z])
    
    # Initialize empty joint arrays
    joints_left = [None, None, None, None, None, None]
    joints_right = [None, None, None, None, None, None]
    gripper_left = None
    gripper_right = None
    
    # Required axes should exist
    required_axes = [e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_Z]
    if heartbeat in [1, 3]:  # Heartbeats with gripper info need ABS_RY
        required_axes.append(e.ABS_RY)
    
    if not all(axis in axis_values for axis in required_axes):
        return None, None, None, None
    
    if heartbeat == 0:
        # Left arm: joints 1-3
        joints_left[0] = unscale_value(axis_values[e.ABS_X])
        joints_left[1] = unscale_value(axis_values[e.ABS_Y])
        joints_left[2] = unscale_value(axis_values[e.ABS_RX])
    elif heartbeat == 1:
        # Left arm: joints 4-6 + gripper
        joints_left[3] = unscale_value(axis_values[e.ABS_X])
        joints_left[4] = unscale_value(axis_values[e.ABS_Y])
        joints_left[5] = unscale_value(axis_values[e.ABS_RX])
        gripper_left = unscale_value(axis_values[e.ABS_RY])
    elif heartbeat == 2:
        # Right arm: joints 1-3
        joints_right[0] = unscale_value(axis_values[e.ABS_X])
        joints_right[1] = unscale_value(axis_values[e.ABS_Y])
        joints_right[2] = unscale_value(axis_values[e.ABS_RX])
    elif heartbeat == 3:
        # Right arm: joints 4-6 + gripper
        joints_right[3] = unscale_value(axis_values[e.ABS_X])
        joints_right[4] = unscale_value(axis_values[e.ABS_Y])
        joints_right[5] = unscale_value(axis_values[e.ABS_RX])
        gripper_right = unscale_value(axis_values[e.ABS_RY])
    
    return joints_left, joints_right, gripper_left, gripper_right

def test_packing():
    """Test function to verify packing/unpacking"""
    # Original test values
    test_joints_left = [1.0, -1.0, 0.5, -0.5, 0.25, -0.25]
    test_joints_right = [0.8, -0.8, 0.4, -0.4, 0.2, -0.2]
    test_gripper_left = 0.75
    test_gripper_right = 0.65
    
    # Print original values for reference
    print("Original values:")
    print(f"Left arm joints: {test_joints_left}")
    print(f"Right arm joints: {test_joints_right}")
    print(f"Left gripper: {test_gripper_left}")
    print(f"Right gripper: {test_gripper_right}")
    print("\n" + "="*50 + "\n")
    
    for hb in range(4):
        print(f"\nTesting heartbeat {hb}:")
        
        # Show which values should be encoded in this heartbeat
        if hb == 0:
            print(f"This heartbeat encodes LEFT joints 1-3: {test_joints_left[:3]}")
        elif hb == 1:
            print(f"This heartbeat encodes LEFT joints 4-6: {test_joints_left[3:]} and LEFT gripper: {test_gripper_left}")
        elif hb == 2:
            print(f"This heartbeat encodes RIGHT joints 1-3: {test_joints_right[:3]}")
        elif hb == 3:
            print(f"This heartbeat encodes RIGHT joints 4-6: {test_joints_right[3:]} and RIGHT gripper: {test_gripper_right}")
        
        # Pack values
        packed = pack_values_into_axes(
            test_joints_left, test_joints_right, 
            test_gripper_left, test_gripper_right, 
            hb
        )
        
        # Unpack values
        joints_left, joints_right, gripper_left, gripper_right = unpack_values_from_axes(packed)
        
        # Print encoded and decoded values
        print(f"Packed values: {packed}")
        print(f"Unpacked left joints: {joints_left}")
        print(f"Unpacked right joints: {joints_right}")
        print(f"Unpacked left gripper: {gripper_left}")
        print(f"Unpacked right gripper: {gripper_right}")
        
        # Compare with original
        print("\nComparison with original values:")
        if hb == 0:
            # Left joints 1-3
            for i in range(3):
                if joints_left[i] is not None:
                    err = abs(joints_left[i] - test_joints_left[i])
                    print(f"  Left joint {i+1}: Original={test_joints_left[i]:.6f}, Unpacked={joints_left[i]:.6f}, Error={err:.6f}")
        elif hb == 1:
            # Left joints 4-6 and gripper
            for i in range(3, 6):
                if joints_left[i] is not None:
                    err = abs(joints_left[i] - test_joints_left[i])
                    print(f"  Left joint {i+1}: Original={test_joints_left[i]:.6f}, Unpacked={joints_left[i]:.6f}, Error={err:.6f}")
            if gripper_left is not None:
                err = abs(gripper_left - test_gripper_left)
                print(f"  Left gripper: Original={test_gripper_left:.6f}, Unpacked={gripper_left:.6f}, Error={err:.6f}")
        elif hb == 2:
            # Right joints 1-3
            for i in range(3):
                if joints_right[i] is not None:
                    err = abs(joints_right[i] - test_joints_right[i])
                    print(f"  Right joint {i+1}: Original={test_joints_right[i]:.6f}, Unpacked={joints_right[i]:.6f}, Error={err:.6f}")
        elif hb == 3:
            # Right joints 4-6 and gripper
            for i in range(3, 6):
                if joints_right[i] is not None:
                    err = abs(joints_right[i] - test_joints_right[i])
                    print(f"  Right joint {i+1}: Original={test_joints_right[i]:.6f}, Unpacked={joints_right[i]:.6f}, Error={err:.6f}")
            if gripper_right is not None:
                err = abs(gripper_right - test_gripper_right)
                print(f"  Right gripper: Original={test_gripper_right:.6f}, Unpacked={gripper_right:.6f}, Error={err:.6f}")

if __name__ == "__main__":
    test_packing()

