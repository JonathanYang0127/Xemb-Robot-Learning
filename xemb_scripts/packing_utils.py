from evdev import UInput, ecodes as e

def scale_value(value, min_val=-3, max_val=3):
    """Scale a value to signed 16-bit integer"""
    normalized = (value - min_val) / (max_val - min_val)
    return int(normalized * 65535) - 32768

def unscale_value(value, min_val=-3, max_val=3):
    """Convert signed 16-bit integer back to original range"""
    normalized = (value + 32768) / 65535
    return normalized * (max_val - min_val) + min_val

def pack_values_into_axes(joint_values, gripper_value, heartbeat):
    """Pack joint and gripper values into axis state values alternating based on heartbeat.
    
    Args:
        joint_values: List of 6 joint values in range [-3, 3]
        gripper_value: Gripper value in range [-3, 3]
        heartbeat: Integer that determines which set of values to send
        
    Returns:
        Dict mapping axis codes to values
    """
    if heartbeat % 2 == 0:
        # Pack joints 1-3
        return {
            e.ABS_X: scale_value(joint_values[0]),  # Joint 1
            e.ABS_Y: scale_value(joint_values[1]),  # Joint 2
            e.ABS_RX: scale_value(joint_values[2]), # Joint 3
            e.ABS_RY: 30000, 
            e.ABS_Z: heartbeat * 100  # Heartbeat counter
        }
    else:
        # Pack joints 4-6 and gripper
        return {
            e.ABS_X: scale_value(joint_values[3]),  # Joint 4
            e.ABS_Y: scale_value(joint_values[4]),  # Joint 5
            e.ABS_RX: scale_value(joint_values[5]), # Joint 6
            e.ABS_RY: scale_value(gripper_value),   # Gripper
            e.ABS_Z: heartbeat * 100  # Heartbeat counter
        }

def unpack_values_from_axes(axis_values):
    """Unpack joint and gripper values from axis state values.
    
    Args:
        axis_values: Dict mapping axis codes to values
        
    Returns:
        tuple (joint_values, gripper_value) where joint_values is incomplete
        based on heartbeat
    """
    heartbeat = int(axis_values[e.ABS_Z] >= 55) # Get heartbeat from HAT0X value
    
    if heartbeat % 2 == 0:
        # Unpack joints 1-3
        return [
            unscale_value(axis_values[e.ABS_X]),  # Joint 1
            unscale_value(axis_values[e.ABS_Y]),  # Joint 2
            unscale_value(axis_values[e.ABS_RX]), # Joint 3
            None, None, None  # Joints 4-6 not in this packet
        ], None
    else:
        # Unpack joints 4-6 and gripper
        return [
            None, None, None,  # Joints 1-3 not in this packet
            unscale_value(axis_values[e.ABS_X]),  # Joint 4
            unscale_value(axis_values[e.ABS_Y]),  # Joint 5
            unscale_value(axis_values[e.ABS_RX])  # Joint 6
        ], unscale_value(axis_values[e.ABS_RY])

def test_packing():
    """Test function to verify packing/unpacking"""
    test_joints = [1.0, -1.0, 0.5, -0.5, 0.25, -0.25]
    test_gripper = 0.75
    
    print("\nTesting heartbeat 0 (joints 1-3):")
    packed_0 = pack_values_into_axes(test_joints, test_gripper, 0)
    joints_0, gripper_0 = unpack_values_from_axes(packed_0)
    print(f"Packed: {packed_0}")
    print(f"Unpacked joints: {joints_0}")
    print(f"Unpacked gripper: {gripper_0}")
    
    print("\nTesting heartbeat 1 (joints 4-6 + gripper):")
    packed_1 = pack_values_into_axes(test_joints, test_gripper, 1)
    joints_1, gripper_1 = unpack_values_from_axes(packed_1)
    print(f"Packed: {packed_1}")
    print(f"Unpacked joints: {joints_1}")
    print(f"Unpacked gripper: {gripper_1}")

if __name__ == "__main__":
    test_packing()
