import numpy as np

# Key definitions for bitmask (consistent with pynput)
KEY_MAP = {
    'w': 0, 'a': 1, 's': 2, 'd': 3,
    'up': 4, 'left': 5, 'down': 6, 'right': 7
}
BIT_W = 1 << KEY_MAP['w']
BIT_A = 1 << KEY_MAP['a']
BIT_S = 1 << KEY_MAP['s']
BIT_D = 1 << KEY_MAP['d']
BIT_UP = 1 << KEY_MAP['up']
BIT_LEFT = 1 << KEY_MAP['left']
BIT_DOWN = 1 << KEY_MAP['down']
BIT_RIGHT = 1 << KEY_MAP['right']

def scale_value(value, min_val=-3, max_val=3):
    """Scale a value to signed 16-bit integer"""
    normalized = (value - min_val) / (max_val - min_val)
    # Clamp normalized value to [0, 1] to prevent out-of-range errors
    normalized = max(0.0, min(1.0, normalized))
    return int(normalized * 65535) - 32768

def unscale_value(value, min_val=-3, max_val=3):
    """Convert signed 16-bit integer back to original range"""
    normalized = (value + 32768) / 65535
    return normalized * (max_val - min_val) + min_val

def pack_data_for_udp(joint_values_left, joint_values_right, gripper_value_left, gripper_value_right, heartbeat, key_states=None):
    """
    Pack robot and key states into a dictionary for UDP transmission.
    Data is chosen based on the heartbeat.

    Args:
        joint_values_left: List of 6 joint values for left arm.
        joint_values_right: List of 6 joint values for right arm.
        gripper_value_left: Left gripper value.
        gripper_value_right: Right gripper value.
        heartbeat: Integer (0-3) determining which data subset to send.
        key_states: Dict with keyboard key states (e.g., {'w': True, 's': False}).

    Returns:
        A dictionary containing the heartbeat and relevant scaled data.
    """
    data_packet = {'heartbeat': heartbeat}
    key_mask = 0
    if key_states:
        if key_states.get('w', False): key_mask |= BIT_W
        if key_states.get('a', False): key_mask |= BIT_A
        if key_states.get('s', False): key_mask |= BIT_S
        if key_states.get('d', False): key_mask |= BIT_D
        if key_states.get('up', False): key_mask |= BIT_UP
        if key_states.get('left', False): key_mask |= BIT_LEFT
        if key_states.get('down', False): key_mask |= BIT_DOWN
        if key_states.get('right', False): key_mask |= BIT_RIGHT

    # Always include key_mask, even if 0, for consistent packet structure.
    data_packet['key_mask'] = key_mask

    if heartbeat == 0: # Left arm joints 1-3
        data_packet.update({
            'j1l': scale_value(joint_values_left[0]),
            'j2l': scale_value(joint_values_left[1]),
            'j3l': scale_value(joint_values_left[2]),
        })
    elif heartbeat == 1: # Left arm joints 4-6 + gripper
        data_packet.update({
            'j4l': scale_value(joint_values_left[3]),
            'j5l': scale_value(joint_values_left[4]),
            'j6l': scale_value(joint_values_left[5]),
            'gl': scale_value(gripper_value_left)
        })
    elif heartbeat == 2: # Right arm joints 1-3
        data_packet.update({
            'j1r': scale_value(joint_values_right[0]),
            'j2r': scale_value(joint_values_right[1]),
            'j3r': scale_value(joint_values_right[2]),
        })
    elif heartbeat == 3: # Right arm joints 4-6 + gripper
        data_packet.update({
            'j4r': scale_value(joint_values_right[3]),
            'j5r': scale_value(joint_values_right[4]),
            'j6r': scale_value(joint_values_right[5]),
            'gr': scale_value(gripper_value_right)
        })
    else:
        # Should not happen with heartbeat % 4
        print(f"Warning: Unknown heartbeat value {heartbeat} in pack_data_for_udp")
        pass

    return data_packet

def unpack_data_from_udp(data_packet):
    """
    Unpack robot and key states from a received data dictionary.

    Args:
        data_packet: A dictionary received over UDP.

    Returns:
        A tuple: (joints_left, joints_right, gripper_left, gripper_right, key_states)
        Unused values in the tuple will be None or lists of Nones.
    """
    heartbeat = data_packet.get('heartbeat', -1)
    key_mask = data_packet.get('key_mask', 0)

    key_states = {
        'w': bool(key_mask & BIT_W), 'a': bool(key_mask & BIT_A),
        's': bool(key_mask & BIT_S), 'd': bool(key_mask & BIT_D),
        'up': bool(key_mask & BIT_UP), 'left': bool(key_mask & BIT_LEFT),
        'down': bool(key_mask & BIT_DOWN), 'right': bool(key_mask & BIT_RIGHT)
    }

    joints_left = [None] * 6
    joints_right = [None] * 6
    gripper_left = None
    gripper_right = None

    if heartbeat == 0:
        if 'j1l' in data_packet: joints_left[0] = unscale_value(data_packet['j1l'])
        if 'j2l' in data_packet: joints_left[1] = unscale_value(data_packet['j2l'])
        if 'j3l' in data_packet: joints_left[2] = unscale_value(data_packet['j3l'])
    elif heartbeat == 1:
        if 'j4l' in data_packet: joints_left[3] = unscale_value(data_packet['j4l'])
        if 'j5l' in data_packet: joints_left[4] = unscale_value(data_packet['j5l'])
        if 'j6l' in data_packet: joints_left[5] = unscale_value(data_packet['j6l'])
        if 'gl' in data_packet: gripper_left = unscale_value(data_packet['gl'])
    elif heartbeat == 2:
        if 'j1r' in data_packet: joints_right[0] = unscale_value(data_packet['j1r'])
        if 'j2r' in data_packet: joints_right[1] = unscale_value(data_packet['j2r'])
        if 'j3r' in data_packet: joints_right[2] = unscale_value(data_packet['j3r'])
    elif heartbeat == 3:
        if 'j4r' in data_packet: joints_right[3] = unscale_value(data_packet['j4r'])
        if 'j5r' in data_packet: joints_right[4] = unscale_value(data_packet['j5r'])
        if 'j6r' in data_packet: joints_right[5] = unscale_value(data_packet['j6r'])
        if 'gr' in data_packet: gripper_right = unscale_value(data_packet['gr'])
    else:
        print(f"Warning: Unknown or missing heartbeat in data_packet: {data_packet}")
        # Return all Nones if heartbeat is invalid, but still return key_states
        return [None]*6, [None]*6, None, None, key_states


    return joints_left, joints_right, gripper_left, gripper_right, key_states

def test_packing_direct():
    """Test function to verify direct packing/unpacking"""
    test_joints_left = [1.0, -1.0, 0.5, -0.5, 0.25, -0.25]
    test_joints_right = [0.8, -0.8, 0.4, -0.4, 0.2, -0.2]
    test_gripper_left = 0.75
    test_gripper_right = 0.65
    test_key_states = {
        'w': True, 'a': False, 's': True, 'd': False,
        'up': True, 'left': False, 'down': True, 'right': False
    }

    print("Original values:")
    print(f"  Left arm joints: {test_joints_left}")
    print(f"  Right arm joints: {test_joints_right}")
    print(f"  Left gripper: {test_gripper_left}")
    print(f"  Right gripper: {test_gripper_right}")
    print(f"  Key states: {test_key_states}")
    print("\\n" + "="*50 + "\\n")

    all_tests_passed = True

    for hb_idx in range(4):
        print(f"Testing heartbeat {hb_idx}:")
        packed_dict = pack_data_for_udp(
            test_joints_left, test_joints_right,
            test_gripper_left, test_gripper_right,
            hb_idx, test_key_states
        )
        print(f"  Packed data: {packed_dict}")

        un_jl, un_jr, un_gl, un_gr, un_ks = unpack_data_from_udp(packed_dict)

        print(f"  Unpacked Left Joints: {un_jl}")
        print(f"  Unpacked Right Joints: {un_jr}")
        print(f"  Unpacked Left Gripper: {un_gl}")
        print(f"  Unpacked Right Gripper: {un_gr}")
        print(f"  Unpacked Key States: {un_ks}")

        # Verify key states (should be decoded correctly in every heartbeat)
        if un_ks != test_key_states:
            all_tests_passed = False
            print(f"  ERROR: Key states mismatch! Original: {test_key_states}, Unpacked: {un_ks}")

        # Verify relevant joint/gripper data for the current heartbeat
        if hb_idx == 0: # Left arm joints 1-3
            for i in range(3):
                if abs(un_jl[i] - test_joints_left[i]) > 1e-4:
                    all_tests_passed = False
                    print(f"  ERROR: Left joint {i} mismatch! Original: {test_joints_left[i]}, Unpacked: {un_jl[i]}")
        elif hb_idx == 1: # Left arm joints 4-6 + gripper
            for i in range(3, 6):
                if abs(un_jl[i] - test_joints_left[i]) > 1e-4:
                    all_tests_passed = False
                    print(f"  ERROR: Left joint {i} mismatch! Original: {test_joints_left[i]}, Unpacked: {un_jl[i]}")
            if abs(un_gl - test_gripper_left) > 1e-4:
                 all_tests_passed = False
                 print(f"  ERROR: Left gripper mismatch! Original: {test_gripper_left}, Unpacked: {un_gl}")
        elif hb_idx == 2: # Right arm joints 1-3
            for i in range(3):
                if abs(un_jr[i] - test_joints_right[i]) > 1e-4:
                    all_tests_passed = False
                    print(f"  ERROR: Right joint {i} mismatch! Original: {test_joints_right[i]}, Unpacked: {un_jr[i]}")
        elif hb_idx == 3: # Right arm joints 4-6 + gripper
            for i in range(3, 6):
                if abs(un_jr[i] - test_joints_right[i]) > 1e-4:
                    all_tests_passed = False
                    print(f"  ERROR: Right joint {i} mismatch! Original: {test_joints_right[i]}, Unpacked: {un_jr[i]}")
            if abs(un_gr - test_gripper_right) > 1e-4:
                all_tests_passed = False
                print(f"  ERROR: Right gripper mismatch! Original: {test_gripper_right}, Unpacked: {un_gr}")
        print("-" * 30)

    if all_tests_passed:
        print("\\nAll direct packing tests passed!")
    else:
        print("\\nSome direct packing tests FAILED.")

if __name__ == "__main__":
    test_packing_direct() 