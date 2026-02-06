#!/usr/bin/env python3
"""
VR Controller Data Packing Utilities
Handles packing/unpacking of Cartesian poses and button states for UDP transmission
"""

import numpy as np

# Button definitions for bitmask
BUTTON_MAP = {
    'trigger': 0,      # Trigger pressed (boolean)
    'a_button': 1,     # A button pressed
    'b_button': 2,     # B button pressed  
    'squeeze': 3,      # Squeeze/grip button pressed
    'menu': 4,         # Menu button pressed
    'thumbstick': 5,   # Thumbstick clicked
    'x_button': 6,     # X button pressed
    'y_button': 7      # Y button pressed
}

# Bitmask constants
BIT_TRIGGER = 1 << BUTTON_MAP['trigger']
BIT_A_BUTTON = 1 << BUTTON_MAP['a_button']
BIT_B_BUTTON = 1 << BUTTON_MAP['b_button']
BIT_SQUEEZE = 1 << BUTTON_MAP['squeeze']
BIT_MENU = 1 << BUTTON_MAP['menu']
BIT_THUMBSTICK = 1 << BUTTON_MAP['thumbstick']
BIT_X_BUTTON = 1 << BUTTON_MAP['x_button']
BIT_Y_BUTTON = 1 << BUTTON_MAP['y_button']

def scale_position(value, min_val=-2.0, max_val=2.0):
    """Scale a position value to signed 16-bit integer for network transmission"""
    normalized = (value - min_val) / (max_val - min_val)
    # Clamp to valid range to prevent overflow
    normalized = np.clip(normalized, 0, 1)
    return int(normalized * 65535) - 32768

def unscale_position(value, min_val=-2.0, max_val=2.0):
    """Convert signed 16-bit integer back to position value"""
    normalized = (value + 32768) / 65535
    return normalized * (max_val - min_val) + min_val

def scale_orientation(value, min_val=-1.0, max_val=1.0):
    """Scale a quaternion component to signed 16-bit integer"""
    normalized = (value - min_val) / (max_val - min_val)
    # Clamp to valid range
    normalized = np.clip(normalized, 0, 1)
    return int(normalized * 65535) - 32768

def unscale_orientation(value, min_val=-1.0, max_val=1.0):
    """Convert signed 16-bit integer back to quaternion component"""
    normalized = (value + 32768) / 65535
    return normalized * (max_val - min_val) + min_val

def scale_trigger(value):
    """Scale trigger value (0.0 to 1.0) to unsigned 16-bit integer"""
    normalized = np.clip(value, 0.0, 1.0)
    return int(normalized * 65535)

def unscale_trigger(value):
    """Convert unsigned 16-bit integer back to trigger value (0.0 to 1.0)"""
    return value / 65535.0

def pack_vr_data_for_udp(position, orientation, button_states, trigger_value, session_id=None, robot_id=0, joystick=None):
    """
    Pack VR controller data into a dictionary for UDP transmission.
    
    Args:
        position: [x, y, z] position in meters
        orientation: [w, x, y, z] quaternion orientation 
        button_states: Dictionary of button states (e.g., {'a_button': True, 'squeeze': False})
        trigger_value: Float from 0.0 to 1.0 representing trigger pressure
        session_id: Session identifier string
        robot_id: Integer robot ID for multi-robot setups
        joystick: [x, y] joystick/thumbstick axis values (-1.0 to 1.0)
    
    Returns:
        Dictionary ready for UDP transmission
    """
    
    # Create base packet
    data_packet = {
        'mode': 'vr_teleop',
        'robot_id': robot_id
    }
    
    if session_id is not None:
        data_packet['session_id'] = session_id
    
    # Pack position (x, y, z)
    if position is not None and len(position) >= 3:
        data_packet.update({
            'pos_x': scale_position(position[0]),
            'pos_y': scale_position(position[1]), 
            'pos_z': scale_position(position[2])
        })
    
    # Pack orientation (quaternion w, x, y, z)
    if orientation is not None and len(orientation) >= 4:
        data_packet.update({
            'quat_w': scale_orientation(orientation[0]),
            'quat_x': scale_orientation(orientation[1]),
            'quat_y': scale_orientation(orientation[2]),
            'quat_z': scale_orientation(orientation[3])
        })
    
    # Pack trigger value
    if trigger_value is not None:
        data_packet['trigger'] = scale_trigger(trigger_value)
    
    # Pack button states into bitmask
    button_mask = 0
    if button_states:
        if button_states.get('trigger', False): 
            button_mask |= BIT_TRIGGER
        if button_states.get('a_button', False): 
            button_mask |= BIT_A_BUTTON
        if button_states.get('b_button', False): 
            button_mask |= BIT_B_BUTTON
        if button_states.get('squeeze', False): 
            button_mask |= BIT_SQUEEZE
        if button_states.get('menu', False): 
            button_mask |= BIT_MENU
        if button_states.get('thumbstick', False): 
            button_mask |= BIT_THUMBSTICK
        if button_states.get('x_button', False): 
            button_mask |= BIT_X_BUTTON
        if button_states.get('y_button', False): 
            button_mask |= BIT_Y_BUTTON
    
    data_packet['button_mask'] = button_mask
    
    # Pack joystick/thumbstick axis data
    if joystick is not None and len(joystick) >= 2:
        data_packet.update({
            'joystick_x': scale_position(joystick[0]),  # Reuse position scaling for -1 to 1 range
            'joystick_y': scale_position(joystick[1])   # Reuse position scaling for -1 to 1 range
        })
    else:
        # Default to centered joystick
        data_packet.update({
            'joystick_x': scale_position(0.0),
            'joystick_y': scale_position(0.0)
        })
    
    return data_packet

def unpack_vr_data_from_udp(data_packet):
    """
    Unpack VR controller data from a received UDP packet.
    
    Args:
        data_packet: Dictionary received over UDP
        
    Returns:
        Tuple: (position, orientation, button_states, trigger_value, session_id, robot_id, joystick)
        - position: [x, y, z] or None
        - orientation: [w, x, y, z] quaternion or None  
        - button_states: Dictionary of button states
        - trigger_value: Float 0.0-1.0 or None
        - session_id: String or None
        - robot_id: Integer or 0
        - joystick: [x, y] joystick/thumbstick axis values (-1.0 to 1.0) or None
    """
    
    # Extract basic metadata
    session_id = data_packet.get('session_id')
    robot_id = data_packet.get('robot_id', 0)
    
    # Unpack position
    position = None
    if all(key in data_packet for key in ['pos_x', 'pos_y', 'pos_z']):
        position = [
            unscale_position(data_packet['pos_x']),
            unscale_position(data_packet['pos_y']),
            unscale_position(data_packet['pos_z'])
        ]
    
    # Unpack orientation  
    orientation = None
    if all(key in data_packet for key in ['quat_w', 'quat_x', 'quat_y', 'quat_z']):
        orientation = [
            unscale_orientation(data_packet['quat_w']),
            unscale_orientation(data_packet['quat_x']),
            unscale_orientation(data_packet['quat_y']),
            unscale_orientation(data_packet['quat_z'])
        ]
    
    # Unpack trigger value
    trigger_value = None
    if 'trigger' in data_packet:
        trigger_value = unscale_trigger(data_packet['trigger'])
    
    # Unpack button states from bitmask
    button_mask = data_packet.get('button_mask', 0)
    button_states = {
        'trigger': bool(button_mask & BIT_TRIGGER),
        'a_button': bool(button_mask & BIT_A_BUTTON),
        'b_button': bool(button_mask & BIT_B_BUTTON),
        'squeeze': bool(button_mask & BIT_SQUEEZE),
        'menu': bool(button_mask & BIT_MENU),
        'thumbstick': bool(button_mask & BIT_THUMBSTICK),
        'x_button': bool(button_mask & BIT_X_BUTTON),
        'y_button': bool(button_mask & BIT_Y_BUTTON)
    }
    
    # Unpack joystick/thumbstick axis data
    joystick = None
    if 'joystick_x' in data_packet and 'joystick_y' in data_packet:
        joystick = [
            unscale_position(data_packet['joystick_x']),
            unscale_position(data_packet['joystick_y'])
        ]
    
    return position, orientation, button_states, trigger_value, session_id, robot_id, joystick

def create_vr_init_packet(session_id, robot_id=0, initial_position=None, initial_orientation=None):
    """
    Create initialization packet for VR teleoperation.
    
    Args:
        session_id: Session identifier string
        robot_id: Integer robot ID
        initial_position: [x, y, z] initial position or None
        initial_orientation: [w, x, y, z] initial quaternion or None
        
    Returns:
        Dictionary initialization packet
    """
    init_packet = {
        'mode': 'vr_init',
        'init_session_id': session_id,
        'robot_id': robot_id
    }
    
    if initial_position is not None:
        init_packet.update({
            'init_pos_x': scale_position(initial_position[0]),
            'init_pos_y': scale_position(initial_position[1]),
            'init_pos_z': scale_position(initial_position[2])
        })
    
    if initial_orientation is not None:
        init_packet.update({
            'init_quat_w': scale_orientation(initial_orientation[0]),
            'init_quat_x': scale_orientation(initial_orientation[1]),
            'init_quat_y': scale_orientation(initial_orientation[2]),
            'init_quat_z': scale_orientation(initial_orientation[3])
        })
    
    return init_packet

def test_vr_packing():
    """Test function to verify VR data packing/unpacking"""
    print("Testing VR data packing/unpacking...")
    
    # Test data
    test_position = [0.5, -0.3, 0.8]
    test_orientation = [0.707, 0.0, 0.707, 0.0]  # 90 degree rotation about Z
    test_button_states = {
        'trigger': True,
        'a_button': False,
        'b_button': True,
        'squeeze': False,
        'menu': True,
        'x_button': False,
        'y_button': True
    }
    test_trigger_value = 0.75
    test_session_id = "test_session_123"
    test_robot_id = 1
    
    print("Original data:")
    print(f"  Position: {test_position}")
    print(f"  Orientation: {test_orientation}")
    print(f"  Button states: {test_button_states}")
    print(f"  Trigger value: {test_trigger_value}")
    print(f"  Session ID: {test_session_id}")
    print(f"  Robot ID: {test_robot_id}")
    
    # Pack data
    packed = pack_vr_data_for_udp(
        test_position, test_orientation, test_button_states, 
        test_trigger_value, test_session_id, test_robot_id
    )
    print(f"\nPacked data: {packed}")
    
    # Unpack data
    pos, ori, buttons, trigger, session, robot = unpack_vr_data_from_udp(packed)
    
    print("\nUnpacked data:")
    print(f"  Position: {pos}")
    print(f"  Orientation: {ori}")
    print(f"  Button states: {buttons}")
    print(f"  Trigger value: {trigger}")
    print(f"  Session ID: {session}")
    print(f"  Robot ID: {robot}")
    
    # Check accuracy
    pos_error = np.linalg.norm(np.array(pos) - np.array(test_position)) if pos else float('inf')
    ori_error = np.linalg.norm(np.array(ori) - np.array(test_orientation)) if ori else float('inf')
    trigger_error = abs(trigger - test_trigger_value) if trigger is not None else float('inf')
    
    print(f"\nAccuracy:")
    print(f"  Position error: {pos_error:.6f}")
    print(f"  Orientation error: {ori_error:.6f}")
    print(f"  Trigger error: {trigger_error:.6f}")
    
    # Test init packet
    print(f"\nTesting init packet...")
    init_packet = create_vr_init_packet(test_session_id, test_robot_id, test_position, test_orientation)
    print(f"Init packet: {init_packet}")
    
    print("✅ VR packing test complete!")

if __name__ == '__main__':
    test_vr_packing() 