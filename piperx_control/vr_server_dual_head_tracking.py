#!/usr/bin/env python3
"""
Dual Arm VR Server with Head Tracking - Handles both left and right VR controllers
Keeps everything in VR coordinates for plug-and-play compatibility
OPTIMIZED FOR LOW LATENCY with HEAD-RELATIVE CONTROL for DUAL ARMS
"""

import sys
import time
import numpy as np
import socket
import pickle
import threading
import argparse
from asyncio import sleep

# Add the parent directory to the path
sys.path.append('..')
from xemb_scripts.cartesian_control.transformations import rotation_matrix_to_quaternion

try:
    from vuer import Vuer, VuerSession
    from vuer.schemas import MotionControllers
except ImportError:
    print("Error: vuer not found. Please install with: pip install vuer")
    sys.exit(1)

# Default Configuration
DEFAULT_VR_PORT = 8012
DEFAULT_BROADCAST_PORT = 5006
DEFAULT_BROADCAST_ADDRESS = "127.0.0.1"
TRIGGER_THRESHOLD = 0.1

# Performance optimizations
BROADCAST_RATE = 60  # Optimal for VR
QUEUE_LEN = 1  # Minimal buffering for lowest latency
DEBUG_PRINTS = False

# Note: No coordinate transformation matrices here!
# We keep everything in VR coordinates and let the puppet handle robot-specific transformations.
# This ensures plug-and-play compatibility with any robot coordinate system.

def fast_mat_inv(mat):
    """Fast matrix inverse for 4x4 transformation matrices."""
    # For transformation matrices [R t; 0 1], inverse is [R^T -R^T*t; 0 1]
    R = mat[:3, :3]
    t = mat[:3, 3]
    inv_mat = np.eye(4)
    inv_mat[:3, :3] = R.T
    inv_mat[:3, 3] = -R.T @ t
    return inv_mat

def mat_update(old_mat, new_mat, alpha=0.1):
    """Smooth matrix update with exponential filtering."""
    return (1 - alpha) * old_mat + alpha * new_mat

# Global VR data with head tracking for DUAL ARMS
g_vr_data = {
    'left': {
        'position': np.zeros(3, dtype=np.float32),
        'orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        'head_relative_position': np.zeros(3, dtype=np.float32),
        'head_relative_orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        'trigger_value': 0.0,
        'joystick': np.zeros(2, dtype=np.float32),
        'button_states': {
            'trigger': False,
            'a_button': False,
            'b_button': False,
            'squeeze': False,
            'menu': False,
            'thumbstick': False,
            'x_button': False,
            'y_button': False
        },
        'connected': False,
        'last_update': time.time(),
        'update_count': 0
    },
    'right': {
        'position': np.zeros(3, dtype=np.float32),
        'orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        'head_relative_position': np.zeros(3, dtype=np.float32),
        'head_relative_orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        'trigger_value': 0.0,
        'joystick': np.zeros(2, dtype=np.float32),
        'button_states': {
            'trigger': False,
            'a_button': False,
            'b_button': False,
            'squeeze': False,
            'menu': False,
            'thumbstick': False,
            'x_button': False,
            'y_button': False
        },
        'connected': False,
        'last_update': time.time(),
        'update_count': 0
    }
}
g_lock = threading.RLock()

# Head tracking state for dual arms
g_head_tracking = {
    'head_matrix': np.eye(4, dtype=np.float32),
    'smoothed_head_matrix': np.eye(4, dtype=np.float32),
    'left_controller_matrix': np.eye(4, dtype=np.float32),
    'smoothed_left_controller_matrix': np.eye(4, dtype=np.float32),
    'right_controller_matrix': np.eye(4, dtype=np.float32),
    'smoothed_right_controller_matrix': np.eye(4, dtype=np.float32)
}
g_head_lock = threading.RLock()

# Global configuration
g_config = {
    'vr_port': DEFAULT_VR_PORT,
    'broadcast_port': DEFAULT_BROADCAST_PORT,
    'broadcast_address': DEFAULT_BROADCAST_ADDRESS
}

def compute_head_relative_pose(controller_matrix, head_matrix):
    """
    Compute controller pose relative to head orientation in VR coordinates.
    This keeps everything in VR space and lets the puppet handle robot coordinate transformation.
    """
    # Keep everything in VR space - no coordinate transformation here
    # Make controller position relative to head position
    relative_controller = controller_matrix.copy()
    relative_controller[:3, 3] = relative_controller[:3, 3] - head_matrix[:3, 3]
    
    # Extract position and orientation in VR space
    position = relative_controller[:3, 3]
    rotation_matrix = relative_controller[:3, :3]
    orientation = rotation_matrix_to_quaternion(rotation_matrix)
    
    return position, orientation

def broadcast_vr_data():
    """Broadcast head-relative VR data for BOTH controllers to puppet scripts."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
    
    print(f"📡 Starting DUAL-ARM HEAD-RELATIVE VR broadcast ({BROADCAST_RATE}Hz) to {g_config['broadcast_address']}:{g_config['broadcast_port']}...")
    packet_count = 0
    last_debug = time.time()
    
    # Pre-allocate combined packet structure
    combined_packet_template = {
        'mode': 'vr_dual_head_tracking',
        'timestamp': 0.0,
        'head_matrix': np.eye(4).tolist(),
        'left': None,
        'right': None
    }
    
    loop_period = 1.0 / BROADCAST_RATE
    
    while True:
        loop_start = time.time()
        
        should_send = False
        try:
            # Get current VR data and head tracking data
            with g_lock:
                current_timestamp = time.time()
                combined_packet_template['timestamp'] = current_timestamp
                
                left_connected = g_vr_data['left']['connected']
                right_connected = g_vr_data['right']['connected']
                
                # Process LEFT controller
                if left_connected:
                    combined_packet_template['left'] = {
                        'position': g_vr_data['left']['head_relative_position'].tolist(),
                        'orientation': g_vr_data['left']['head_relative_orientation'].tolist(),
                        'raw_position': g_vr_data['left']['position'].tolist(),
                        'raw_orientation': g_vr_data['left']['orientation'].tolist(),
                        'button_states': g_vr_data['left']['button_states'].copy(),
                        'trigger_value': g_vr_data['left']['trigger_value'],
                        'joystick': g_vr_data['left']['joystick'].tolist(),
                        'robot_id': 0,
                        'connected': True
                    }
                else:
                    combined_packet_template['left'] = None
                
                # Process RIGHT controller
                if right_connected:
                    combined_packet_template['right'] = {
                        'position': g_vr_data['right']['head_relative_position'].tolist(),
                        'orientation': g_vr_data['right']['head_relative_orientation'].tolist(),
                        'raw_position': g_vr_data['right']['position'].tolist(),
                        'raw_orientation': g_vr_data['right']['orientation'].tolist(),
                        'button_states': g_vr_data['right']['button_states'].copy(),
                        'trigger_value': g_vr_data['right']['trigger_value'],
                        'joystick': g_vr_data['right']['joystick'].tolist(),
                        'robot_id': 1,
                        'connected': True
                    }
                else:
                    combined_packet_template['right'] = None
            
            with g_head_lock:
                combined_packet_template['head_matrix'] = g_head_tracking['smoothed_head_matrix'].tolist()
            
            if left_connected or right_connected:
                should_send = True
            
            if should_send:
                # Serialize and send
                data = pickle.dumps(combined_packet_template.copy())
                sock.sendto(data, (g_config['broadcast_address'], g_config['broadcast_port']))
                packet_count += 1
            
            # Performance monitoring
            now = time.time()
            if now - last_debug > 10.0:
                elapsed = now - last_debug
                rate = packet_count / elapsed if elapsed > 0 else 0
                
                # Count active controllers
                with g_lock:
                    active_controllers = sum(1 for ctrl in ['left', 'right'] if g_vr_data[ctrl]['connected'])
                
                if rate < BROADCAST_RATE * 0.9:
                    print(f"⚠️ DUAL HEAD-RELATIVE VR: {rate:.0f}Hz (target: {BROADCAST_RATE}Hz, active: {active_controllers})")
                elif packet_count > 0:
                    print(f"📡 DUAL HEAD-RELATIVE VR: {rate:.0f}Hz OK (active controllers: {active_controllers})")
                
                packet_count = 0
                last_debug = now

            # Precise timing control
            elapsed = time.time() - loop_start
            sleep_time = max(0, loop_period - elapsed)
            
            if sleep_time > 0:
                time.sleep(sleep_time)
                
        except Exception as e:
            print(f"Broadcast error: {e}")
            time.sleep(0.001)

def setup_vr_server():
    """Setup VR server with head tracking capabilities for dual arms."""
    print("🎮 Starting DUAL-ARM HEAD-RELATIVE VR Server...")
    print("📋 Setup Instructions:")
    print(f"  1. Make sure ngrok is running: ngrok http {g_config['vr_port']}")
    print("  2. Access the ngrok URL from Meta Quest Browser")
    print("  3. Move both left AND right controllers to start data streaming")
    print("  4. 🧠 HEAD TRACKING: Turn your head/body - coordinates stay consistent for both arms!")
    print("  5. Left controller → Robot 0 (Left arm)")
    print("  6. Right controller → Robot 1 (Right arm)")
    
    app = Vuer(
        host='0.0.0.0', 
        port=g_config['vr_port'], 
        queries=dict(grid=False), 
        queue_len=QUEUE_LEN
    )
    
    @app.add_handler("CAMERA_MOVE")
    async def camera_handler(event, session):
        """Handle head/camera tracking for head-relative coordinate transformation."""
        global g_head_tracking
        
        try:
            camera_data = event.value.get("camera", {})
            if "matrix" in camera_data:
                raw_head_matrix = np.array(camera_data["matrix"], dtype=np.float32).reshape(4, 4, order='F')
                
                with g_head_lock:
                    # Smooth head matrix updates to reduce jitter
                    g_head_tracking['head_matrix'] = raw_head_matrix
                    g_head_tracking['smoothed_head_matrix'] = mat_update(
                        g_head_tracking['smoothed_head_matrix'], 
                        raw_head_matrix, 
                        alpha=0.3  # Faster smoothing for head tracking
                    )
                    
                    # Recompute head-relative controller poses for both arms
                    with g_lock:
                        if g_vr_data['left']['connected']:
                            position, orientation = compute_head_relative_pose(
                                g_head_tracking['smoothed_left_controller_matrix'],
                                g_head_tracking['smoothed_head_matrix']
                            )
                            g_vr_data['left']['head_relative_position'][:] = position
                            g_vr_data['left']['head_relative_orientation'][:] = orientation
                        
                        if g_vr_data['right']['connected']:
                            position, orientation = compute_head_relative_pose(
                                g_head_tracking['smoothed_right_controller_matrix'],
                                g_head_tracking['smoothed_head_matrix']
                            )
                            g_vr_data['right']['head_relative_position'][:] = position
                            g_vr_data['right']['head_relative_orientation'][:] = orientation
                    
        except Exception as e:
            if DEBUG_PRINTS:
                print(f"Head tracking error: {e}")

    @app.add_handler("CONTROLLER_MOVE")
    async def controller_handler(event, session):
        global g_vr_data, g_head_tracking
        
        data = event.value
        current_time = time.time()
        
        # Process LEFT controller
        if 'left' in data and isinstance(data['left'], (list, tuple)) and len(data['left']) == 16:
            try:
                # Get raw controller matrix
                raw_controller_matrix = np.array(data['left'], dtype=np.float32).reshape(4, 4, order='F')
                
                # Extract raw position and orientation for fallback
                position = raw_controller_matrix[:3, 3].copy()
                rotation_matrix = raw_controller_matrix[:3, :3]
                orientation = rotation_matrix_to_quaternion(rotation_matrix)
                
                with g_head_lock:
                    # Smooth left controller matrix
                    g_head_tracking['left_controller_matrix'] = raw_controller_matrix
                    g_head_tracking['smoothed_left_controller_matrix'] = mat_update(
                        g_head_tracking['smoothed_left_controller_matrix'],
                        raw_controller_matrix,
                        alpha=0.2  # Moderate smoothing for controller
                    )
                    
                    # Compute head-relative pose
                    head_rel_position, head_rel_orientation = compute_head_relative_pose(
                        g_head_tracking['smoothed_left_controller_matrix'],
                        g_head_tracking['smoothed_head_matrix']
                    )
                
                with g_lock:
                    # Update LEFT VR data with both raw and head-relative coordinates
                    g_vr_data['left']['position'][:] = position
                    g_vr_data['left']['orientation'][:] = orientation
                    g_vr_data['left']['head_relative_position'][:] = head_rel_position
                    g_vr_data['left']['head_relative_orientation'][:] = head_rel_orientation
                    g_vr_data['left']['connected'] = True
                    g_vr_data['left']['last_update'] = current_time
                    g_vr_data['left']['update_count'] += 1
                    
                    # Process LEFT button states
                    if 'leftState' in data:
                        state = data['leftState']
                        trigger_val = state.get('triggerValue', 0.0)
                        g_vr_data['left']['trigger_value'] = trigger_val
                        squeeze_pressed = state.get('squeeze', False)
                        
                        # Extract joystick data
                        joystick_x = state.get('thumbstickX', state.get('joystickX', state.get('axisX', 0.0)))
                        joystick_y = state.get('thumbstickY', state.get('joystickY', state.get('axisY', 0.0)))
                        
                        if joystick_x == 0.0 and joystick_y == 0.0:
                            joystick_data = state.get('joystick', state.get('thumbstick_axis', [0.0, 0.0]))
                            if isinstance(joystick_data, (list, tuple)) and len(joystick_data) >= 2:
                                joystick_x = joystick_data[0]
                                joystick_y = joystick_data[1]
                        
                        g_vr_data['left']['joystick'][0] = joystick_x
                        g_vr_data['left']['joystick'][1] = joystick_y
                        
                        # Button mapping
                        buttons = g_vr_data['left']['button_states']
                        old_buttons = buttons.copy()
                        
                        buttons['trigger'] = trigger_val > TRIGGER_THRESHOLD
                        buttons['squeeze'] = squeeze_pressed
                        buttons['x_button'] = state.get('bButton', False)  # LEFT controller uses bButton for reset
                        buttons['a_button'] = state.get('aButton', False)
                        buttons['y_button'] = False
                        buttons['b_button'] = False
                        buttons['menu'] = state.get('menu', False)
                        buttons['thumbstick'] = state.get('thumbstick', False)
                        
                        # Smart feedback for head-relative control - LEFT
                        if squeeze_pressed and not old_buttons.get('squeeze', False):
                            print(f"🎯 LEFT VR HEAD-RELATIVE MOVEMENT ENABLED: Move left controller relative to head direction", flush=True)
                        elif not squeeze_pressed and old_buttons.get('squeeze', False):
                            print(f"🎯 LEFT VR MOVEMENT DISABLED: Left robot stopped", flush=True)
                        
                        if buttons['trigger'] and not old_buttons.get('trigger', False):
                            print(f"🤏 LEFT GRIPPER CLOSE: Trigger {trigger_val:.2f}", flush=True)
                        elif not buttons['trigger'] and old_buttons.get('trigger', False):
                            print(f"🤏 LEFT GRIPPER OPEN: Trigger released", flush=True)
                            
                        if buttons['x_button'] and not old_buttons.get('x_button', False):
                            print(f"🔄 LEFT RESET BUTTON (X) PRESSED", flush=True)
            
            except Exception as e:
                print(f"LEFT VR processing error: {e}")
        
        # Process RIGHT controller
        if 'right' in data and isinstance(data['right'], (list, tuple)) and len(data['right']) == 16:
            try:
                # Get raw controller matrix
                raw_controller_matrix = np.array(data['right'], dtype=np.float32).reshape(4, 4, order='F')
                
                # Extract raw position and orientation for fallback
                position = raw_controller_matrix[:3, 3].copy()
                rotation_matrix = raw_controller_matrix[:3, :3]
                orientation = rotation_matrix_to_quaternion(rotation_matrix)
                
                with g_head_lock:
                    # Smooth right controller matrix
                    g_head_tracking['right_controller_matrix'] = raw_controller_matrix
                    g_head_tracking['smoothed_right_controller_matrix'] = mat_update(
                        g_head_tracking['smoothed_right_controller_matrix'],
                        raw_controller_matrix,
                        alpha=0.2  # Moderate smoothing for controller
                    )
                    
                    # Compute head-relative pose
                    head_rel_position, head_rel_orientation = compute_head_relative_pose(
                        g_head_tracking['smoothed_right_controller_matrix'],
                        g_head_tracking['smoothed_head_matrix']
                    )
                
                with g_lock:
                    # Update RIGHT VR data with both raw and head-relative coordinates
                    g_vr_data['right']['position'][:] = position
                    g_vr_data['right']['orientation'][:] = orientation
                    g_vr_data['right']['head_relative_position'][:] = head_rel_position
                    g_vr_data['right']['head_relative_orientation'][:] = head_rel_orientation
                    g_vr_data['right']['connected'] = True
                    g_vr_data['right']['last_update'] = current_time
                    g_vr_data['right']['update_count'] += 1
                    
                    # Process RIGHT button states
                    if 'rightState' in data:
                        state = data['rightState']
                        trigger_val = state.get('triggerValue', 0.0)
                        g_vr_data['right']['trigger_value'] = trigger_val
                        squeeze_pressed = state.get('squeeze', False)
                        
                        # Extract joystick data
                        joystick_x = state.get('thumbstickX', state.get('joystickX', state.get('axisX', 0.0)))
                        joystick_y = state.get('thumbstickY', state.get('joystickY', state.get('axisY', 0.0)))
                        
                        if joystick_x == 0.0 and joystick_y == 0.0:
                            joystick_data = state.get('joystick', state.get('thumbstick_axis', [0.0, 0.0]))
                            if isinstance(joystick_data, (list, tuple)) and len(joystick_data) >= 2:
                                joystick_x = joystick_data[0]
                                joystick_y = joystick_data[1]
                        
                        g_vr_data['right']['joystick'][0] = joystick_x
                        g_vr_data['right']['joystick'][1] = joystick_y
                        
                        # Button mapping
                        buttons = g_vr_data['right']['button_states']
                        old_buttons = buttons.copy()
                        
                        buttons['trigger'] = trigger_val > TRIGGER_THRESHOLD
                        buttons['squeeze'] = squeeze_pressed
                        buttons['a_button'] = state.get('aButton', False)  # RIGHT controller uses aButton for reset
                        buttons['x_button'] = state.get('bButton', False)
                        buttons['y_button'] = False
                        buttons['b_button'] = False
                        buttons['menu'] = state.get('menu', False)
                        buttons['thumbstick'] = state.get('thumbstick', False)
                        
                        # Smart feedback for head-relative control - RIGHT
                        if squeeze_pressed and not old_buttons.get('squeeze', False):
                            print(f"🎯 RIGHT VR HEAD-RELATIVE MOVEMENT ENABLED: Move right controller relative to head direction", flush=True)
                        elif not squeeze_pressed and old_buttons.get('squeeze', False):
                            print(f"🎯 RIGHT VR MOVEMENT DISABLED: Right robot stopped", flush=True)
                        
                        if buttons['trigger'] and not old_buttons.get('trigger', False):
                            print(f"🤏 RIGHT GRIPPER CLOSE: Trigger {trigger_val:.2f}", flush=True)
                        elif not buttons['trigger'] and old_buttons.get('trigger', False):
                            print(f"🤏 RIGHT GRIPPER OPEN: Trigger released", flush=True)
                            
                        if buttons['a_button'] and not old_buttons.get('a_button', False):
                            print(f"🔄 RIGHT RESET BUTTON (A) PRESSED", flush=True)
            
            except Exception as e:
                print(f"RIGHT VR processing error: {e}")
    
    @app.spawn(start=True)
    async def main_vr(session: VuerSession):
        print("🚀 Setting up DUAL Motion Controllers with Head Tracking...")
        
        session.upsert @ MotionControllers(
            stream=True,
            key="motion-controller",
            left=True,   # Enable LEFT controller
            right=True   # Enable RIGHT controller
        )
        
        print("✅ DUAL-ARM VR HEAD-RELATIVE Motion Controllers activated!")
        print(f"📱 Broadcasting LEFT & RIGHT VR head-relative coordinates at {BROADCAST_RATE}Hz")
        print("🧠 Turn your head/body freely - robot directions stay consistent for both arms!")
        print("🔧 Coordinates kept in VR space for plug-and-play compatibility")
        print("🎮 Left Controller → Robot 0 (Left Arm) with head tracking")
        print("🎮 Right Controller → Robot 1 (Right Arm) with head tracking")
        
        while True:
            await sleep(1)
    
    return app

def main():
    """Main function with argument parsing."""
    global BROADCAST_RATE, DEBUG_PRINTS
    
    parser = argparse.ArgumentParser(description='DUAL-ARM HEAD-RELATIVE VR Server for PiperX Teleoperation')
    parser.add_argument('--vr-port', type=int, default=DEFAULT_VR_PORT,
                        help=f'Port for VR server (default: {DEFAULT_VR_PORT})')
    parser.add_argument('--broadcast-port', type=int, default=DEFAULT_BROADCAST_PORT,
                        help=f'Port for broadcasting VR data (default: {DEFAULT_BROADCAST_PORT})')
    parser.add_argument('--broadcast-address', default=DEFAULT_BROADCAST_ADDRESS,
                        help=f'Address for broadcasting VR data (default: {DEFAULT_BROADCAST_ADDRESS})')
    parser.add_argument('--broadcast-rate', type=int, default=BROADCAST_RATE,
                        help=f'Broadcast rate in Hz (default: {BROADCAST_RATE})')
    parser.add_argument('--debug-prints', action='store_true',
                        help='Enable debug print statements')
    args = parser.parse_args()
    
    # Update global configuration
    BROADCAST_RATE = args.broadcast_rate
    DEBUG_PRINTS = args.debug_prints
    g_config.update({
        'vr_port': args.vr_port,
        'broadcast_port': args.broadcast_port,
        'broadcast_address': args.broadcast_address
    })
    
    print("🌟 DUAL-ARM HEAD-RELATIVE VR Server Starting...")
    print(f"📡 VR Server Port: {g_config['vr_port']}")
    print(f"📤 Broadcast: {g_config['broadcast_address']}:{g_config['broadcast_port']} @ {BROADCAST_RATE}Hz")
    print(f"🧠 Head Tracking: ENABLED for both arms (coordinates stay consistent when you turn)")
    print(f"⚡ Queue Length: {QUEUE_LEN} (minimal buffering)")
    print(f"🔧 Debug Prints: {'ON' if DEBUG_PRINTS else 'OFF'}")
    print("🦾 DUAL ARM MODE: Both left and right controllers with head tracking enabled")
    
    # Start broadcast thread
    broadcast_thread = threading.Thread(target=broadcast_vr_data, daemon=True)
    broadcast_thread.start()
    
    # Setup and run VR server
    vr_app = setup_vr_server()
    
    try:
        print("🎮 DUAL-ARM HEAD-RELATIVE VR Server running. Press Ctrl+C to stop.")
        print("💡 Now you can turn your head/body and robot directions stay consistent for both arms!")
        vr_app.run()
    except KeyboardInterrupt:
        print("\n🛑 DUAL-ARM HEAD-RELATIVE VR Server stopped")

if __name__ == '__main__':
    main()