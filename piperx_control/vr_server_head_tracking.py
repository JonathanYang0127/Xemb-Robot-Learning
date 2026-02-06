#!/usr/bin/env python3
"""
VR Server with Head Tracking - Head-relative coordinate transformation
Keeps everything in VR coordinates for plug-and-play compatibility
OPTIMIZED FOR LOW LATENCY with HEAD-RELATIVE CONTROL
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

# Global VR data with head tracking
g_vr_data = {
    'position': np.zeros(3, dtype=np.float32),
    'orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    'head_matrix': np.eye(4, dtype=np.float32),
    'trigger_value': 0.0,
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
g_lock = threading.RLock()

# Head tracking state
g_head_tracking = {
    'head_matrix': np.eye(4, dtype=np.float32),
    'smoothed_head_matrix': np.eye(4, dtype=np.float32),
    'controller_matrix': np.eye(4, dtype=np.float32),
    'smoothed_controller_matrix': np.eye(4, dtype=np.float32),
    'head_relative_position': np.zeros(3, dtype=np.float32),
    'head_relative_orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
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
    """Broadcast head-relative VR data to puppet scripts."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
    
    print(f"📡 Starting HEAD-RELATIVE VR broadcast ({BROADCAST_RATE}Hz) to {g_config['broadcast_address']}:{g_config['broadcast_port']}...")
    packet_count = 0
    last_debug = time.time()
    
    # Pre-allocate packet structure
    packet_template = {
        'mode': 'vr_data',
        'position': [0.0, 0.0, 0.0],
        'orientation': [1.0, 0.0, 0.0, 0.0],
        'head_matrix': np.eye(4).tolist(),
        'button_states': {},
        'trigger_value': 0.0,
        'timestamp': 0.0
    }
    
    loop_period = 1.0 / BROADCAST_RATE
    
    while True:
        loop_start = time.time()
        
        should_send = False
        try:
            # Get current VR data and head tracking data
            with g_lock:
                vr_connected = g_vr_data['connected']
                if vr_connected:
                    button_states = g_vr_data['button_states'].copy()
                    trigger_value = g_vr_data['trigger_value']
            
            with g_head_lock:
                head_relative_position = g_head_tracking['head_relative_position'].copy()
                head_relative_orientation = g_head_tracking['head_relative_orientation'].copy()
                head_matrix = g_head_tracking['smoothed_head_matrix'].copy()
            
            if vr_connected:
                # Update packet with head-relative data
                packet_template['position'] = head_relative_position.tolist()
                packet_template['orientation'] = head_relative_orientation.tolist()
                packet_template['head_matrix'] = head_matrix.tolist()
                packet_template['button_states'] = button_states
                packet_template['trigger_value'] = trigger_value
                packet_template['timestamp'] = time.time()
                
                should_send = True
            
            if should_send:
                # Serialize and send
                data = pickle.dumps(packet_template)
                sock.sendto(data, (g_config['broadcast_address'], g_config['broadcast_port']))
                packet_count += 1
            
            # Performance monitoring
            now = time.time()
            if now - last_debug > 10.0:
                elapsed = now - last_debug
                rate = packet_count / elapsed if elapsed > 0 else 0
                
                if rate < BROADCAST_RATE * 0.9:
                    print(f"⚠️ HEAD-RELATIVE VR: {rate:.0f}Hz (target: {BROADCAST_RATE}Hz)")
                elif packet_count > 0:
                    print(f"📡 HEAD-RELATIVE VR: {rate:.0f}Hz OK")
                
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
    """Setup VR server with head tracking capabilities."""
    print("🎮 Starting HEAD-RELATIVE VR Server...")
    print("📋 Setup Instructions:")
    print(f"  1. Make sure ngrok is running: ngrok http {g_config['vr_port']}")
    print("  2. Access the ngrok URL from Meta Quest Browser")
    print("  3. Move left controller to start data streaming")
    print("  4. 🧠 HEAD TRACKING: Turn your head/body - coordinates stay consistent!")
    
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
                    
                    # Recompute head-relative controller pose
                    if g_vr_data['connected']:
                        position, orientation = compute_head_relative_pose(
                            g_head_tracking['smoothed_controller_matrix'],
                            g_head_tracking['smoothed_head_matrix']
                        )
                        g_head_tracking['head_relative_position'] = position
                        g_head_tracking['head_relative_orientation'] = orientation
                    
        except Exception as e:
            if DEBUG_PRINTS:
                print(f"Head tracking error: {e}")

    @app.add_handler("CONTROLLER_MOVE")
    async def controller_handler(event, session):
        global g_vr_data, g_head_tracking
        
        data = event.value
        
        if 'left' in data and isinstance(data['left'], (list, tuple)) and len(data['left']) == 16:
            try:
                # Get raw controller matrix
                raw_controller_matrix = np.array(data['left'], dtype=np.float32).reshape(4, 4, order='F')
                
                with g_head_lock:
                    # Smooth controller matrix
                    g_head_tracking['controller_matrix'] = raw_controller_matrix
                    g_head_tracking['smoothed_controller_matrix'] = mat_update(
                        g_head_tracking['smoothed_controller_matrix'],
                        raw_controller_matrix,
                        alpha=0.2  # Moderate smoothing for controller
                    )
                    
                    # Compute head-relative pose
                    position, orientation = compute_head_relative_pose(
                        g_head_tracking['smoothed_controller_matrix'],
                        g_head_tracking['smoothed_head_matrix']
                    )
                    g_head_tracking['head_relative_position'] = position
                    g_head_tracking['head_relative_orientation'] = orientation
                
                with g_lock:
                    # Update VR data with head-relative coordinates
                    g_vr_data['position'][:] = g_head_tracking['head_relative_position']
                    g_vr_data['orientation'][:] = g_head_tracking['head_relative_orientation']
                    g_vr_data['connected'] = True
                    g_vr_data['last_update'] = time.time()
                    g_vr_data['update_count'] += 1
                    
                    # Process button states
                    if 'leftState' in data:
                        state = data['leftState']
                        trigger_val = state.get('triggerValue', 0.0)
                        g_vr_data['trigger_value'] = trigger_val
                        squeeze_pressed = state.get('squeeze', False)
                        
                        # Button mapping
                        buttons = g_vr_data['button_states']
                        old_buttons = buttons.copy()
                        
                        buttons['trigger'] = trigger_val > TRIGGER_THRESHOLD
                        buttons['squeeze'] = squeeze_pressed
                        buttons['x_button'] = state.get('aButton', False)
                        buttons['y_button'] = state.get('bButton', False)
                        buttons['a_button'] = False
                        buttons['b_button'] = False
                        buttons['menu'] = state.get('menu', False)
                        buttons['thumbstick'] = state.get('thumbstick', False)
                        
                        # Smart feedback for head-relative control
                        if squeeze_pressed and not old_buttons.get('squeeze', False):
                            print(f"🎯 VR HEAD-RELATIVE MOVEMENT ENABLED: Move controller relative to your head direction", flush=True)
                        elif not squeeze_pressed and old_buttons.get('squeeze', False):
                            print(f"🎯 VR MOVEMENT DISABLED: Robot stopped", flush=True)
                        
                        if buttons['trigger'] and not old_buttons.get('trigger', False):
                            print(f"🤏 GRIPPER CLOSE: Trigger {trigger_val:.2f}", flush=True)
                        elif not buttons['trigger'] and old_buttons.get('trigger', False):
                            print(f"🤏 GRIPPER OPEN: Trigger released", flush=True)
                            
                        if buttons['x_button'] and not old_buttons.get('x_button', False):
                            print(f"🔄 RESET BUTTON (X) PRESSED", flush=True)
            
            except Exception as e:
                print(f"VR processing error: {e}")
    
    @app.spawn(start=True)
    async def main_vr(session: VuerSession):
        print("🚀 Setting up Motion Controllers with Head Tracking...")
        
        session.upsert @ MotionControllers(
            stream=True,
            key="motion-controller",
            left=True,
            right=False
        )
        
        print("✅ VR HEAD-RELATIVE Motion Controller activated!")
        print(f"📱 Broadcasting VR head-relative coordinates at {BROADCAST_RATE}Hz")
        print("🧠 Turn your head/body freely - robot directions stay consistent!")
        print("🔧 Coordinates kept in VR space for plug-and-play compatibility")
        
        while True:
            await sleep(1)
    
    return app

def main():
    """Main function with argument parsing."""
    global BROADCAST_RATE, DEBUG_PRINTS
    
    parser = argparse.ArgumentParser(description='HEAD-RELATIVE VR Server for PiperX Teleoperation')
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
    
    print("🌟 HEAD-RELATIVE VR Server Starting...")
    print(f"📡 VR Server Port: {g_config['vr_port']}")
    print(f"📤 Broadcast: {g_config['broadcast_address']}:{g_config['broadcast_port']} @ {BROADCAST_RATE}Hz")
    print(f"🧠 Head Tracking: ENABLED (coordinates stay consistent when you turn)")
    print(f"⚡ Queue Length: {QUEUE_LEN} (minimal buffering)")
    print(f"🔧 Debug Prints: {'ON' if DEBUG_PRINTS else 'OFF'}")
    
    # Start broadcast thread
    broadcast_thread = threading.Thread(target=broadcast_vr_data, daemon=True)
    broadcast_thread.start()
    
    # Setup and run VR server
    vr_app = setup_vr_server()
    
    try:
        print("🎮 HEAD-RELATIVE VR Server running. Press Ctrl+C to stop.")
        print("💡 Now you can turn your head/body and robot directions stay consistent!")
        vr_app.run()
    except KeyboardInterrupt:
        print("\n🛑 HEAD-RELATIVE VR Server stopped")

if __name__ == '__main__':
    main()