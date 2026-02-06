#!/usr/bin/env python3
"""
Dual Arm VR Server - Handles both left and right VR controllers
Runs Vuer and broadcasts VR controller data for both arms
OPTIMIZED FOR LOW LATENCY
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
DEFAULT_BROADCAST_ADDRESS = "127.0.0.1"  # Use localhost for consistency with relay
TRIGGER_THRESHOLD = 0.1

# Performance optimizations
BROADCAST_RATE = 60  # Optimal for VR (per user feedback)
QUEUE_LEN = 1  # Minimal buffering for lowest latency
DEBUG_PRINTS = False  # Disable prints for testing latency issues

# Global VR data - optimized structure for DUAL ARMS
g_vr_data = {
    'left': {
        'position': np.zeros(3, dtype=np.float32),
        'orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        'trigger_value': 0.0,
        'joystick': np.zeros(2, dtype=np.float32),  # [X, Y] joystick/thumbstick axes
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
        'trigger_value': 0.0,
        'joystick': np.zeros(2, dtype=np.float32),  # [X, Y] joystick/thumbstick axes
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
g_lock = threading.RLock()  # Use RLock for better performance

# Global configuration
g_config = {
    'vr_port': DEFAULT_VR_PORT,
    'broadcast_port': DEFAULT_BROADCAST_PORT,
    'broadcast_address': DEFAULT_BROADCAST_ADDRESS
}

def broadcast_vr_data():
    """Broadcast VR data for BOTH controllers to any listening relay scripts - OPTIMIZED FOR LOW LATENCY"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Remove broadcast option since we're using localhost
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # Increase send buffer
    
    print(f"📡 Starting DUAL-ARM LOW-LATENCY VR broadcast ({BROADCAST_RATE}Hz) to {g_config['broadcast_address']}:{g_config['broadcast_port']}...")
    packet_count = 0
    last_debug = time.time()
    
    # Pre-allocate combined packet structure for efficiency
    combined_packet_template = {
        'mode': 'vr_dual_data',
        'timestamp': 0.0,
        'left': None,
        'right': None
    }
    
    loop_period = 1.0 / BROADCAST_RATE
    
    while True:
        loop_start = time.time()
        
        try:
            # Minimize lock time - copy data quickly for both controllers into single packet
            with g_lock:
                current_timestamp = time.time()
                combined_packet_template['timestamp'] = current_timestamp
                
                # Process LEFT controller
                if g_vr_data['left']['connected']:
                    combined_packet_template['left'] = {
                        'position': g_vr_data['left']['position'].tolist(),
                        'orientation': g_vr_data['left']['orientation'].tolist(),
                        'button_states': g_vr_data['left']['button_states'].copy(),
                        'trigger_value': g_vr_data['left']['trigger_value'],
                        'joystick': g_vr_data['left']['joystick'].tolist(),
                        'robot_id': 0,
                        'connected': True
                    }
                else:
                    combined_packet_template['left'] = None
                
                # Process RIGHT controller
                if g_vr_data['right']['connected']:
                    combined_packet_template['right'] = {
                        'position': g_vr_data['right']['position'].tolist(),
                        'orientation': g_vr_data['right']['orientation'].tolist(),
                        'button_states': g_vr_data['right']['button_states'].copy(),
                        'trigger_value': g_vr_data['right']['trigger_value'],
                        'joystick': g_vr_data['right']['joystick'].tolist(),
                        'robot_id': 1,
                        'connected': True
                    }
                else:
                    combined_packet_template['right'] = None
            
            # Send single combined packet outside of lock
            if combined_packet_template['left'] or combined_packet_template['right']:
                data = pickle.dumps(combined_packet_template.copy())
                sock.sendto(data, (g_config['broadcast_address'], g_config['broadcast_port']))
                packet_count += 1
            
            # Lightweight performance monitoring
            now = time.time()
            if now - last_debug > 10.0:  # Every 10 seconds
                elapsed = now - last_debug
                rate = packet_count / elapsed if elapsed > 0 else 0
                
                # Count active controllers
                with g_lock:
                    active_controllers = sum(1 for ctrl in ['left', 'right'] if g_vr_data[ctrl]['connected'])
                
                # Now we send 1 packet per cycle regardless of active controllers
                target_rate = BROADCAST_RATE
                if rate < target_rate * 0.9:  # Less than 90% of target
                    print(f"⚠️ DUAL VR COMBINED: {rate:.0f}Hz (target: {target_rate}Hz, active: {active_controllers})")
                elif packet_count > 0:
                    print(f"📡 DUAL VR COMBINED: {rate:.0f}Hz OK (active controllers: {active_controllers})")
                
                packet_count = 0
                last_debug = now

            # Precise timing control
            elapsed = time.time() - loop_start
            sleep_time = max(0, loop_period - elapsed)
            
            if sleep_time > 0:
                time.sleep(sleep_time)
                
        except Exception as e:
            print(f"Broadcast error: {e}")
            time.sleep(0.001)  # Minimal recovery delay

def setup_vr_server():
    """Setup VR server with Vuer - DUAL ARM CONTROLLER SUPPORT"""
    print("🎮 Starting DUAL-ARM LOW-LATENCY VR Server...")
    print("📋 Setup Instructions:")
    print(f"  1. Make sure ngrok is running: ngrok http {g_config['vr_port']}")
    print("  2. Access the ngrok URL from Meta Quest Browser")
    print("  3. Move both left AND right controllers to start data streaming")
    print("  4. Left controller → Robot 0 (Left arm)")
    print("  5. Right controller → Robot 1 (Right arm)")
    
    # Optimized Vuer configuration for minimal latency
    app = Vuer(
        host='0.0.0.0', 
        port=g_config['vr_port'], 
        queries=dict(grid=False), 
        queue_len=QUEUE_LEN  # Reduced queue length for lower latency
    )
    
    @app.add_handler("CONTROLLER_MOVE")
    async def controller_handler(event, session):
        global g_vr_data
        
        data = event.value
        current_time = time.time()
        
        # Process LEFT controller
        if 'left' in data and isinstance(data['left'], (list, tuple)) and len(data['left']) == 16:
            try:
                # Fast matrix operations
                transform_matrix = np.array(data['left'], dtype=np.float32).reshape(4, 4, order='F')
                position = transform_matrix[:3, 3].copy()
                rotation_matrix = transform_matrix[:3, :3]
                orientation = rotation_matrix_to_quaternion(rotation_matrix)
                
                # Fast lock acquisition and update
                with g_lock:
                    # Update LEFT VR data
                    g_vr_data['left']['position'][:] = position  # In-place update
                    g_vr_data['left']['orientation'][:] = orientation  # In-place update
                    g_vr_data['left']['connected'] = True
                    g_vr_data['left']['last_update'] = current_time
                    g_vr_data['left']['update_count'] += 1
                    
                    # Process LEFT button states efficiently
                    if 'leftState' in data:
                        state = data['leftState']
                        trigger_val = state.get('triggerValue', 0.0)
                        g_vr_data['left']['trigger_value'] = trigger_val
                        squeeze_pressed = state.get('squeeze', False)
                        
                        # Extract joystick/thumbstick axis data for base control
                        # Try different possible field names for joystick data
                        joystick_x = state.get('thumbstickX', state.get('joystickX', state.get('axisX', 0.0)))
                        joystick_y = state.get('thumbstickY', state.get('joystickY', state.get('axisY', 0.0)))
                        
                        # Alternative: try to get joystick data as an array
                        if joystick_x == 0.0 and joystick_y == 0.0:
                            joystick_data = state.get('joystick', state.get('thumbstick_axis', [0.0, 0.0]))
                            if isinstance(joystick_data, (list, tuple)) and len(joystick_data) >= 2:
                                joystick_x = joystick_data[0]
                                joystick_y = joystick_data[1]
                        
                        g_vr_data['left']['joystick'][0] = joystick_x
                        g_vr_data['left']['joystick'][1] = joystick_y
                        
                        # CONTROLS: trigger=gripper, squeeze=movement, x_button=reset, joystick=base_movement
                        buttons = g_vr_data['left']['button_states']
                        old_buttons = buttons.copy()  # Save previous state for change detection
                        
                        buttons['trigger'] = trigger_val > TRIGGER_THRESHOLD  # Gripper control
                        buttons['squeeze'] = squeeze_pressed  # Movement enable  
                        buttons['x_button'] = state.get('bButton', False)  # X button (reset) - LEFT controller uses bButton
                        buttons['a_button'] = state.get('aButton', False)  # A button (available)
                        buttons['y_button'] = False  # Available for future use
                        buttons['b_button'] = False  # Available for future use
                        buttons['menu'] = state.get('menu', False)
                        buttons['thumbstick'] = state.get('thumbstick', False)
                        
                        # Smart feedback: Show control changes for LEFT
                        if squeeze_pressed and not old_buttons.get('squeeze', False):
                            print(f"🎯 LEFT MOVEMENT ENABLED: Squeeze pressed - Move left controller to control left robot", flush=True)
                        elif not squeeze_pressed and old_buttons.get('squeeze', False):
                            print(f"🎯 LEFT MOVEMENT DISABLED: Squeeze released - Left robot stopped", flush=True)
                        
                        if buttons['trigger'] and not old_buttons.get('trigger', False):
                            print(f"🤏 LEFT GRIPPER CLOSE: Trigger {trigger_val:.2f} - Close left gripper", flush=True)
                        elif not buttons['trigger'] and old_buttons.get('trigger', False):
                            print(f"🤏 LEFT GRIPPER OPEN: Trigger released - Open left gripper", flush=True)
                            
                        if buttons['x_button'] and not old_buttons.get('x_button', False):
                            print(f"🔄 LEFT RESET BUTTON (X) PRESSED - Left robot will move to neutral", flush=True)
            
            except Exception as e:
                print(f"LEFT VR processing error: {e}")
        
        # Process RIGHT controller
        if 'right' in data and isinstance(data['right'], (list, tuple)) and len(data['right']) == 16:
            try:
                # Fast matrix operations
                transform_matrix = np.array(data['right'], dtype=np.float32).reshape(4, 4, order='F')
                position = transform_matrix[:3, 3].copy()
                rotation_matrix = transform_matrix[:3, :3]
                orientation = rotation_matrix_to_quaternion(rotation_matrix)
                
                # Fast lock acquisition and update
                with g_lock:
                    # Update RIGHT VR data
                    g_vr_data['right']['position'][:] = position  # In-place update
                    g_vr_data['right']['orientation'][:] = orientation  # In-place update
                    g_vr_data['right']['connected'] = True
                    g_vr_data['right']['last_update'] = current_time
                    g_vr_data['right']['update_count'] += 1
                    
                    # Process RIGHT button states efficiently
                    if 'rightState' in data:
                        state = data['rightState']
                        trigger_val = state.get('triggerValue', 0.0)
                        g_vr_data['right']['trigger_value'] = trigger_val
                        squeeze_pressed = state.get('squeeze', False)
                        
                        # Extract joystick/thumbstick axis data for base control
                        # Try different possible field names for joystick data
                        joystick_x = state.get('thumbstickX', state.get('joystickX', state.get('axisX', 0.0)))
                        joystick_y = state.get('thumbstickY', state.get('joystickY', state.get('axisY', 0.0)))
                        
                        # Alternative: try to get joystick data as an array
                        if joystick_x == 0.0 and joystick_y == 0.0:
                            joystick_data = state.get('joystick', state.get('thumbstick_axis', [0.0, 0.0]))
                            if isinstance(joystick_data, (list, tuple)) and len(joystick_data) >= 2:
                                joystick_x = joystick_data[0]
                                joystick_y = joystick_data[1]
                        
                        g_vr_data['right']['joystick'][0] = joystick_x
                        g_vr_data['right']['joystick'][1] = joystick_y
                        
                        # CONTROLS: trigger=gripper, squeeze=movement, a_button=reset, joystick=base_movement
                        buttons = g_vr_data['right']['button_states']
                        old_buttons = buttons.copy()  # Save previous state for change detection
                        
                        buttons['trigger'] = trigger_val > TRIGGER_THRESHOLD  # Gripper control
                        buttons['squeeze'] = squeeze_pressed  # Movement enable  
                        buttons['a_button'] = state.get('aButton', False)  # A button (reset) - RIGHT controller uses aButton
                        buttons['x_button'] = state.get('bButton', False)  # X button (available)
                        buttons['y_button'] = False  # Available for future use
                        buttons['b_button'] = False  # Available for future use
                        buttons['menu'] = state.get('menu', False)
                        buttons['thumbstick'] = state.get('thumbstick', False)
                        
                        # Smart feedback: Show control changes for RIGHT
                        if squeeze_pressed and not old_buttons.get('squeeze', False):
                            print(f"🎯 RIGHT MOVEMENT ENABLED: Squeeze pressed - Move right controller to control right robot", flush=True)
                        elif not squeeze_pressed and old_buttons.get('squeeze', False):
                            print(f"🎯 RIGHT MOVEMENT DISABLED: Squeeze released - Right robot stopped", flush=True)
                        
                        if buttons['trigger'] and not old_buttons.get('trigger', False):
                            print(f"🤏 RIGHT GRIPPER CLOSE: Trigger {trigger_val:.2f} - Close right gripper", flush=True)
                        elif not buttons['trigger'] and old_buttons.get('trigger', False):
                            print(f"🤏 RIGHT GRIPPER OPEN: Trigger released - Open right gripper", flush=True)
                            
                        if buttons['a_button'] and not old_buttons.get('a_button', False):
                            print(f"🔄 RIGHT RESET BUTTON (A) PRESSED - Right robot will move to neutral", flush=True)
            
            except Exception as e:
                print(f"RIGHT VR processing error: {e}")
    
    @app.spawn(start=True)
    async def main_vr(session: VuerSession):
        print("🚀 Setting up DUAL Motion Controllers...")
        
        session.upsert @ MotionControllers(
            stream=True,
            key="motion-controller",
            left=True,   # Enable LEFT controller
            right=True   # Enable RIGHT controller 
        )
        
        print("✅ DUAL-ARM LOW-LATENCY VR Motion Controllers activated!")
        print(f"📱 Broadcasting LEFT & RIGHT at {BROADCAST_RATE}Hz to {g_config['broadcast_address']}:{g_config['broadcast_port']}")
        print("🎮 Left Controller → Robot 0 (Left Arm)")
        print("🎮 Right Controller → Robot 1 (Right Arm)")
        
        while True:
            await sleep(1)
    
    return app

def main():
    """Main function with argument parsing"""
    global BROADCAST_RATE, DEBUG_PRINTS
    
    parser = argparse.ArgumentParser(description='DUAL-ARM LOW-LATENCY VR Server for PiperX Teleoperation')
    parser.add_argument('--vr-port', type=int, default=DEFAULT_VR_PORT,
                        help=f'Port for VR server (default: {DEFAULT_VR_PORT})')
    parser.add_argument('--broadcast-port', type=int, default=DEFAULT_BROADCAST_PORT,
                        help=f'Port for broadcasting VR data (default: {DEFAULT_BROADCAST_PORT})')
    parser.add_argument('--broadcast-address', default=DEFAULT_BROADCAST_ADDRESS,
                        help=f'Address for broadcasting VR data (default: {DEFAULT_BROADCAST_ADDRESS})')
    parser.add_argument('--broadcast-rate', type=int, default=BROADCAST_RATE,
                        help=f'Broadcast rate in Hz (default: {BROADCAST_RATE})')
    parser.add_argument('--debug-prints', action='store_true',
                        help='Enable debug print statements (may cause latency)')
    args = parser.parse_args()
    
    # Update global configuration
    BROADCAST_RATE = args.broadcast_rate
    DEBUG_PRINTS = args.debug_prints
    g_config.update({
        'vr_port': args.vr_port,
        'broadcast_port': args.broadcast_port,
        'broadcast_address': args.broadcast_address
    })
    
    print("🌟 DUAL-ARM LOW-LATENCY VR Server Starting...")
    print(f"📡 VR Server Port: {g_config['vr_port']}")
    print(f"📤 Broadcast: {g_config['broadcast_address']}:{g_config['broadcast_port']} @ {BROADCAST_RATE}Hz")
    print(f"⚡ Queue Length: {QUEUE_LEN} (minimal buffering)")
    print(f"🔧 Debug Prints: {'ON' if DEBUG_PRINTS else 'OFF (use --debug-prints to enable)'}")
    print("🦾 DUAL ARM MODE: Both left and right controllers enabled")
    
    # Start broadcast thread with high priority
    broadcast_thread = threading.Thread(target=broadcast_vr_data, daemon=True)
    broadcast_thread.start()
    
    # Setup and run VR server
    vr_app = setup_vr_server()
    
    try:
        print("🎮 DUAL-ARM LOW-LATENCY VR Server running. Press Ctrl+C to stop.")
        vr_app.run()
    except KeyboardInterrupt:
        print("\n🛑 VR Server stopped")

if __name__ == '__main__':
    main()