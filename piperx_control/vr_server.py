#!/usr/bin/env python3
"""
Simple VR Server - Just runs Vuer and broadcasts VR controller data
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
DEFAULT_BROADCAST_ADDRESS = "127.0.0.1"
TRIGGER_THRESHOLD = 0.1

# Performance optimizations
BROADCAST_RATE = 60  # Optimal for VR (per user feedback)
QUEUE_LEN = 1  # Minimal buffering for lowest latency
DEBUG_PRINTS = False  # Disable prints for testing latency issues

# Global VR data - optimized structure
g_vr_data = {
    'position': np.zeros(3, dtype=np.float32),  # Use float32 for speed
    'orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
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
    'update_count': 0  # Track VR data updates
}
g_lock = threading.RLock()  # Use RLock for better performance

# Global configuration
g_config = {
    'vr_port': DEFAULT_VR_PORT,
    'broadcast_port': DEFAULT_BROADCAST_PORT,
    'broadcast_address': DEFAULT_BROADCAST_ADDRESS
}

def broadcast_vr_data():
    """Broadcast VR data to any listening relay scripts - OPTIMIZED FOR LOW LATENCY"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # Increase send buffer
    
    print(f"📡 Starting LOW-LATENCY VR broadcast ({BROADCAST_RATE}Hz) to {g_config['broadcast_address']}:{g_config['broadcast_port']}...")
    packet_count = 0
    last_debug = time.time()
    
    # Pre-allocate packet structure for efficiency
    packet_template = {
        'mode': 'vr_data',
        'position': [0.0, 0.0, 0.0],
        'orientation': [1.0, 0.0, 0.0, 0.0],
        'button_states': {},
        'trigger_value': 0.0,
        'timestamp': 0.0
    }
    
    loop_period = 1.0 / BROADCAST_RATE
    
    while True:
        loop_start = time.time()
        
        should_send = False
        try:
            # Minimize lock time - copy data quickly
            with g_lock:
                if g_vr_data['connected']:
                    # Fast copy using pre-allocated structure
                    packet_template['position'] = g_vr_data['position'].tolist()
                    packet_template['orientation'] = g_vr_data['orientation'].tolist()
                    packet_template['button_states'] = g_vr_data['button_states'].copy()
                    packet_template['trigger_value'] = g_vr_data['trigger_value']
                    packet_template['timestamp'] = time.time()
                    
                    should_send = True
            
            if should_send:
                # Serialize and send outside of lock for better concurrency
                data = pickle.dumps(packet_template)
                sock.sendto(data, (g_config['broadcast_address'], g_config['broadcast_port']))
                packet_count += 1
            
            # Lightweight performance monitoring
            now = time.time()
            if now - last_debug > 10.0:  # Every 10 seconds
                elapsed = now - last_debug
                rate = packet_count / elapsed if elapsed > 0 else 0
                
                if rate < BROADCAST_RATE * 0.9:  # Less than 90% of target
                    print(f"⚠️ VR: {rate:.0f}Hz (target: {BROADCAST_RATE}Hz)")
                elif packet_count > 0:
                    print(f"📡 VR: {rate:.0f}Hz OK")
                
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
    """Setup VR server with Vuer - OPTIMIZED FOR LOW LATENCY"""
    print("🎮 Starting LOW-LATENCY VR Server...")
    print("📋 Setup Instructions:")
    print(f"  1. Make sure ngrok is running: ngrok http {g_config['vr_port']}")
    print("  2. Access the ngrok URL from Meta Quest Browser")
    print("  3. Move left controller to start data streaming")
    
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
        
        # Minimize lock time and processing
        if 'left' in data and isinstance(data['left'], (list, tuple)) and len(data['left']) == 16:
            try:
                # Fast matrix operations
                transform_matrix = np.array(data['left'], dtype=np.float32).reshape(4, 4, order='F')
                position = transform_matrix[:3, 3].copy()
                rotation_matrix = transform_matrix[:3, :3]
                orientation = rotation_matrix_to_quaternion(rotation_matrix)
                
                # Fast lock acquisition and update
                with g_lock:
                    # Update VR data
                    g_vr_data['position'][:] = position  # In-place update
                    g_vr_data['orientation'][:] = orientation  # In-place update
                    g_vr_data['connected'] = True
                    g_vr_data['last_update'] = time.time()
                    g_vr_data['update_count'] += 1
                    
                    # Process button states efficiently
                    if 'leftState' in data:
                        state = data['leftState']
                        trigger_val = state.get('triggerValue', 0.0)
                        g_vr_data['trigger_value'] = trigger_val
                        squeeze_pressed = state.get('squeeze', False)
                        
                        # CONTROLS: trigger=gripper, squeeze=movement, x_button=reset
                        buttons = g_vr_data['button_states']
                        old_buttons = buttons.copy()  # Save previous state for change detection
                        
                        buttons['trigger'] = trigger_val > TRIGGER_THRESHOLD  # Gripper control
                        buttons['squeeze'] = squeeze_pressed  # Movement enable  
                        buttons['x_button'] = state.get('aButton', False)  # X button (reset)
                        buttons['y_button'] = state.get('bButton', False)  # Y button (available)
                        buttons['a_button'] = False  # Available for future use
                        buttons['b_button'] = False  # Available for future use
                        buttons['menu'] = state.get('menu', False)
                        buttons['thumbstick'] = state.get('thumbstick', False)
                        
                        # Smart feedback: Show control changes
                        if squeeze_pressed and not old_buttons.get('squeeze', False):
                            print(f"🎯 MOVEMENT ENABLED: Squeeze pressed - Move controller to control robot", flush=True)
                        elif not squeeze_pressed and old_buttons.get('squeeze', False):
                            print(f"🎯 MOVEMENT DISABLED: Squeeze released - Robot stopped", flush=True)
                        
                        if buttons['trigger'] and not old_buttons.get('trigger', False):
                            print(f"🤏 GRIPPER CLOSE: Trigger {trigger_val:.2f} - Close gripper", flush=True)
                        elif not buttons['trigger'] and old_buttons.get('trigger', False):
                            print(f"🤏 GRIPPER OPEN: Trigger released - Open gripper", flush=True)
                            
                        if buttons['x_button'] and not old_buttons.get('x_button', False):
                            print(f"🔄 RESET BUTTON (X) PRESSED - Robot will move to neutral", flush=True)
                        
                        if buttons['y_button'] and not old_buttons.get('y_button', False):
                            print(f"🔄 Y BUTTON PRESSED - Available for future features", flush=True)
            
            except Exception as e:
                print(f"VR processing error: {e}")
    
    @app.spawn(start=True)
    async def main_vr(session: VuerSession):
        print("🚀 Setting up Motion Controllers...")
        
        session.upsert @ MotionControllers(
            stream=True,
            key="motion-controller",
            left=True,
            right=False
        )
        
        print("✅ LOW-LATENCY VR Motion Controller activated!")
        print(f"📱 Broadcasting at {BROADCAST_RATE}Hz to {g_config['broadcast_address']}:{g_config['broadcast_port']}")
        
        while True:
            await sleep(1)
    
    return app

def main():
    """Main function with argument parsing"""
    global BROADCAST_RATE, DEBUG_PRINTS
    
    parser = argparse.ArgumentParser(description='LOW-LATENCY VR Server for PiperX Teleoperation')
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
    
    print("🌟 LOW-LATENCY VR Server Starting...")
    print(f"📡 VR Server Port: {g_config['vr_port']}")
    print(f"📤 Broadcast: {g_config['broadcast_address']}:{g_config['broadcast_port']} @ {BROADCAST_RATE}Hz")
    print(f"⚡ Queue Length: {QUEUE_LEN} (minimal buffering)")
    print(f"🔧 Debug Prints: {'ON' if DEBUG_PRINTS else 'OFF (use --debug-prints to enable)'}")
    
    # Start broadcast thread with high priority
    broadcast_thread = threading.Thread(target=broadcast_vr_data, daemon=True)
    broadcast_thread.start()
    
    # Setup and run VR server
    vr_app = setup_vr_server()
    
    try:
        print("🎮 LOW-LATENCY VR Server running. Press Ctrl+C to stop.")
        vr_app.run()
    except KeyboardInterrupt:
        print("\n🛑 VR Server stopped")

if __name__ == '__main__':
    main()