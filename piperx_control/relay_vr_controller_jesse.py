#!/usr/bin/env python3
"""
VR Controller Relay Script
Captures Quest 3 VR controller data via Vuer and transmits to PiperX puppet robot
"""

import sys
import threading
import time
import numpy as np
import argparse
import uuid
import socket
import pickle
from datetime import datetime
from asyncio import sleep
import asyncio

try:
    from vuer import Vuer, VuerSession
    from vuer.schemas import MotionControllers
except ImportError:
    print("Error: vuer not found. Please install with: pip install vuer")
    sys.exit(1)

# Add the parent directory to the path
sys.path.append('..')
from xemb_scripts.cartesian_control.transformations import rotation_matrix_to_quaternion
from packing_utils_vr import pack_vr_data_for_udp, create_vr_init_packet

# --- Configuration ---
DEFAULT_PUPPET_IP = "100.105.116.25"  # Default puppet IP
UDP_PORT = 5005
VR_PORT = 8012  # Vuer server port
CONTROL_FREQUENCY = 50.0  # Hz - how often to send UDP packets
DEAD_ZONE = 0.01  # Controller movement dead zone
TRIGGER_THRESHOLD = 0.1  # Minimum trigger value to enable movement

# --- Global Variables for VR Controller Data ---
g_controller_data = {
    'position': np.zeros(3),
    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),  # Identity quaternion
    'trigger_value': 0.0,
    'button_states': {
        'trigger': False,
        'a_button': False,
        'b_button': False,
        'squeeze': False,
        'menu': False,
        'thumbstick': False
    },
    'connected': False,
    'last_update': time.time()
}
g_lock = threading.Lock()
g_vr_active = False

def setup_vr_system():
    """Setup the VR controller system using Vuer."""
    global g_controller_data, g_vr_active
    
    print("🎮 Setting up VR controller system...")
    print("📋 Setup Instructions:")
    print("  1. Make sure ngrok is running: ngrok http 8012")
    print("  2. Access the ngrok URL from Meta Quest Browser")
    print("  3. Ensure Quest 3 controllers are paired and active")
    print("  4. Point controllers at the screen to connect")
    
    # Create Vuer app
    app = Vuer(host='0.0.0.0', port=VR_PORT, queries=dict(grid=False), queue_len=5)
    
    @app.add_handler("CONTROLLER_MOVE")
    async def controller_handler(event, session):
        """Handle controller movement events - only process left controller."""
        global g_controller_data, g_vr_active
        
        data = event.value
        print(f"🔍 DEBUG: Raw controller data received: {data}")
        
        # Debug: Print the data structure
        print(f"🔍 Received controller data: {data}")
        print(f"🔍 Data type: {type(data)}")
        if isinstance(data, dict):
            print(f"🔍 Keys: {list(data.keys())}")
            if 'left' in data:
                print(f"🔍 Left controller data: {data['left']}")
                print(f"🔍 Left data type: {type(data['left'])}")
                if isinstance(data['left'], list):
                    print(f"🔍 Left data length: {len(data['left'])}")
        
        with g_lock:
            # Only process left controller
            if 'left' in data:
                print(f"🔍 DEBUG: Left controller data type: {type(data['left'])}")
                print(f"🔍 DEBUG: Left controller data value: {data['left']}")
                
                # Check if data is valid before processing
                if not isinstance(data['left'], (list, tuple)) or len(data['left']) != 16:
                    print(f"⚠️ DEBUG: Invalid left controller data - type: {type(data['left'])}, length: {len(data['left']) if hasattr(data['left'], '__len__') else 'N/A'}")
                    g_controller_data['connected'] = False
                    return
                
                try:
                    left_data = data['left']
                    
                    # Handle different data formats
                    if hasattr(left_data, 'data') and hasattr(left_data, 'code'):
                        # This is a msgpack ExtType - try to decode it
                        print(f"🔍 ExtType data: code={left_data.code}, data={left_data.data}")
                        # For now, skip this data as it's not in the expected format
                        g_controller_data['connected'] = False
                        return
                    
                    # Try to parse as regular array
                    if isinstance(left_data, (list, tuple)):
                        # Parse transformation matrix (column-major format from Vuer)
                        transform_matrix = np.array(left_data).reshape(4, 4, order='F')
                        
                        # Extract position (translation vector)
                        position = transform_matrix[:3, 3]
                        
                        # Extract rotation matrix and convert to quaternion
                        rotation_matrix = transform_matrix[:3, :3]
                        orientation = rotation_matrix_to_quaternion(rotation_matrix)
                        
                        # Update controller data
                        g_controller_data['position'] = position
                        g_controller_data['orientation'] = orientation
                        g_controller_data['connected'] = True
                        g_controller_data['last_update'] = time.time()
                        
                        # Process left controller button states
                        if 'leftState' in data:
                            state = data['leftState']
                            
                            # Update trigger value (analog)
                            g_controller_data['trigger_value'] = state.get('triggerValue', 0.0)
                            
                            # Update button states
                            g_controller_data['button_states'].update({
                                'trigger': g_controller_data['trigger_value'] > TRIGGER_THRESHOLD,
                                'a_button': state.get('aButton', False),
                                'b_button': state.get('bButton', False),
                                'squeeze': state.get('squeeze', False),
                                'menu': state.get('menu', False),
                                'thumbstick': state.get('thumbstick', False)
                            })
                        
                        g_vr_active = True
                        print(f"✅ Successfully processed controller data")
                    else:
                        print(f"⚠️  Unexpected left data type: {type(left_data)}")
                        g_controller_data['connected'] = False
                    
                except Exception as e:
                    print(f"❌ Error processing left controller data: {e}")
                    print(f"🔍 DEBUG: Exception details: {type(e).__name__}: {str(e)}")
                    g_controller_data['connected'] = False
            else:
                print(f"🔍 DEBUG: No left controller data in packet")
                # No left controller data
                g_controller_data['connected'] = False
    
    @app.spawn(start=True)
    async def main_vr(session: VuerSession):
        """Main VR session that sets up motion controllers."""
        print("🚀 Setting up Motion Controllers...")
        print(f"🔍 DEBUG: Session object: {session}")
        
        # Enable only left controller streaming
        session.upsert @ MotionControllers(
            stream=True, 
            key="motion-controller", 
            left=True,   # Only left controller
            right=False  # Disable right controller
        )
        
        print("✅ VR Motion Controller activated!")
        print("📱 Left controller is now streaming data")
        print("🎯 Move controller and press trigger to start teleoperation")
        print("🔍 DEBUG: Motion controller setup complete, entering keep-alive loop...")
        
        while True:
            await sleep(1)
    
    return app

def wait_for_vr_connection(timeout=30.0):
    """Wait for VR controller to connect within timeout period."""
    print(f"⏳ Waiting for VR controller connection (timeout: {timeout}s)...")
    
    start_time = time.time()
    while time.time() - start_time < timeout:
        with g_lock:
            elapsed = time.time() - start_time
            print(f"🔍 DEBUG: Waiting... {elapsed:.1f}s elapsed, connected={g_controller_data['connected']}")
            if g_controller_data['connected']:
                print("✅ VR controller connected!")
                return True
        time.sleep(0.1)
    
    print(f"❌ VR controller connection timeout after {timeout}s")
    print(f"🔍 DEBUG: Final connection state: {g_controller_data['connected']}")
    return False

def send_initialization_handshake(puppet_ip, session_id, robot_id):
    """Send initialization packet and wait for confirmation."""
    print(f"🤝 Sending initialization handshake to {puppet_ip}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    # Get current controller position/orientation for initial reference
    with g_lock:
        initial_pos = g_controller_data['position'].copy()
        initial_ori = g_controller_data['orientation'].copy()
    
    init_packet = create_vr_init_packet(session_id, robot_id, initial_pos, initial_ori)
    data = pickle.dumps(init_packet)
    
    max_attempts = 10
    for attempt in range(max_attempts):
        try:
            print(f"📤 Sending init packet (attempt {attempt + 1}/{max_attempts})...")
            sock.sendto(data, (puppet_ip, UDP_PORT))
            
            # Wait for acknowledgment
            resp_data, addr = sock.recvfrom(1024)
            resp_packet = pickle.loads(resp_data)
            
            if resp_packet.get('mode') == 'vr_init_ack':
                print("✅ Puppet confirmed initialization!")
                sock.close()
                return True
                
        except socket.timeout:
            print(f"⏰ No response, retrying...")
        except Exception as e:
            print(f"❌ Init error: {e}")
        
        time.sleep(0.5)
    
    print("❌ Failed to establish connection with puppet after all attempts")
    sock.close()
    return False

def run_vr_relay(puppet_ip, session_id, robot_id, debug_poses=False):
    """Main VR relay loop - continuously send controller data to puppet."""
    print(f"🚀 Starting VR relay to {puppet_ip} (Robot {robot_id})")
    print("📡 Transmitting VR controller data...")
    if debug_poses:
        print("🐛 Debug pose mode enabled - printing all poses")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Control loop timing
    loop_period = 1.0 / CONTROL_FREQUENCY
    packet_count = 0
    last_status_time = time.time()
    
    try:
        while g_vr_active:
            loop_start = time.time()
            
            # Get current controller data
            with g_lock:
                if not g_controller_data['connected']:
                    time.sleep(0.01)
                    continue
                
                # Check for stale data
                if time.time() - g_controller_data['last_update'] > 1.0:
                    print("⚠️  VR controller data is stale, skipping...")
                    time.sleep(0.01)
                    continue
                
                # Copy current data
                position = g_controller_data['position'].copy()
                orientation = g_controller_data['orientation'].copy()
                button_states = g_controller_data['button_states'].copy()
                trigger_value = g_controller_data['trigger_value']
            
            # Apply dead zone to position
            position = np.where(np.abs(position) < DEAD_ZONE, 0, position)
            
            # Pack and send VR data
            try:
                packed_data = pack_vr_data_for_udp(
                    position, orientation, button_states, 
                    trigger_value, session_id, robot_id
                )
                
                data = pickle.dumps(packed_data)
                sock.sendto(data, (puppet_ip, UDP_PORT))
                packet_count += 1
                
            except Exception as e:
                print(f"❌ Error sending VR data: {e}")
            
            # Debug pose printing (if enabled)
            if debug_poses:
                print(f"🐛 Pos: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] "
                      f"Ori: [{orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f}, {orientation[3]:.3f}] "
                      f"Trigger: {trigger_value:.2f}")
            
            # Status reporting
            if time.time() - last_status_time > 5.0:
                rate = packet_count / (time.time() - last_status_time)
                print(f"📊 VR Relay Status: {rate:.1f} Hz, Trigger: {trigger_value:.2f}")
                print(f"📍 Position: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
                print(f"🧭 Orientation: w={orientation[0]:.3f}, x={orientation[1]:.3f}, y={orientation[2]:.3f}, z={orientation[3]:.3f}")
                if any(button_states.values()):
                    active_buttons = [k for k, v in button_states.items() if v]
                    print(f"🎮 Active buttons: {active_buttons}")
                packet_count = 0
                last_status_time = time.time()
            
            # Maintain control frequency
            elapsed = time.time() - loop_start
            sleep_time = max(0, loop_period - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n🛑 VR relay stopped by user")
    finally:
        sock.close()
        print("🔚 VR relay terminated")

def main():
    """Main function."""
    global CONTROL_FREQUENCY, VR_PORT
    
    parser = argparse.ArgumentParser(description='VR Controller Relay for PiperX Teleoperation')
    parser.add_argument('--puppet-ip', default=DEFAULT_PUPPET_IP, 
                       help=f'IP address of puppet robot (default: {DEFAULT_PUPPET_IP})')
    parser.add_argument('--robot-id', type=int, default=0, 
                       help='Robot ID for multi-robot setups (default: 0)')
    parser.add_argument('--frequency', type=float, default=CONTROL_FREQUENCY,
                       help=f'Control frequency in Hz (default: {CONTROL_FREQUENCY})')
    parser.add_argument('--vr-port', type=int, default=VR_PORT,
                       help=f'Vuer VR server port (default: {VR_PORT})')
    parser.add_argument('--connection-timeout', type=float, default=30.0,
                       help='VR connection timeout in seconds (default: 30.0)')
    parser.add_argument('--debug-poses', action='store_true',
                       help='Print pose data every packet for debugging (reduces performance)')
    args = parser.parse_args()
    
    # Update global settings
    CONTROL_FREQUENCY = args.frequency
    VR_PORT = args.vr_port
    
    # Generate session ID
    session_id = uuid.uuid4().hex
    
    print(f"🤖 PiperX VR Controller Relay")
    print(f"📡 Puppet IP: {args.puppet_ip}")
    print(f"🤖 Robot ID: {args.robot_id}")
    print(f"📊 Control Frequency: {args.frequency} Hz")
    print(f"🌐 VR Server Port: {args.vr_port}")
    print(f"🆔 Session ID: {session_id}")
    
    try:
        # Setup VR system
        vr_app = setup_vr_system()
        
        # Start VR server in background thread
        vr_thread = threading.Thread(target=lambda: vr_app.run(), daemon=True)
        vr_thread.start()
        
        # Wait for VR controller connection
        if not wait_for_vr_connection(args.connection_timeout):
            print("❌ Exiting due to VR connection failure")
            print("💡 VR data was never marked as connected. Try moving the controller or refreshing Vuer.")
            return
        
        print("🔍 DEBUG: VR connection successful, proceeding to handshake...")
        
        # Send initialization handshake
        if not send_initialization_handshake(args.puppet_ip, session_id, args.robot_id):
            print("❌ Exiting due to initialization failure")
            return
        
        # Start main relay loop
        run_vr_relay(args.puppet_ip, session_id, args.robot_id, args.debug_poses)
        
    except KeyboardInterrupt:
        print("\n🛑 VR relay stopped by user")
    except Exception as e:
        print(f"❌ VR relay error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("🔚 VR Controller Relay terminated")

if __name__ == '__main__':
    main() 