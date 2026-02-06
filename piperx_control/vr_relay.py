#!/usr/bin/env python3
"""
VR Relay Script - Handles handshake and relays VR data to robot
Listens for VR data from vr_server.py and forwards to puppet robot
"""

import sys
import socket
import pickle
import time
import numpy as np
import argparse
import uuid

# Add the parent directory to the path
sys.path.append('..')

from packing_utils_vr import pack_vr_data_for_udp, create_vr_init_packet

# Configuration
DEFAULT_PUPPET_IP = "100.105.116.25"
UDP_PORT = 5005  # Port for robot communication
VR_LISTEN_PORT = 5006  # Port to listen for VR data from vr_server.py
CONTROL_FREQUENCY = 50.0
DEAD_ZONE = 0.01
TRIGGER_THRESHOLD = 0.1  # Minimum trigger value to be considered active

def wait_for_vr_server(timeout=30.0):
    """Wait for VR server to start broadcasting data"""
    print(f"⏳ Waiting for VR server to start broadcasting (timeout: {timeout}s)...")
    print("💡 Make sure to run 'python vr_server.py' first!")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", VR_LISTEN_PORT))
    sock.settimeout(1.0)
    
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            data, addr = sock.recvfrom(4096)
            packet = pickle.loads(data)
            
            print(f"📥 DEBUG: Received packet from {addr}: mode={packet.get('mode', 'unknown')}")
            
            if packet.get('mode') == 'vr_data':
                print("✅ VR server detected and broadcasting data!")
                sock.close()
                return True
                
        except socket.timeout:
            elapsed = time.time() - start_time
            print(f"🔍 Still waiting... {elapsed:.1f}s elapsed")
        except Exception as e:
            print(f"Error: {e}")
        
        time.sleep(0.5)
    
    print(f"❌ VR server timeout after {timeout}s")
    sock.close()
    return False

def send_initialization_handshake(puppet_ip, session_id, robot_id):
    """Send initialization packet and wait for confirmation."""
    print(f"🤝 Sending initialization handshake to {puppet_ip}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    # Use default position/orientation for init
    initial_pos = np.zeros(3)
    initial_ori = np.array([1.0, 0.0, 0.0, 0.0])
    
    init_packet = create_vr_init_packet(session_id, robot_id, initial_pos, initial_ori)
    data = pickle.dumps(init_packet)
    
    max_attempts = 10000
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
    """Main VR relay loop - listen for VR data and forward to puppet"""
    print(f"🚀 Starting VR relay to {puppet_ip} (Robot {robot_id})")
    print("📡 Listening for VR data and forwarding to puppet...")
    if debug_poses:
        print("🐛 Debug pose mode enabled")

    # Setup sockets with low-latency optimizations
    vr_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vr_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    vr_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    vr_sock.bind(("127.0.0.1", VR_LISTEN_PORT))
    # Set a very short timeout to make it non-blocking
    vr_sock.settimeout(0.0)

    puppet_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Control loop timing
    loop_period = 1.0 / CONTROL_FREQUENCY
    packet_count = 0
    last_status_time = time.time()

    # Latency tracking
    latency_samples = []
    max_latency = 0.0

    print(f"🚀 RELAY: Optimized for {CONTROL_FREQUENCY}Hz, expecting ~{1000/CONTROL_FREQUENCY:.1f}ms loop time")

    latest_data = None
    try:
        while True:
            loop_start = time.time()

            # --- DRAIN THE BUFFER: Read all available packets and keep only the last one ---
            try:
                while True:
                    # Keep receiving until the buffer is empty (which raises an exception)
                    latest_data = vr_sock.recv(4096)
            except (BlockingIOError, socket.error):
                # This is the expected behavior when the buffer is empty
                pass

            # --- PROCESS THE LATEST PACKET ---
            if latest_data:
                packet = pickle.loads(latest_data)
                latest_data = None # Clear after processing

                if packet.get('mode') == 'vr_data':
                    packet_count += 1 # Increment packet count once per processed packet

                    # Extract VR data
                    position = np.array(packet['position'])
                    orientation = np.array(packet['orientation'])
                    button_states = packet['button_states']
                    trigger_value = packet['trigger_value']

                    # Latency calculation
                    server_timestamp = packet.get('timestamp', 0)
                    if server_timestamp > 0:
                        delay_ms = (time.time() - server_timestamp) * 1000
                        latency_samples.append(delay_ms)
                        max_latency = max(max_latency, delay_ms)
                        if delay_ms > 500: # Critical latency alert
                             print(f"🚨 CRITICAL LATENCY: {delay_ms:.0f}ms")

                    # Apply dead zone to position
                    position = np.where(np.abs(position) < DEAD_ZONE, 0, position)

                    # Pack and send to puppet
                    packed_data = pack_vr_data_for_udp(
                        position, orientation, button_states,
                        trigger_value, session_id, robot_id
                    )
                    puppet_data = pickle.dumps(packed_data)
                    puppet_sock.sendto(puppet_data, (puppet_ip, UDP_PORT))

                    if debug_poses:
                        print(f"🐛 Relayed: pos=[{position[0]:.3f},{position[1]:.3f},{position[2]:.3f}] "
                              f"trigger={trigger_value:.2f}")

            # Lightweight performance monitoring (every 15 seconds)
            if time.time() - last_status_time > 15.0:
                elapsed = time.time() - last_status_time
                rate = packet_count / elapsed if elapsed > 0 else 0
                
                if latency_samples:
                    avg_latency = sum(latency_samples) / len(latency_samples)
                    latency_status = f"latency: avg {avg_latency:.0f}ms, max {max_latency:.0f}ms"
                    
                    if avg_latency > 100:
                        print(f"📊 Relay: {rate:.0f}Hz, ⚠️ {latency_status}")
                    elif rate > 10:
                        print(f"📊 Relay: {rate:.0f}Hz, {latency_status}")

                    # Reset for next period
                    latency_samples.clear()
                    max_latency = 0.0
                elif rate > 0:
                    print(f"📊 Relay: {rate:.1f}Hz")
                
                packet_count = 0
                last_status_time = time.time()

            # Maintain control frequency
            elapsed = time.time() - loop_start
            sleep_time = max(0, loop_period - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n🛑 VR relay stopped by user")
    finally:
        vr_sock.close()
        puppet_sock.close()
        print("🔚 VR relay terminated")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='VR Relay for PiperX Teleoperation')
    parser.add_argument('--puppet-ip', default=DEFAULT_PUPPET_IP,
                        help=f'IP address of puppet robot (default: {DEFAULT_PUPPET_IP})')
    parser.add_argument('--robot-id', type=int, default=0,
                        help='Robot ID for multi-robot setups (default: 0)')
    parser.add_argument('--frequency', type=float, default=CONTROL_FREQUENCY,
                        help=f'Control frequency in Hz (default: {CONTROL_FREQUENCY})')
    parser.add_argument('--vr-timeout', type=float, default=30.0,
                        help='VR server connection timeout in seconds (default: 30.0)')
    parser.add_argument('--debug-poses', action='store_true',
                        help='Print pose data for debugging')
    args = parser.parse_args()
    
    # Generate session ID
    session_id = uuid.uuid4().hex
    
    print(f"🤖 PiperX VR Relay")
    print(f"📡 Puppet IP: {args.puppet_ip}")
    print(f"🤖 Robot ID: {args.robot_id}")
    print(f"📊 Control Frequency: {args.frequency} Hz")
    print(f"🆔 Session ID: {session_id}")
    print(f"🔍 VR Listen Port: {VR_LISTEN_PORT}")
    
    try:
        # Wait for VR server to start
        if not wait_for_vr_server(args.vr_timeout):
            print("❌ Exiting due to VR server connection failure")
            print("💡 Make sure to run 'python vr_server.py' first!")
            return
        
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
        print("🔚 VR Relay terminated")

if __name__ == '__main__':
    main()