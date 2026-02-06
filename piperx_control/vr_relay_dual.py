#!/usr/bin/env python3
"""
Dual Arm VR Relay Script - Handles handshake and relays VR data for BOTH controllers to robots
Listens for VR data from vr_server_dual.py and forwards to puppet robots
Supports both left and right controllers routing to separate robot arms
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

from packing_utils_vr_dual import pack_vr_data_for_udp, pack_dual_vr_data_for_udp, create_dual_vr_init_packet

# Configuration
DEFAULT_PUPPET_IP = "100.105.116.25"
UDP_PORT = 5005  # Port for robot communication
VR_LISTEN_PORT = 5006  # Port to listen for VR data from vr_server_dual.py
CONTROL_FREQUENCY = 50.0
DEAD_ZONE = 0.01
TRIGGER_THRESHOLD = 0.1  # Minimum trigger value to be considered active

def wait_for_vr_server(timeout=30.0):
    """Wait for dual-arm VR server to start broadcasting data (accepts 1 or more controllers)"""
    print(f"⏳ Waiting for VR server to start broadcasting (timeout: {timeout}s)...")
    print("💡 Make sure to run 'python vr_server_dual.py' first!")
    print("🎮 Will proceed with any number of detected controllers (1 or 2)")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind to localhost for consistency with single VR relay
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    sock.bind(("127.0.0.1", VR_LISTEN_PORT))
    sock.settimeout(1.0)
    
    start_time = time.time()
    controllers_detected = set()
    server_detected = False
    
    while time.time() - start_time < timeout:
        try:
            data, addr = sock.recvfrom(4096)
            packet = pickle.loads(data)
            
            print(f"📥 DEBUG: Received packet from {addr}: mode={packet.get('mode', 'unknown')}")
            
            if packet.get('mode') == 'vr_dual_data':
                server_detected = True
                # Check which controllers are present in the combined packet
                left_data = packet.get('left')
                right_data = packet.get('right')
                
                if left_data and left_data.get('connected'):
                    if 'left' not in controllers_detected:
                        controllers_detected.add('left')
                        print(f"✅ LEFT controller detected!")
                        
                if right_data and right_data.get('connected'):
                    if 'right' not in controllers_detected:
                        controllers_detected.add('right')
                        print(f"✅ RIGHT controller detected!")
                
                # Accept any number of controllers (1 or more) after detecting the server
                # Give it a few seconds to detect additional controllers, but don't require both
                if len(controllers_detected) >= 1 and (time.time() - start_time) >= 3.0:
                    if len(controllers_detected) == 2:
                        print("✅ VR server detected with BOTH controllers broadcasting data!")
                    else:
                        print(f"✅ VR server detected with {len(controllers_detected)} controller(s) broadcasting data!")
                        print("⚠️ Single-arm mode: Only one controller detected, but continuing...")
                    sock.close()
                    return True, list(controllers_detected)
                    
        except socket.timeout:
            elapsed = time.time() - start_time
            if server_detected:
                print(f"🔍 Server detected, waiting for controllers... {elapsed:.1f}s elapsed (controllers found: {list(controllers_detected)})")
            else:
                print(f"🔍 Still waiting for server... {elapsed:.1f}s elapsed")
        except Exception as e:
            print(f"Error: {e}")
        
        time.sleep(0.5)
    
    # If we detected a server and at least one controller, consider it a success
    if server_detected and len(controllers_detected) >= 1:
        print(f"✅ VR server timeout reached, but proceeding with {len(controllers_detected)} controller(s)")
        sock.close()
        return True, list(controllers_detected)
    
    # If we detected the server but no controllers, offer debugging mode
    if server_detected and len(controllers_detected) == 0:
        print(f"⚠️ VR server detected but no controllers found after {timeout}s")
        print("🔧 DEBUG MODE: Server is running but no controller data received.")
        print("💡 This can happen if controllers are not connected or not moving.")
        print("🎮 You can proceed in server-only mode for debugging purposes.")
        sock.close()
        return True, []  # Return success with empty controller list for debugging
    
    print(f"❌ VR server timeout after {timeout}s (found controllers: {list(controllers_detected)})")
    if len(controllers_detected) > 0:
        print("⚠️ Some controllers detected but server may not be fully ready.")
    else:
        print("❌ No VR server detected. Make sure 'python vr_server_dual.py' is running.")
    sock.close()
    return False, []

def send_dual_initialization_handshake(puppet_ip, session_id, active_robot_count, single_arm=False, timeout=10.0):
    """Send single initialization packet for dual-arm setup and wait for confirmation."""
    robot_desc = "single-arm" if single_arm else "dual-arm"
    print(f"🤝 Sending {robot_desc} initialization handshake to {puppet_ip} (active robots: {active_robot_count})...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    
    # Use default position/orientation for init
    initial_pos = np.zeros(3)
    initial_ori = np.array([1.0, 0.0, 0.0, 0.0])
    
    init_packet = create_dual_vr_init_packet(session_id, active_robot_count, single_arm, initial_pos, initial_ori)
    data = pickle.dumps(init_packet)
    
    max_attempts = 50  # Reduced from 10000 to reasonable number
    for attempt in range(max_attempts):
        try:
            print(f"📤 Sending {robot_desc} init packet (attempt {attempt + 1}/{max_attempts})...")
            print(f"   📋 Packet contents: mode={init_packet['mode']}, robots={init_packet['active_robot_count']}, single_arm={init_packet['single_arm_mode']}")
            sock.sendto(data, (puppet_ip, UDP_PORT))
            
            # Wait for acknowledgment
            print(f"⏰ Waiting for response (timeout: {timeout}s)...")
            resp_data, addr = sock.recvfrom(1024)
            resp_packet = pickle.loads(resp_data)
            print(f"📥 Received response: {resp_packet}")
            
            if resp_packet.get('mode') == 'vr_dual_init_ack':
                puppet_robot_count = resp_packet.get('confirmed_robot_count', active_robot_count)
                print(f"✅ Puppet confirmed {robot_desc} initialization!")
                print(f"   📊 Requested robots: {active_robot_count}, Puppet confirmed: {puppet_robot_count}")
                sock.close()
                return True, puppet_robot_count
                
        except socket.timeout:
            print(f"⏰ No response, retrying...")
        except Exception as e:
            print(f"❌ Init error: {e}")
        
        time.sleep(0.5)
    
    print(f"❌ Failed to establish {robot_desc} connection with puppet after all attempts")
    sock.close()
    return False, 0

def run_dual_vr_relay(puppet_ip, session_id, debug_poses=False, debug_network=False):
    """Main dual VR relay loop - listen for VR data and forward to appropriate puppet robots"""
    print(f"🚀 Starting DUAL-ARM VR relay to {puppet_ip}")
    print("📡 Listening for dual VR data and forwarding to puppet robots...")
    print("🎮 Left Controller → Robot 0 (Left Arm)")
    print("🎮 Right Controller → Robot 1 (Right Arm)")
    if debug_poses:
        print("🐛 Debug pose mode enabled")

    # Setup sockets with low-latency optimizations
    vr_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vr_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    vr_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    vr_sock.bind(("127.0.0.1", VR_LISTEN_PORT))  # Listen on localhost for consistency
    print("✅ Listening on localhost (127.0.0.1)")
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
    
    # Controller-specific data storage
    controller_data = {
        'left': None,
        'right': None
    }
    controller_stats = {
        'left': {'packets': 0, 'last_data': None},
        'right': {'packets': 0, 'last_data': None}
    }

    print(f"🚀 DUAL RELAY: Optimized for {CONTROL_FREQUENCY}Hz, expecting ~{1000/CONTROL_FREQUENCY:.1f}ms loop time")
    print("📦 Using OPTIMIZED COMBINED packet mode - single packet with both controllers")

    latest_packet = None
    try:
        while True:
            loop_start = time.time()

            # --- DRAIN THE BUFFER: Read all available packets and keep only the latest one ---
            try:
                while True:
                    # Keep receiving until the buffer is empty (which raises an exception)
                    latest_packet = vr_sock.recv(4096)  # Overwrites old packets - only latest matters!
            except (BlockingIOError, socket.error) as e:
                # This is the expected behavior when the buffer is empty
                error_msg = str(e).lower()
                if "forcibly closed" in error_msg or "connection reset" in error_msg:
                    print(f"⚠️ VR connection forcibly closed: {e}")
                    print("💡 Try restarting vr_server_dual.py and ensure network connectivity")
                    time.sleep(1.0)  # Brief pause before continuing
                elif debug_network:
                    print(f"🔧 DEBUG: Socket error (normal for non-blocking): {e}")

            # --- PROCESS THE LATEST COMBINED PACKET ---
            if latest_packet:
                try:
                    packet = pickle.loads(latest_packet)
                    latest_packet = None  # Clear after processing
                    
                    if packet.get('mode') == 'vr_dual_data':
                        packet_count += 1
                        
                        # Extract combined VR data
                        left_data = packet.get('left')
                        right_data = packet.get('right')
                        server_timestamp = packet.get('timestamp', 0)

                        # Latency calculation
                        if server_timestamp > 0:
                            delay_ms = (time.time() - server_timestamp) * 1000
                            latency_samples.append(delay_ms)
                            max_latency = max(max_latency, delay_ms)
                            if delay_ms > 500:
                                 print(f"🚨 CRITICAL LATENCY: {delay_ms:.0f}ms")

                        # Process LEFT controller data
                        if left_data and left_data.get('connected'):
                            position = np.array(left_data['position'])
                            orientation = np.array(left_data['orientation'])
                            button_states = left_data['button_states']
                            trigger_value = left_data['trigger_value']
                            joystick = left_data.get('joystick', [0.0, 0.0])

                            # Apply dead zone
                            position = np.where(np.abs(position) < DEAD_ZONE, 0, position)
                            
                            controller_data['left'] = {
                                'position': position,
                                'orientation': orientation,
                                'button_states': button_states,
                                'trigger_value': trigger_value,
                                'joystick': joystick
                            }
                            
                            controller_stats['left']['packets'] += 1
                            controller_stats['left']['last_data'] = {
                                'position': position,
                                'trigger_value': trigger_value,
                                'robot_id': 0
                            }

                            if debug_poses:
                                print(f"🐛 LEFT: pos=[{position[0]:.3f},{position[1]:.3f},{position[2]:.3f}] "
                                      f"trigger={trigger_value:.2f}")
                        else:
                            controller_data['left'] = None

                        # Process RIGHT controller data  
                        if right_data and right_data.get('connected'):
                            position = np.array(right_data['position'])
                            orientation = np.array(right_data['orientation'])
                            button_states = right_data['button_states']
                            trigger_value = right_data['trigger_value']
                            joystick = right_data.get('joystick', [0.0, 0.0])

                            # Apply dead zone
                            position = np.where(np.abs(position) < DEAD_ZONE, 0, position)
                            
                            controller_data['right'] = {
                                'position': position,
                                'orientation': orientation,
                                'button_states': button_states,
                                'trigger_value': trigger_value,
                                'joystick': joystick
                            }
                            
                            controller_stats['right']['packets'] += 1
                            controller_stats['right']['last_data'] = {
                                'position': position,
                                'trigger_value': trigger_value,
                                'robot_id': 1
                            }

                            if debug_poses:
                                print(f"🐛 RIGHT: pos=[{position[0]:.3f},{position[1]:.3f},{position[2]:.3f}] "
                                      f"trigger={trigger_value:.2f}")
                        else:
                            controller_data['right'] = None
                                      
                except Exception as e:
                    print(f"Error processing packet: {e}")
            
            # --- SEND COMBINED PACKET IF WE HAVE DATA FROM ANY CONTROLLER ---
            if packet_count > 0 and (controller_data['left'] is not None or controller_data['right'] is not None):
                try:
                    # Pack both controllers' data into single packet
                    packed_data = pack_dual_vr_data_for_udp(
                        controller_data['left'], 
                        controller_data['right'], 
                        session_id
                    )
                    puppet_data = pickle.dumps(packed_data)
                    puppet_sock.sendto(puppet_data, (puppet_ip, UDP_PORT))
                    
                    if debug_poses:
                        left_active = controller_data['left'] is not None
                        right_active = controller_data['right'] is not None
                        print(f"📦 Sent combined packet: LEFT={left_active} RIGHT={right_active}")
                        
                except Exception as e:
                    print(f"Error sending combined packet: {e}")

            # Lightweight performance monitoring (every 15 seconds)
            if time.time() - last_status_time > 15.0:
                elapsed = time.time() - last_status_time
                rate = packet_count / elapsed if elapsed > 0 else 0
                
                # Per-controller rates
                left_rate = controller_stats['left']['packets'] / elapsed if elapsed > 0 else 0
                right_rate = controller_stats['right']['packets'] / elapsed if elapsed > 0 else 0
                
                if latency_samples:
                    avg_latency = sum(latency_samples) / len(latency_samples)
                    latency_status = f"latency: avg {avg_latency:.0f}ms, max {max_latency:.0f}ms"
                    
                    if avg_latency > 100:
                        print(f"📊 Dual Relay: {rate:.0f}Hz total (L:{left_rate:.0f}Hz R:{right_rate:.0f}Hz), ⚠️ {latency_status}")
                    elif rate > 10:
                        print(f"📊 Dual Relay: {rate:.0f}Hz total (L:{left_rate:.0f}Hz R:{right_rate:.0f}Hz), {latency_status}")

                    # Reset for next period
                    latency_samples.clear()
                    max_latency = 0.0
                elif rate > 0:
                    print(f"📊 Dual Relay: {rate:.1f}Hz total (L:{left_rate:.0f}Hz R:{right_rate:.0f}Hz)")
                
                # Show last data for each controller
                for ctrl in ['left', 'right']:
                    last_data = controller_stats[ctrl]['last_data']
                    if last_data:
                        pos = last_data['position']
                        trigger = last_data['trigger_value']
                        robot_id = last_data['robot_id']
                        print(f"  {ctrl.upper()}: pos=[{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}] trigger={trigger:.2f} → Robot{robot_id}")
                
                packet_count = 0
                controller_stats['left']['packets'] = 0
                controller_stats['right']['packets'] = 0
                last_status_time = time.time()

            # Maintain control frequency
            elapsed = time.time() - loop_start
            sleep_time = max(0, loop_period - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n🛑 Dual VR relay stopped by user")
    finally:
        vr_sock.close()
        puppet_sock.close()
        print("🔚 Dual VR relay terminated")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Flexible VR Relay for PiperX Teleoperation (supports 1 or 2 controllers)')
    parser.add_argument('--puppet-ip', default=DEFAULT_PUPPET_IP,
                        help=f'IP address of puppet robot (default: {DEFAULT_PUPPET_IP})')
    parser.add_argument('--frequency', type=float, default=CONTROL_FREQUENCY,
                        help=f'Control frequency in Hz (default: {CONTROL_FREQUENCY})')
    parser.add_argument('--vr-timeout', type=float, default=30.0,
                        help='VR server connection timeout in seconds (default: 30.0)')
    parser.add_argument('--init-timeout', type=float, default=10.0,
                        help='Robot initialization handshake timeout in seconds (default: 10.0)')
    parser.add_argument('--debug-poses', action='store_true',
                        help='Print pose data for debugging')
    parser.add_argument('--skip-init', action='store_true',
                        help='Skip initialization handshake (for debugging)')
    parser.add_argument('--debug-network', action='store_true',
                        help='Enable detailed network debugging for connection issues')
    args = parser.parse_args()
    
    # Generate session ID
    session_id = uuid.uuid4().hex
    
    print(f"🤖 PiperX FLEXIBLE VR Relay")
    print(f"📡 Puppet IP: {args.puppet_ip}")
    print(f"📊 Control Frequency: {args.frequency} Hz")
    print(f"⏰ Init Timeout: {args.init_timeout}s")
    print(f"🆔 Session ID: {session_id}")
    print(f"🔍 VR Listen Port: {VR_LISTEN_PORT}")
    print("🦾 FLEXIBLE MODE: Supporting 1 or 2 controllers (single/dual arm)")
    
    try:
        # Wait for VR server to start
        vr_success, detected_controllers = wait_for_vr_server(args.vr_timeout)
        if not vr_success:
            print("❌ Exiting due to VR server connection failure")
            print("💡 Make sure to run 'python vr_server_dual.py' first!")
            return
        
        # Determine active robot count based on detected controllers
        active_robot_count = len(detected_controllers)
        single_arm = (active_robot_count == 1)
        debug_mode = (active_robot_count == 0)
        
        print(f"📊 Detected {active_robot_count} controller(s): {detected_controllers}")
        
        # Handle the case where no controllers are detected
        if debug_mode:
            print("🔧 DEBUG MODE: No controllers detected - running in server-only mode")
            print("⚠️ Skipping robot initialization due to no active controllers")
            print("💡 This is useful for testing server connectivity without controllers")
        else:
            # Send single initialization handshake for the detected setup if not skipping
            if not args.skip_init:
                if single_arm:
                    print("🤝 Initializing single-arm system...")
                else:
                    print("🤝 Initializing dual-arm system...")
                
                # Send handshake based on actual detected controllers
                success, confirmed_count = send_dual_initialization_handshake(
                    args.puppet_ip, session_id, active_robot_count, single_arm, args.init_timeout
                )
                
                if not success:
                    mode_desc = "single-arm" if single_arm else "dual-arm"
                    print(f"❌ Exiting due to {mode_desc} initialization failure")
                    return
                
                if confirmed_count != active_robot_count:
                    print(f"⚠️ Warning: Requested {active_robot_count} robots but puppet confirmed {confirmed_count}")
                    print("Continuing with puppet's confirmed configuration...")
                
                if single_arm:
                    print("✅ Single-arm system initialized successfully!")
                else:
                    print("✅ Dual-arm system initialized successfully!")
            else:
                print("⚠️ Skipping initialization handshake (debug mode)")
        
        # Start main relay loop
        run_dual_vr_relay(args.puppet_ip, session_id, args.debug_poses, args.debug_network)
        
    except KeyboardInterrupt:
        print("\n🛑 Dual VR relay stopped by user")
    except Exception as e:
        print(f"❌ Dual VR relay error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("🔚 Dual VR Relay terminated")

if __name__ == '__main__':
    main()