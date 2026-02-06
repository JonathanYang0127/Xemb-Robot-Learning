#!/usr/bin/env python3
"""
Debug script to test VR initialization handshake between relay and puppet
"""

import socket
import pickle
import time
import sys
import argparse
from packing_utils_vr_dual import create_dual_vr_init_packet
import uuid

UDP_PORT = 5005

def test_puppet_listener(timeout=30.0):
    """Test listening for init packets (puppet side)"""
    print("🎮 [PUPPET TEST] Starting VR initialization listener...")
    print(f"🔧 DEBUG: Binding to 0.0.0.0:{UDP_PORT}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(1.0)
    
    start_time = time.time()
    packet_count = 0
    
    while time.time() - start_time < timeout:
        try:
            print("🔧 DEBUG: Waiting for init packet...")
            data, addr = sock.recvfrom(4096)
            packet_count += 1
            print(f"🔧 DEBUG: Received packet #{packet_count} from {addr}, size: {len(data)} bytes")
            
            packet = pickle.loads(data)
            print(f"🔧 DEBUG: Unpacked packet: {packet}")
            
            mode = packet.get('mode', 'unknown')
            print(f"📥 Packet mode: {mode}")
            
            if mode == 'vr_dual_init':
                session_id = packet.get('init_session_id')
                active_robot_count = packet.get('active_robot_count', 1)
                single_arm_mode = packet.get('single_arm_mode', True)
                
                print(f"✅ [INIT] Dual-arm VR init received!")
                print(f"   📊 Session: {session_id[:8] if session_id else 'None'}...")
                print(f"   🤖 Active robots: {active_robot_count}")
                print(f"   🦾 Single-arm mode: {single_arm_mode}")
                
                # Send acknowledgment
                ack_packet = {
                    'mode': 'vr_dual_init_ack',
                    'confirmed_robot_count': min(active_robot_count, 2)
                }
                print(f"🔧 DEBUG: Sending ack: {ack_packet}")
                sock.sendto(pickle.dumps(ack_packet), addr)
                print("🔧 DEBUG: Acknowledgment sent successfully")
                
            elif mode == 'vr_init':
                session_id = packet.get('init_session_id')
                robot_id = packet.get('robot_id', 0)
                
                print(f"✅ [INIT] Legacy VR init received!")
                print(f"   📊 Session: {session_id[:8] if session_id else 'None'}...")
                print(f"   🤖 Robot ID: {robot_id}")
                
                # Send legacy acknowledgment
                ack_packet = {'mode': 'vr_init_ack'}
                print(f"🔧 DEBUG: Sending legacy ack: {ack_packet}")
                sock.sendto(pickle.dumps(ack_packet), addr)
                print("🔧 DEBUG: Legacy acknowledgment sent successfully")
                
            else:
                print(f"🔧 DEBUG: Unknown mode: {mode}")
                
        except socket.timeout:
            elapsed = time.time() - start_time
            print(f"🔧 DEBUG: Socket timeout, elapsed: {elapsed:.1f}s")
            continue
        except Exception as e:
            print(f"❌ Error: {e}")
            continue
    
    sock.close()
    print(f"🔚 Test completed. Received {packet_count} packets total.")

def test_relay_sender(puppet_ip, active_robot_count=2, single_arm=False, timeout=10.0):
    """Test sending init packets (relay side)"""
    print(f"📤 [RELAY TEST] Testing initialization handshake to {puppet_ip}")
    print(f"🤖 Robot config: {active_robot_count} robots, single_arm={single_arm}")
    
    session_id = uuid.uuid4().hex
    print(f"🆔 Session ID: {session_id}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    
    # Create init packet
    import numpy as np
    initial_pos = np.zeros(3)
    initial_ori = np.array([1.0, 0.0, 0.0, 0.0])
    
    init_packet = create_dual_vr_init_packet(session_id, active_robot_count, single_arm, initial_pos, initial_ori)
    data = pickle.dumps(init_packet)
    
    print(f"📋 Packet contents: {init_packet}")
    print(f"📏 Packet size: {len(data)} bytes")
    
    max_attempts = 10
    for attempt in range(max_attempts):
        try:
            print(f"📤 Sending init packet (attempt {attempt + 1}/{max_attempts})...")
            sock.sendto(data, (puppet_ip, UDP_PORT))
            
            print(f"⏰ Waiting for response (timeout: {timeout}s)...")
            resp_data, addr = sock.recvfrom(1024)
            resp_packet = pickle.loads(resp_data)
            print(f"📥 Received response from {addr}: {resp_packet}")
            
            if resp_packet.get('mode') == 'vr_dual_init_ack':
                confirmed_count = resp_packet.get('confirmed_robot_count', active_robot_count)
                print(f"✅ SUCCESS! Puppet confirmed dual-arm initialization!")
                print(f"   📊 Requested: {active_robot_count}, Confirmed: {confirmed_count}")
                sock.close()
                return True
            elif resp_packet.get('mode') == 'vr_init_ack':
                print(f"✅ SUCCESS! Puppet confirmed legacy initialization!")
                sock.close()
                return True
                
        except socket.timeout:
            print(f"⏰ No response, retrying...")
        except Exception as e:
            print(f"❌ Error: {e}")
        
        time.sleep(0.5)
    
    print("❌ Failed to get confirmation after all attempts")
    sock.close()
    return False

def main():
    parser = argparse.ArgumentParser(description='Debug VR initialization handshake')
    parser.add_argument('--mode', choices=['puppet', 'relay'], required=True,
                        help='Test mode: puppet (listen) or relay (send)')
    parser.add_argument('--puppet-ip', default='127.0.0.1',
                        help='IP address of puppet (relay mode only)')
    parser.add_argument('--robots', type=int, default=2,
                        help='Number of active robots (relay mode only)')
    parser.add_argument('--single-arm', action='store_true',
                        help='Single arm mode (relay mode only)')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Timeout in seconds')
    args = parser.parse_args()
    
    if args.mode == 'puppet':
        test_puppet_listener(args.timeout)
    elif args.mode == 'relay':
        test_relay_sender(args.puppet_ip, args.robots, args.single_arm, args.timeout)

if __name__ == '__main__':
    main()