#!/usr/bin/env python3
"""
Quick Dual-Arm VR Control Test
A simplified version for rapid testing of dual-arm control packet delivery.
"""

import socket
import pickle
import time
import sys
import os
import argparse

# Add parent directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from constants import UDP_PORT
from packing_utils_vr_dual import unpack_vr_data_from_udp

# Quick test configuration
DEFAULT_PUPPET_IP = "100.105.116.25"
QUICK_TEST_DURATION = 10.0  # Quick 10-second test

def quick_dual_arm_test(puppet_ip=DEFAULT_PUPPET_IP, duration=QUICK_TEST_DURATION):
    """Run a quick test to verify both arms are sending controls."""
    print("🚀 QUICK DUAL-ARM CONTROL TEST")
    print("=" * 40)
    print(f"📡 Monitoring {puppet_ip}:{UDP_PORT} for {duration} seconds")
    print("🎮 Move both VR controllers to test!")
    print()
    
    # Setup monitoring socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", UDP_PORT))
        sock.settimeout(0.5)
    except Exception as e:
        print(f"❌ Socket setup failed: {e}")
        return False
    
    # Track arm activity
    arm_activity = {0: 0, 1: 0}  # Left, Right
    arm_last_seen = {0: None, 1: None}
    total_packets = 0
    start_time = time.time()
    
    print("🔍 Monitoring started...")
    
    try:
        while (time.time() - start_time) < duration:
            try:
                data, addr = sock.recvfrom(4096)
                packet = pickle.loads(data)
                
                if packet.get('mode') == 'vr_teleop':
                    position, orientation, button_states, trigger_value, timestamp, robot_id, joystick = unpack_vr_data_from_udp(packet)
                    
                    if robot_id in [0, 1] and position is not None:
                        arm_activity[robot_id] += 1
                        arm_last_seen[robot_id] = time.time()
                        total_packets += 1
                        
                        # Show real-time activity
                        arm_name = "LEFT" if robot_id == 0 else "RIGHT"
                        if trigger_value > 0.1 or any(button_states.values()):
                            print(f"🎮 {arm_name}: trigger={trigger_value:.2f} buttons={[k for k,v in button_states.items() if v]}")
                        
            except socket.timeout:
                continue
            except Exception as e:
                print(f"⚠️  Packet error: {e}")
                continue
                
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    finally:
        sock.close()
    
    # Results
    elapsed = time.time() - start_time
    print(f"\n📊 QUICK TEST RESULTS ({elapsed:.1f}s):")
    print(f"   Total packets: {total_packets}")
    print(f"   Left arm (ID=0): {arm_activity[0]} packets")
    print(f"   Right arm (ID=1): {arm_activity[1]} packets")
    
    # Assessment
    left_ok = arm_activity[0] > 0
    right_ok = arm_activity[1] > 0
    
    if left_ok and right_ok:
        print("   ✅ BOTH ARMS DETECTED - Dual control working!")
        return True
    elif left_ok:
        print("   ⚠️  ONLY LEFT ARM detected")
        return False
    elif right_ok:
        print("   ⚠️  ONLY RIGHT ARM detected") 
        return False
    else:
        print("   ❌ NO ARMS detected - Check VR system")
        return False

def main():
    """Main function for quick dual-arm test."""
    parser = argparse.ArgumentParser(description='Quick dual-arm VR control test')
    parser.add_argument('--puppet-ip', default=DEFAULT_PUPPET_IP,
                        help=f'Puppet IP address (default: {DEFAULT_PUPPET_IP})')
    parser.add_argument('--duration', type=float, default=QUICK_TEST_DURATION,
                        help=f'Test duration in seconds (default: {QUICK_TEST_DURATION})')
    args = parser.parse_args()
    
    print("📋 Make sure VR system is running:")
    print("   1. vr_server_dual.py")
    print("   2. vr_relay_dual.py")
    print("   3. Both VR controllers active")
    print()
    
    input("Press Enter to start quick test...")
    
    if quick_dual_arm_test(args.puppet_ip, args.duration):
        print("\n🎉 Quick test PASSED!")
        return 0
    else:
        print("\n❌ Quick test FAILED!")
        return 1

if __name__ == '__main__':
    sys.exit(main())