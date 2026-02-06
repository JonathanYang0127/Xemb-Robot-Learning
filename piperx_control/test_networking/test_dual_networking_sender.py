#!/usr/bin/env python3
"""
Test Dual Networking Sender - Simulates VR Relay Side
Tests the dual-arm VR handshake mechanism from the relay perspective.
This script sends initialization packets to the receiver (puppet simulator).
"""

import socket
import pickle
import time
import sys
import os
import uuid
import numpy as np
import argparse

# Add the parent directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append('..')

from packing_utils_vr_dual import create_dual_vr_init_packet, create_vr_init_packet

# Test configuration
DEFAULT_HOST = "100.105.116.25"  # Default puppet IP address
DEFAULT_PORT = 5005
TIMEOUT = 10.0  # Increased timeout for real robot testing

class VRRelaySender:
    """Simulates the VR relay side sending handshake packets"""
    
    def __init__(self, puppet_host=DEFAULT_HOST, puppet_port=DEFAULT_PORT, timeout=TIMEOUT):
        self.puppet_host = puppet_host
        self.puppet_port = puppet_port
        self.timeout = timeout
        self.test_results = []
        
    def send_dual_initialization_handshake(self, session_id, active_robot_count, single_arm=False):
        """Simulate the dual-arm handshake from relay side"""
        print(f"🔗 VR Relay Sender: Starting dual-arm handshake")
        print(f"   Target: {self.puppet_host}:{self.puppet_port}")
        print(f"   Session: {session_id[:8]}...")
        print(f"   Active robots: {active_robot_count}")
        print(f"   Single-arm: {single_arm}")
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self.timeout)
        
        try:
            # Create dual init packet
            initial_pos = np.zeros(3)
            initial_ori = np.array([1.0, 0.0, 0.0, 0.0])
            
            init_packet = create_dual_vr_init_packet(
                session_id, active_robot_count, single_arm, initial_pos, initial_ori
            )
            
            # Send packet
            data = pickle.dumps(init_packet)
            print(f"🔗 VR Relay Sender: Sending dual init packet ({len(data)} bytes)")
            print(f"   📋 Packet contents: mode={init_packet['mode']}, robots={init_packet['active_robot_count']}, single_arm={init_packet['single_arm_mode']}")
            sock.sendto(data, (self.puppet_host, self.puppet_port))
            
            # Wait for response
            print(f"🔗 VR Relay Sender: Waiting for response (timeout: {self.timeout}s)...")
            resp_data, addr = sock.recvfrom(1024)
            resp_packet = pickle.loads(resp_data)
            print(f"   📥 Received response: {resp_packet}")
            
            if resp_packet.get('mode') == 'vr_dual_init_ack':
                confirmed_count = resp_packet.get('confirmed_robot_count', active_robot_count)
                print(f"✅ VR Relay Sender: Received dual-arm ack!")
                print(f"   Confirmed robots: {confirmed_count}")
                print(f"   Response from: {addr}")
                return True, confirmed_count
            else:
                print(f"❌ VR Relay Sender: Unexpected response: {resp_packet}")
                return False, 0
                
        except socket.timeout:
            print(f"⏰ VR Relay Sender: Timeout waiting for response")
            return False, 0
        except Exception as e:
            print(f"❌ VR Relay Sender: Handshake failed: {e}")
            return False, 0
        finally:
            sock.close()
    
    def send_legacy_initialization_handshake(self, session_id, robot_id=0):
        """Simulate legacy single-robot handshake for backward compatibility test"""
        print(f"🔗 VR Relay Sender: Starting legacy handshake")
        print(f"   Target: {self.puppet_host}:{self.puppet_port}")
        print(f"   Session: {session_id[:8]}...")
        print(f"   Robot ID: {robot_id}")
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self.timeout)
        
        try:
            # Create legacy init packet
            initial_pos = np.zeros(3)
            initial_ori = np.array([1.0, 0.0, 0.0, 0.0])
            
            init_packet = create_vr_init_packet(session_id, robot_id, initial_pos, initial_ori)
            
            # Send packet
            data = pickle.dumps(init_packet)
            print(f"🔗 VR Relay Sender: Sending legacy init packet ({len(data)} bytes)")
            print(f"   📋 Packet contents: mode={init_packet['mode']}, robot_id={init_packet['robot_id']}")
            sock.sendto(data, (self.puppet_host, self.puppet_port))
            
            # Wait for response
            print(f"🔗 VR Relay Sender: Waiting for response (timeout: {self.timeout}s)...")
            resp_data, addr = sock.recvfrom(1024)
            resp_packet = pickle.loads(resp_data)
            print(f"   📥 Received response: {resp_packet}")
            
            if resp_packet.get('mode') == 'vr_init_ack':
                print(f"✅ VR Relay Sender: Received legacy ack!")
                print(f"   Response from: {addr}")
                return True
            else:
                print(f"❌ VR Relay Sender: Unexpected response: {resp_packet}")
                return False
                
        except socket.timeout:
            print(f"⏰ VR Relay Sender: Timeout waiting for response")
            return False
        except Exception as e:
            print(f"❌ VR Relay Sender: Legacy handshake failed: {e}")
            return False
        finally:
            sock.close()

    def test_dual_arm_handshake(self):
        """Test dual-arm handshake"""
        print("\n🧪 Test: Dual-Arm Handshake")
        print("-" * 40)
        
        session_id = uuid.uuid4().hex
        success, confirmed_count = self.send_dual_initialization_handshake(
            session_id, active_robot_count=2, single_arm=False
        )
        
        result = success and confirmed_count == 2
        self.test_results.append(("Dual-Arm Handshake", result))
        
        if result:
            print("✅ Dual-arm handshake test PASSED")
        else:
            print(f"❌ Dual-arm handshake test FAILED")
        
        return result

    def test_single_arm_handshake(self):
        """Test single-arm handshake"""
        print("\n🧪 Test: Single-Arm Handshake")
        print("-" * 40)
        
        session_id = uuid.uuid4().hex
        success, confirmed_count = self.send_dual_initialization_handshake(
            session_id, active_robot_count=1, single_arm=True
        )
        
        result = success and confirmed_count == 1
        self.test_results.append(("Single-Arm Handshake", result))
        
        if result:
            print("✅ Single-arm handshake test PASSED")
        else:
            print(f"❌ Single-arm handshake test FAILED")
        
        return result

    def test_robot_count_negotiation(self):
        """Test robot count negotiation (expecting puppet to limit to available robots)"""
        print("\n🧪 Test: Robot Count Negotiation")
        print("-" * 40)
        print("💡 This test expects the receiver to limit robots to what's available")
        
        session_id = uuid.uuid4().hex
        success, confirmed_count = self.send_dual_initialization_handshake(
            session_id, active_robot_count=3, single_arm=False  # Request more than max
        )
        
        # Success if we get a response with reasonable robot count (≤2)
        result = success and confirmed_count <= 2 and confirmed_count > 0
        self.test_results.append(("Robot Count Negotiation", result))
        
        if result:
            print(f"✅ Robot count negotiation test PASSED (got {confirmed_count} robots)")
        else:
            print(f"❌ Robot count negotiation test FAILED")
        
        return result

    def test_backward_compatibility(self):
        """Test backward compatibility with legacy handshake"""
        print("\n🧪 Test: Backward Compatibility")
        print("-" * 40)
        
        session_id = uuid.uuid4().hex
        success = self.send_legacy_initialization_handshake(session_id, robot_id=0)
        
        self.test_results.append(("Backward Compatibility", success))
        
        if success:
            print("✅ Backward compatibility test PASSED")
        else:
            print("❌ Backward compatibility test FAILED")
        
        return success

    def run_all_tests(self):
        """Run all handshake tests"""
        print("🚀 VR Relay Sender Test Suite")
        print("=" * 50)
        print(f"Target: {self.puppet_host}:{self.puppet_port}")
        print("💡 Make sure the receiver is running!")
        print()
        
        # Wait a moment for receiver to be ready
        print("⏰ Waiting 2 seconds for receiver to be ready...")
        time.sleep(2)
        
        tests = [
            self.test_dual_arm_handshake,
            self.test_single_arm_handshake,
            self.test_robot_count_negotiation,
            self.test_backward_compatibility,
        ]
        
        for test in tests:
            try:
                test()
                time.sleep(1)  # Brief pause between tests
            except Exception as e:
                print(f"❌ Test error: {e}")
                self.test_results.append((test.__name__, False))
        
        # Print summary
        print("\n" + "=" * 50)
        passed = sum(1 for _, result in self.test_results if result)
        total = len(self.test_results)
        
        print(f"📊 Test Results: {passed}/{total} tests passed")
        
        for test_name, result in self.test_results:
            status = "✅ PASSED" if result else "❌ FAILED"
            print(f"   {test_name}: {status}")
        
        if passed == total:
            print("\n🎉 All sender tests PASSED!")
            return True
        else:
            print(f"\n❌ {total - passed} test(s) failed")
            return False

def main():
    """Main function with argument parsing"""
    parser = argparse.ArgumentParser(description='VR Relay Handshake Sender Test')
    parser.add_argument('--host', default=DEFAULT_HOST, help=f'Puppet host IP address (default: {DEFAULT_HOST})')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help=f'Puppet port (default: {DEFAULT_PORT})')
    parser.add_argument('--timeout', type=float, default=TIMEOUT, help=f'Response timeout in seconds (default: {TIMEOUT})')
    parser.add_argument('--test', choices=['dual', 'single', 'negotiate', 'legacy', 'all'], 
                       default='all', help='Which test to run (default: all)')
    
    args = parser.parse_args()
    
    sender = VRRelaySender(args.host, args.port, args.timeout)
    
    if args.test == 'all':
        success = sender.run_all_tests()
    elif args.test == 'dual':
        success = sender.test_dual_arm_handshake()
    elif args.test == 'single':
        success = sender.test_single_arm_handshake()
    elif args.test == 'negotiate':
        success = sender.test_robot_count_negotiation()
    elif args.test == 'legacy':
        success = sender.test_backward_compatibility()
    
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()