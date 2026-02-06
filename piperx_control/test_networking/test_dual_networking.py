#!/usr/bin/env python3
"""
Test Dual Networking - Comprehensive Dual Relay <-> Dual Puppet Testing
This script tests the complete dual-arm relay to puppet mapping without requiring VR hardware.
Tests latency, control sending, packet validation, and dual robot routing.

Usage:
  python test_dual_networking.py              # Run complete dual relay-puppet test
  python test_dual_networking.py --duration 60  # Run for 60 seconds
  python test_dual_networking.py --single-arm   # Test single arm mode
  python test_dual_networking.py --frequency 30  # Test at 30Hz
  
This script simulates the teleop setup and sending loop from relay_dual without VR dependencies.
It validates dual-arm robot routing (left controller -> robot_id 0, right controller -> robot_id 1).
"""

import sys
import os
import time
import socket
import pickle
import threading
import argparse
import uuid
import numpy as np
from collections import defaultdict
import math

# Add the parent directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append('.')

from packing_utils_vr_dual import (
    create_dual_vr_init_packet, 
    pack_vr_data_for_udp, 
    pack_dual_vr_data_for_udp,
    unpack_vr_data_from_udp,
    unpack_dual_vr_data_from_udp
)

# Configuration
DEFAULT_PUPPET_IP = "127.0.0.1"  # Use localhost for testing
UDP_PORT = 5005  # Port for robot communication
CONTROL_FREQUENCY = 50.0  # Default control frequency (Hz)
TEST_DURATION = 30.0  # Default test duration (seconds)
DEAD_ZONE = 0.01
TRIGGER_THRESHOLD = 0.1

class MockPuppetReceiver:
    """Mock puppet that receives and validates dual-arm control packets"""
    
    def __init__(self, host="0.0.0.0", port=UDP_PORT, max_robots=2):
        self.host = host
        self.port = port
        self.max_robots = max_robots
        self.sock = None
        self.running = False
        
        # Statistics tracking
        self.stats = {
            'total_packets': 0,
            'init_packets': 0,
            'teleop_packets': 0,
            'robot_packets': {0: 0, 1: 0},  # Per robot tracking
            'latencies': [],
            'frequencies': [],
            'last_packet_time': 0,
            'errors': 0
        }
        
        # Robot state tracking
        self.robot_states = {
            0: {'position': np.zeros(3), 'orientation': np.array([1,0,0,0]), 'last_update': 0},
            1: {'position': np.zeros(3), 'orientation': np.array([1,0,0,0]), 'last_update': 0}
        }
        
    def start(self):
        """Start the mock puppet receiver"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(0.1)
            self.running = True
            
            print(f"🤖 Mock Puppet: Listening on {self.host}:{self.port}")
            print(f"   Max robots: {self.max_robots}")
            
            last_stats_time = time.time()
            
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(4096)
                    packet = pickle.loads(data)
                    self.process_packet(packet, addr)
                    
                    # Print statistics periodically
                    if time.time() - last_stats_time > 5.0:
                        self.print_stats()
                        last_stats_time = time.time()
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"❌ Mock Puppet error: {e}")
                        self.stats['errors'] += 1
                        
        except Exception as e:
            print(f"❌ Mock Puppet failed to start: {e}")
        finally:
            self.stop()
    
    def process_packet(self, packet, addr):
        """Process received packet and update statistics"""
        current_time = time.time()
        self.stats['total_packets'] += 1
        
        mode = packet.get('mode', 'unknown')
        
        # Handle initialization packets
        if mode == 'vr_dual_init':
            self.handle_dual_init(packet, addr)
            
        # Handle legacy teleop packets
        elif mode == 'vr_teleop':
            self.handle_teleop_packet(packet, addr, current_time)
            
        # Handle new combined dual teleop packets
        elif mode == 'vr_dual_teleop':
            self.handle_dual_teleop_packet(packet, addr, current_time)
            
        else:
            print(f"⚠️ Unknown packet mode: {mode}")
    
    def handle_dual_init(self, packet, addr):
        """Handle dual-arm initialization packet"""
        self.stats['init_packets'] += 1
        
        session_id = packet.get('init_session_id', 'unknown')
        requested_robots = packet.get('active_robot_count', 1)
        single_arm = packet.get('single_arm_mode', True)
        
        print(f"📥 Init packet: session={session_id[:8]}..., robots={requested_robots}, single_arm={single_arm}")
        
        # Confirm available robots
        confirmed_robots = min(requested_robots, self.max_robots)
        
        response = {
            'mode': 'vr_dual_init_ack',
            'confirmed_robot_count': confirmed_robots
        }
        
        print(f"📤 Sending init ack: confirmed {confirmed_robots} robots")
        self.sock.sendto(pickle.dumps(response), addr)
    
    def handle_dual_teleop_packet(self, packet, addr, current_time):
        """Handle combined dual-arm teleop packet"""
        self.stats['teleop_packets'] += 1
        
        try:
            # Unpack combined VR data
            left_data, right_data, session_id = unpack_dual_vr_data_from_udp(packet)
            timestamp = packet.get('timestamp', 0)
            
            print(f"📦 COMBINED packet: LEFT={left_data is not None} RIGHT={right_data is not None}")
            
            # Process left controller data
            if left_data:
                robot_id = left_data['robot_id']  # Should be 0
                
                # Update robot tracking
                self.stats['robot_packets'][robot_id] += 1
                self.robot_states[robot_id]['position'] = left_data['position']
                self.robot_states[robot_id]['orientation'] = left_data['orientation']
                self.robot_states[robot_id]['last_update'] = current_time
                
                print(f"🎮 LEFT → Robot {robot_id}: pos=[{left_data['position'][0]:.3f},{left_data['position'][1]:.3f},{left_data['position'][2]:.3f}] trigger={left_data['trigger_value']:.2f}")
            
            # Process right controller data
            if right_data:
                robot_id = right_data['robot_id']  # Should be 1
                
                # Update robot tracking
                self.stats['robot_packets'][robot_id] += 1
                self.robot_states[robot_id]['position'] = right_data['position']
                self.robot_states[robot_id]['orientation'] = right_data['orientation']
                self.robot_states[robot_id]['last_update'] = current_time
                
                print(f"🎮 RIGHT → Robot {robot_id}: pos=[{right_data['position'][0]:.3f},{right_data['position'][1]:.3f},{right_data['position'][2]:.3f}] trigger={right_data['trigger_value']:.2f}")
            
            # Calculate latency if timestamp available
            if timestamp > 0:
                latency = (current_time - timestamp) * 1000
                self.stats['latencies'].append(latency)
                if latency > 100:  # High latency warning
                    print(f"⚠️ High latency: {latency:.1f}ms")
            
            # Calculate frequency
            if self.stats['last_packet_time'] > 0:
                freq = 1.0 / (current_time - self.stats['last_packet_time'])
                self.stats['frequencies'].append(freq)
            
            self.stats['last_packet_time'] = current_time
            
        except Exception as e:
            print(f"❌ Error processing combined packet: {e}")
            self.stats['errors'] += 1
    
    def handle_teleop_packet(self, packet, addr, current_time):
        """Handle teleop control packet"""
        self.stats['teleop_packets'] += 1
        
        try:
            # Unpack VR data
            position, orientation, button_states, trigger_value, session_id, robot_id, joystick = unpack_vr_data_from_udp(packet)
            timestamp = packet.get('timestamp', 0)
            
            # Validate robot_id
            if robot_id not in [0, 1]:
                print(f"❌ Invalid robot_id: {robot_id}")
                self.stats['errors'] += 1
                return
            
            # Update robot tracking
            self.stats['robot_packets'][robot_id] += 1
            self.robot_states[robot_id]['position'] = position
            self.robot_states[robot_id]['orientation'] = orientation
            self.robot_states[robot_id]['last_update'] = current_time
            
            # Calculate latency if timestamp available
            if timestamp > 0:
                latency = (current_time - timestamp) * 1000
                self.stats['latencies'].append(latency)
            
            # Calculate frequency
            if self.stats['last_packet_time'] > 0:
                freq = 1.0 / (current_time - self.stats['last_packet_time'])
                self.stats['frequencies'].append(freq)
            
            self.stats['last_packet_time'] = current_time
            
        except Exception as e:
            print(f"❌ Error processing teleop packet: {e}")
            self.stats['errors'] += 1
    
    def print_stats(self):
        """Print current statistics"""
        total = self.stats['total_packets']
        robot0 = self.stats['robot_packets'][0]
        robot1 = self.stats['robot_packets'][1]
        
        print(f"📊 Mock Puppet Stats: Total={total}, Robot0={robot0}, Robot1={robot1}, Errors={self.stats['errors']}")
        
        if self.stats['latencies']:
            avg_latency = np.mean(self.stats['latencies'][-100:])  # Last 100 samples
            print(f"   Latency: {avg_latency:.1f}ms avg")
        
        if self.stats['frequencies']:
            avg_freq = np.mean(self.stats['frequencies'][-100:])  # Last 100 samples
            print(f"   Frequency: {avg_freq:.1f}Hz avg")
    
    def stop(self):
        """Stop the receiver and print final statistics"""
        self.running = False
        if self.sock:
            self.sock.close()
        
        print("\n" + "=" * 50)
        print("📊 FINAL MOCK PUPPET STATISTICS:")
        print(f"   Total packets: {self.stats['total_packets']}")
        print(f"   Init packets: {self.stats['init_packets']}")
        print(f"   Teleop packets: {self.stats['teleop_packets']}")
        print(f"   Robot 0 packets: {self.stats['robot_packets'][0]}")
        print(f"   Robot 1 packets: {self.stats['robot_packets'][1]}")
        print(f"   Errors: {self.stats['errors']}")
        
        if self.stats['latencies']:
            print(f"   Avg latency: {np.mean(self.stats['latencies']):.1f}ms")
            print(f"   Max latency: {np.max(self.stats['latencies']):.1f}ms")
        
        if self.stats['frequencies']:
            print(f"   Avg frequency: {np.mean(self.stats['frequencies']):.1f}Hz")


class DualRelaySimulator:
    """Simulates the dual relay functionality without VR dependencies"""
    
    def __init__(self, puppet_ip, frequency=CONTROL_FREQUENCY, single_arm=False, combined_mode=False):
        self.puppet_ip = puppet_ip
        self.frequency = frequency
        self.single_arm = single_arm
        self.combined_mode = combined_mode
        self.session_id = uuid.uuid4().hex
        self.running = False
        
        # Control state
        self.time_offset = 0
        self.packet_count = 0
        
        # Socket for sending
        self.sock = None
        
        # For combined mode - store latest data from each controller
        self.controller_data = {
            'left': None,
            'right': None
        }
        
    def send_dual_initialization(self):
        """Send dual-arm initialization handshake"""
        print(f"🤝 Sending dual-arm initialization...")
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(10.0)
        
        try:
            # Create initialization packet
            active_robots = 1 if self.single_arm else 2
            initial_pos = np.zeros(3)
            initial_ori = np.array([1.0, 0.0, 0.0, 0.0])
            
            init_packet = create_dual_vr_init_packet(
                self.session_id, active_robots, self.single_arm, initial_pos, initial_ori
            )
            
            # Send initialization
            data = pickle.dumps(init_packet)
            print(f"📤 Sending init packet: robots={active_robots}, single_arm={self.single_arm}")
            sock.sendto(data, (self.puppet_ip, UDP_PORT))
            
            # Wait for response
            resp_data, addr = sock.recvfrom(1024)
            resp_packet = pickle.loads(resp_data)
            
            if resp_packet.get('mode') == 'vr_dual_init_ack':
                confirmed_count = resp_packet.get('confirmed_robot_count', active_robots)
                print(f"✅ Initialization successful: {confirmed_count} robots confirmed")
                return True, confirmed_count
            else:
                print(f"❌ Unexpected response: {resp_packet}")
                return False, 0
                
        except Exception as e:
            print(f"❌ Initialization failed: {e}")
            return False, 0
        finally:
            sock.close()
    
    def generate_control_data(self, robot_id, current_time):
        """Generate synthetic control data for testing"""
        # Create circular motion pattern for demonstration
        t = current_time + self.time_offset
        
        if robot_id == 0:  # Left controller
            # Larger circle on the left
            x = 0.1 * math.cos(t * 0.5) - 0.2
            y = 0.1 * math.sin(t * 0.5)
            z = 0.05 * math.sin(t * 0.3)
        else:  # Right controller
            # Smaller circle on the right
            x = 0.08 * math.cos(t * 0.7) + 0.2
            y = 0.08 * math.sin(t * 0.7)
            z = 0.03 * math.cos(t * 0.4)
        
        position = np.array([x, y, z])
        
        # Gentle rotation
        angle = t * 0.1
        orientation = np.array([
            math.cos(angle/2),
            0.1 * math.sin(angle/2),
            0.1 * math.sin(angle/2),
            0.1 * math.sin(angle/2)
        ])
        orientation = orientation / np.linalg.norm(orientation)
        
        # Varying trigger
        trigger_value = 0.5 + 0.3 * math.sin(t * 2.0)
        trigger_value = max(0.0, min(1.0, trigger_value))
        
        # Simple button states
        button_states = {
            'grip': trigger_value > 0.5,
            'menu': False,
            'trigger': trigger_value > TRIGGER_THRESHOLD
        }
        
        # Joystick data
        joystick = [0.1 * math.sin(t * 0.8), 0.1 * math.cos(t * 0.6)]
        
        return position, orientation, button_states, trigger_value, joystick
    
    def run_control_loop(self, duration):
        """Run the main control sending loop"""
        print(f"🚀 Starting dual relay simulation")
        print(f"   Duration: {duration}s")
        print(f"   Frequency: {self.frequency}Hz")
        print(f"   Single arm: {self.single_arm}")
        print(f"   Target: {self.puppet_ip}:{UDP_PORT}")
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True
        
        loop_period = 1.0 / self.frequency
        start_time = time.time()
        last_status_time = start_time
        
        # Determine which robots to simulate
        robots_to_simulate = [0] if self.single_arm else [0, 1]
        
        try:
            while self.running and (time.time() - start_time) < duration:
                loop_start = time.time()
                
                if self.combined_mode and not self.single_arm:
                    # COMBINED MODE: Collect data from both controllers and send as single packet
                    
                    # Generate data for left controller (robot_id 0)
                    left_position, left_orientation, left_button_states, left_trigger_value, left_joystick = self.generate_control_data(
                        0, time.time() - start_time
                    )
                    left_data = {
                        'position': left_position,
                        'orientation': left_orientation,
                        'button_states': left_button_states,
                        'trigger_value': left_trigger_value,
                        'joystick': left_joystick
                    }
                    
                    # Generate data for right controller (robot_id 1)
                    right_position, right_orientation, right_button_states, right_trigger_value, right_joystick = self.generate_control_data(
                        1, time.time() - start_time
                    )
                    right_data = {
                        'position': right_position,
                        'orientation': right_orientation,
                        'button_states': right_button_states,
                        'trigger_value': right_trigger_value,
                        'joystick': right_joystick
                    }
                    
                    # Pack both controllers' data into single packet
                    packed_data = pack_dual_vr_data_for_udp(left_data, right_data, self.session_id)
                    # Add timestamp manually for latency measurement
                    packed_data['timestamp'] = time.time()
                    
                    data = pickle.dumps(packed_data)
                    self.sock.sendto(data, (self.puppet_ip, UDP_PORT))
                    self.packet_count += 1
                    
                else:
                    # LEGACY MODE: Send separate packets for each robot
                    for robot_id in robots_to_simulate:
                        position, orientation, button_states, trigger_value, joystick = self.generate_control_data(
                            robot_id, time.time() - start_time
                        )
                        
                        # Pack and send data
                        packed_data = pack_vr_data_for_udp(
                            position, orientation, button_states,
                            trigger_value, self.session_id, robot_id, joystick
                        )
                        # Add timestamp manually for latency measurement
                        packed_data['timestamp'] = time.time()
                        
                        data = pickle.dumps(packed_data)
                        self.sock.sendto(data, (self.puppet_ip, UDP_PORT))
                        self.packet_count += 1
                
                # Status reporting
                if time.time() - last_status_time > 10.0:
                    elapsed = time.time() - start_time
                    rate = self.packet_count / elapsed
                    remaining = duration - elapsed
                    print(f"📊 Relay Status: {rate:.1f}Hz, {self.packet_count} packets, {remaining:.1f}s remaining")
                    last_status_time = time.time()
                
                # Maintain frequency
                elapsed = time.time() - loop_start
                sleep_time = max(0, loop_period - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n🛑 Relay simulation stopped by user")
        finally:
            self.running = False
            if self.sock:
                self.sock.close()
            
            total_time = time.time() - start_time
            final_rate = self.packet_count / total_time if total_time > 0 else 0
            print(f"🔚 Relay simulation completed: {self.packet_count} packets in {total_time:.1f}s ({final_rate:.1f}Hz)")


def run_dual_networking_test(puppet_ip, duration, frequency, single_arm, combined_mode=False):
    """Run the complete dual networking test"""
    print("🧪 DUAL NETWORKING TEST")
    print("=" * 50)
    print(f"Target: {puppet_ip}:{UDP_PORT}")
    print(f"Duration: {duration}s")
    print(f"Frequency: {frequency}Hz")
    print(f"Mode: {'Single-arm' if single_arm else 'Dual-arm'}")
    print(f"Packet mode: {'Combined' if combined_mode else 'Legacy'}")
    print()
    
    # Start mock puppet in background thread
    puppet = MockPuppetReceiver(max_robots=1 if single_arm else 2)
    puppet_thread = threading.Thread(target=puppet.start)
    puppet_thread.daemon = True
    puppet_thread.start()
    
    # Give puppet time to start
    time.sleep(1.0)
    
    # Create relay simulator
    relay = DualRelaySimulator(puppet_ip, frequency, single_arm, combined_mode)
    
    try:
        # Send initialization handshake
        success, confirmed_robots = relay.send_dual_initialization()
        if not success:
            print("❌ Initialization failed, aborting test")
            return False
        
        print(f"✅ Initialization successful with {confirmed_robots} robots")
        print()
        
        # Run control loop
        relay.run_control_loop(duration)
        
        print("\n🛑 Stopping mock puppet...")
        puppet.stop()
        
        # Final validation
        print("\n" + "=" * 50)
        print("🎯 TEST VALIDATION:")
        
        # Check packet counts
        total_expected = int(duration * frequency)
        if single_arm:
            total_expected *= 1  # Only robot 0
        else:
            if relay.combined_mode:  # Use relay.combined_mode instead of combined_mode
                total_expected *= 1  # Combined mode: 1 packet contains both robots
            else:
                total_expected *= 2  # Legacy mode: separate packets for both robots
        
        actual_packets = puppet.stats['teleop_packets']
        packet_ratio = actual_packets / total_expected if total_expected > 0 else 0
        
        print(f"   Expected packets: ~{total_expected}")
        print(f"   Actual packets: {actual_packets}")
        print(f"   Packet ratio: {packet_ratio:.2f}")
        
        # Check robot distribution
        if not single_arm:
            if relay.combined_mode:  # Use relay.combined_mode instead of combined_mode
                # Combined mode: both robots should get data from every packet
                robot0_ratio = puppet.stats['robot_packets'][0] / actual_packets if actual_packets > 0 else 0
                robot1_ratio = puppet.stats['robot_packets'][1] / actual_packets if actual_packets > 0 else 0
            else:
                # Legacy mode: robots get data from separate packets
                robot0_ratio = puppet.stats['robot_packets'][0] / (actual_packets/2) if actual_packets > 0 else 0
                robot1_ratio = puppet.stats['robot_packets'][1] / (actual_packets/2) if actual_packets > 0 else 0
            print(f"   Robot 0 ratio: {robot0_ratio:.2f}")
            print(f"   Robot 1 ratio: {robot1_ratio:.2f}")
        
        # Determine success
        success = (
            packet_ratio > 0.8 and  # At least 80% of expected packets
            puppet.stats['errors'] == 0 and  # No errors
            puppet.stats['init_packets'] > 0  # Initialization worked
        )
        
        if not single_arm:
            # For dual arm, both robots should receive packets
            success = success and puppet.stats['robot_packets'][0] > 0 and puppet.stats['robot_packets'][1] > 0
        
        print(f"   Result: {'✅ SUCCESS' if success else '❌ FAILED'}")
        return success
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        puppet.stop()
        return False


def main():
    """Main function with argument parsing"""
    parser = argparse.ArgumentParser(description='Dual Relay to Puppet Network Test')
    parser.add_argument('--puppet-ip', default=DEFAULT_PUPPET_IP,
                        help=f'Puppet IP address (default: {DEFAULT_PUPPET_IP})')
    parser.add_argument('--duration', type=float, default=TEST_DURATION,
                        help=f'Test duration in seconds (default: {TEST_DURATION})')
    parser.add_argument('--frequency', type=float, default=CONTROL_FREQUENCY,
                        help=f'Control frequency in Hz (default: {CONTROL_FREQUENCY})')
    parser.add_argument('--single-arm', action='store_true',
                        help='Test single-arm mode instead of dual-arm')
    parser.add_argument('--combined-mode', action='store_true',
                        help='Test new combined packet mode (both controllers in single packet)')
    
    args = parser.parse_args()
    
    try:
        success = run_dual_networking_test(
            args.puppet_ip, args.duration, args.frequency, args.single_arm, args.combined_mode
        )
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()