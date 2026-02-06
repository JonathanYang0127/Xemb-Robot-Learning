#!/usr/bin/env python3
"""
Test Script for Dual-Arm VR Control Validation
Tests if controls for both left and right arms are being sent correctly from VR relay to puppet.
This verifies the controller routing, packet contents, and timing for dual-arm operations.
"""

import socket
import pickle
import time
import sys
import os
import threading
import argparse
from collections import defaultdict
import numpy as np

# Add parent directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from constants import UDP_PORT, VR_LISTEN_PORT
from packing_utils_vr_dual import unpack_vr_data_from_udp

# Test configuration
DEFAULT_PUPPET_IP = "100.105.116.25"  # Default puppet IP
MONITOR_DURATION = 30.0  # How long to monitor packets (seconds)
EXPECTED_FREQUENCY = 60.0  # Expected packet rate (Hz)
LATENCY_THRESHOLD = 100.0  # Latency warning threshold (ms)

class DualArmControlMonitor:
    """Monitor and validate dual-arm VR control packets."""
    
    def __init__(self, puppet_ip=DEFAULT_PUPPET_IP, monitor_duration=MONITOR_DURATION):
        self.puppet_ip = puppet_ip
        self.monitor_duration = monitor_duration
        self.start_time = None
        self.end_time = None
        
        # Packet tracking per robot arm
        self.arm_stats = {
            0: {  # Left arm (robot_id=0)
                'packets': 0,
                'last_position': None,
                'last_orientation': None,
                'last_trigger': 0.0,
                'last_buttons': {},
                'last_joystick': [0.0, 0.0],
                'last_timestamp': 0,
                'latencies': [],
                'positions': [],
                'orientations': []
            },
            1: {  # Right arm (robot_id=1)
                'packets': 0,
                'last_position': None,
                'last_orientation': None,
                'last_trigger': 0.0,
                'last_buttons': {},
                'last_joystick': [0.0, 0.0],
                'last_timestamp': 0,
                'latencies': [],
                'positions': [],
                'orientations': []
            }
        }
        
        # Overall statistics
        self.total_packets = 0
        self.unknown_robot_packets = 0
        self.session_ids = set()
        self.packet_modes = defaultdict(int)
        
        # Setup socket to intercept packets
        self.sock = None
        self.running = False
        
    def setup_monitoring_socket(self):
        """Setup socket to monitor packets being sent to puppet."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # Bind to puppet port to intercept packets
            self.sock.bind(("0.0.0.0", UDP_PORT))
            self.sock.settimeout(0.1)  # Short timeout for responsive monitoring
            print(f"✅ Monitoring socket bound to port {UDP_PORT}")
            return True
        except Exception as e:
            print(f"❌ Failed to setup monitoring socket: {e}")
            return False
    
    def process_vr_packet(self, packet, receive_time):
        """Process and validate a VR control packet."""
        try:
            # Basic packet validation
            mode = packet.get('mode', 'unknown')
            self.packet_modes[mode] += 1
            
            if mode != 'vr_teleop':
                print(f"⚠️  Non-teleop packet received: {mode}")
                return
            
            # Track session ID
            session_id = packet.get('session_id', 'unknown')
            self.session_ids.add(session_id)
            
            # Unpack VR data
            position, orientation, button_states, trigger_value, timestamp, robot_id, joystick = unpack_vr_data_from_udp(packet)
            
            if position is None or orientation is None:
                print(f"⚠️  Invalid VR data in packet")
                return
            
            # Validate robot_id
            if robot_id not in [0, 1]:
                self.unknown_robot_packets += 1
                print(f"⚠️  Unknown robot_id: {robot_id}")
                return
            
            # Calculate latency if timestamp available
            latency_ms = 0
            if timestamp > 0:
                latency_ms = (receive_time - timestamp) * 1000
                self.arm_stats[robot_id]['latencies'].append(latency_ms)
                
                if latency_ms > LATENCY_THRESHOLD:
                    print(f"🚨 HIGH LATENCY Robot {robot_id}: {latency_ms:.1f}ms")
            
            # Update arm statistics
            arm = self.arm_stats[robot_id]
            arm['packets'] += 1
            arm['last_position'] = position.copy()
            arm['last_orientation'] = orientation.copy()
            arm['last_trigger'] = trigger_value
            arm['last_buttons'] = button_states.copy()
            arm['last_joystick'] = joystick.copy() if joystick else [0.0, 0.0]
            arm['last_timestamp'] = receive_time
            
            # Store for analysis
            arm['positions'].append(position.copy())
            arm['orientations'].append(orientation.copy())
            
            self.total_packets += 1
            
            # Real-time feedback for significant movements
            if np.linalg.norm(position) > 0.1 or trigger_value > 0.1 or any(button_states.values()):
                arm_name = "LEFT" if robot_id == 0 else "RIGHT"
                print(f"🎮 {arm_name} ARM: pos=[{position[0]:.3f},{position[1]:.3f},{position[2]:.3f}] "
                      f"trigger={trigger_value:.2f} buttons={[k for k,v in button_states.items() if v]}")
                
        except Exception as e:
            print(f"❌ Error processing packet: {e}")
    
    def run_monitoring(self):
        """Run the packet monitoring loop."""
        if not self.setup_monitoring_socket():
            return False
        
        print(f"🔍 Monitoring dual-arm VR controls for {self.monitor_duration} seconds...")
        print(f"📡 Expecting packets on port {UDP_PORT} from VR relay")
        print(f"🎯 Target frequency: {EXPECTED_FREQUENCY:.1f} Hz per arm")
        print("🚀 Monitoring started...\n")
        
        self.start_time = time.time()
        self.running = True
        packet_count = 0
        
        try:
            while self.running and (time.time() - self.start_time) < self.monitor_duration:
                try:
                    data, addr = self.sock.recvfrom(4096)
                    receive_time = time.time()
                    
                    packet = pickle.loads(data)
                    self.process_vr_packet(packet, receive_time)
                    packet_count += 1
                    
                    # Progress indicator every 5 seconds
                    elapsed = time.time() - self.start_time
                    if packet_count % 300 == 0:  # Approximately every 5s at 60Hz
                        print(f"📊 Progress: {elapsed:.1f}s elapsed, {packet_count} packets processed")
                    
                except socket.timeout:
                    # Normal timeout, continue monitoring
                    continue
                except Exception as e:
                    print(f"❌ Error receiving packet: {e}")
                    continue
        
        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped by user")
        finally:
            self.end_time = time.time()
            self.running = False
            if self.sock:
                self.sock.close()
        
        return True
    
    def analyze_results(self):
        """Analyze and report the monitoring results."""
        if not self.start_time or not self.end_time:
            print("❌ No valid monitoring session to analyze")
            return False
        
        total_duration = self.end_time - self.start_time
        print(f"\n{'='*60}")
        print(f"🔍 DUAL-ARM VR CONTROL ANALYSIS REPORT")
        print(f"{'='*60}")
        print(f"📊 Monitoring Duration: {total_duration:.1f} seconds")
        print(f"🎮 Total Packets Received: {self.total_packets}")
        print(f"📡 Overall Rate: {self.total_packets/total_duration:.1f} Hz")
        print(f"🆔 Unique Session IDs: {len(self.session_ids)}")
        print(f"❓ Unknown Robot ID Packets: {self.unknown_robot_packets}")
        
        # Packet mode breakdown
        print(f"\n📋 Packet Modes:")
        for mode, count in self.packet_modes.items():
            print(f"   {mode}: {count} packets")
        
        # Per-arm analysis
        print(f"\n🤖 PER-ARM ANALYSIS:")
        for robot_id in [0, 1]:
            arm_name = "LEFT ARM (Robot 0)" if robot_id == 0 else "RIGHT ARM (Robot 1)"
            arm = self.arm_stats[robot_id]
            
            print(f"\n  🎮 {arm_name}:")
            print(f"     📦 Packets: {arm['packets']}")
            
            if arm['packets'] > 0:
                rate = arm['packets'] / total_duration
                print(f"     📊 Rate: {rate:.1f} Hz")
                
                # Rate validation
                if rate >= EXPECTED_FREQUENCY * 0.8:  # 80% of expected
                    print(f"     ✅ Rate is GOOD (>= {EXPECTED_FREQUENCY*0.8:.1f} Hz)")
                elif rate >= EXPECTED_FREQUENCY * 0.5:  # 50% of expected
                    print(f"     ⚠️  Rate is LOW (< {EXPECTED_FREQUENCY*0.8:.1f} Hz)")
                else:
                    print(f"     ❌ Rate is CRITICAL (< {EXPECTED_FREQUENCY*0.5:.1f} Hz)")
                
                # Latency analysis
                if arm['latencies']:
                    avg_latency = sum(arm['latencies']) / len(arm['latencies'])
                    max_latency = max(arm['latencies'])
                    print(f"     ⏰ Latency: avg {avg_latency:.1f}ms, max {max_latency:.1f}ms")
                    
                    if avg_latency <= 50:
                        print(f"     ✅ Latency is EXCELLENT (<= 50ms)")
                    elif avg_latency <= 100:
                        print(f"     ✅ Latency is GOOD (<= 100ms)")
                    else:
                        print(f"     ⚠️  Latency is HIGH (> 100ms)")
                
                # Motion analysis
                if arm['positions']:
                    positions = np.array(arm['positions'])
                    position_range = np.max(positions, axis=0) - np.min(positions, axis=0)
                    position_movement = np.mean(np.linalg.norm(np.diff(positions, axis=0), axis=1))
                    
                    print(f"     📍 Position range: x={position_range[0]:.3f}, y={position_range[1]:.3f}, z={position_range[2]:.3f}")
                    print(f"     🏃 Avg movement: {position_movement:.3f} units/packet")
                    
                    if position_movement > 0.001:
                        print(f"     ✅ Active motion detected")
                    else:
                        print(f"     ⚠️  Minimal motion detected")
                
                # Last known state
                if arm['last_position'] is not None:
                    print(f"     🎯 Last position: [{arm['last_position'][0]:.3f}, {arm['last_position'][1]:.3f}, {arm['last_position'][2]:.3f}]")
                    print(f"     🎮 Last trigger: {arm['last_trigger']:.3f}")
                    print(f"     🕹️  Last joystick: [{arm['last_joystick'][0]:.3f}, {arm['last_joystick'][1]:.3f}]")
                    
                    if any(arm['last_buttons'].values()):
                        active_buttons = [k for k, v in arm['last_buttons'].items() if v]
                        print(f"     🔘 Active buttons: {active_buttons}")
            else:
                print(f"     ❌ NO PACKETS RECEIVED")
        
        # Overall assessment
        print(f"\n🎯 OVERALL ASSESSMENT:")
        left_packets = self.arm_stats[0]['packets']
        right_packets = self.arm_stats[1]['packets']
        
        if left_packets > 0 and right_packets > 0:
            print(f"     ✅ BOTH ARMS ACTIVE - Dual-arm control working!")
            balance = min(left_packets, right_packets) / max(left_packets, right_packets)
            if balance >= 0.8:
                print(f"     ✅ ARM BALANCE GOOD ({balance:.2f} ratio)")
            else:
                print(f"     ⚠️  ARM IMBALANCE ({balance:.2f} ratio)")
        elif left_packets > 0:
            print(f"     ⚠️  ONLY LEFT ARM ACTIVE - Right arm not detected")
        elif right_packets > 0:
            print(f"     ⚠️  ONLY RIGHT ARM ACTIVE - Left arm not detected")
        else:
            print(f"     ❌ NO ARM ACTIVITY - VR relay may not be working")
        
        if self.total_packets > 0:
            overall_rate = self.total_packets / total_duration
            if overall_rate >= EXPECTED_FREQUENCY * 1.5:  # Expecting ~60Hz per arm = 120Hz total
                print(f"     ✅ OVERALL RATE EXCELLENT ({overall_rate:.1f} Hz)")
            elif overall_rate >= EXPECTED_FREQUENCY:
                print(f"     ✅ OVERALL RATE GOOD ({overall_rate:.1f} Hz)")
            else:
                print(f"     ⚠️  OVERALL RATE LOW ({overall_rate:.1f} Hz)")
        
        print(f"\n{'='*60}")
        
        return True

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Test dual-arm VR control packet delivery')
    parser.add_argument('--puppet-ip', default=DEFAULT_PUPPET_IP, 
                        help=f'Puppet robot IP address (default: {DEFAULT_PUPPET_IP})')
    parser.add_argument('--duration', type=float, default=MONITOR_DURATION,
                        help=f'Monitoring duration in seconds (default: {MONITOR_DURATION})')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress real-time packet feedback')
    return parser.parse_args()

def main():
    """Main function for dual-arm control testing."""
    args = parse_arguments()
    
    print("🤖 DUAL-ARM VR CONTROL TESTER")
    print("=" * 50)
    print(f"🎯 Target Puppet: {args.puppet_ip}:{UDP_PORT}")
    print(f"⏰ Monitor Duration: {args.duration} seconds")
    print(f"📊 Expected Frequency: {EXPECTED_FREQUENCY} Hz per arm")
    print(f"🔧 Latency Threshold: {LATENCY_THRESHOLD} ms")
    print("=" * 50)
    
    print("\n📋 INSTRUCTIONS:")
    print("1. Start the VR server (vr_server_dual.py)")
    print("2. Start the VR relay (vr_relay_dual.py)")
    print("3. Use both VR controllers actively")
    print("4. This script will monitor packets for both arms")
    print("5. Results will be analyzed at the end\n")
    
    input("Press Enter when VR system is running and ready to test...")
    
    # Create monitor and run test
    monitor = DualArmControlMonitor(args.puppet_ip, args.duration)
    
    # Start monitoring
    if monitor.run_monitoring():
        # Analyze results
        monitor.analyze_results()
        
        # Success/failure determination
        left_ok = monitor.arm_stats[0]['packets'] > 0
        right_ok = monitor.arm_stats[1]['packets'] > 0
        
        if left_ok and right_ok:
            print("\n🎉 SUCCESS: Dual-arm VR controls are working correctly!")
            return 0
        elif left_ok or right_ok:
            print("\n⚠️  PARTIAL: Only one arm detected - check VR controllers")
            return 1
        else:
            print("\n❌ FAILURE: No VR control packets detected")
            return 2
    else:
        print("\n❌ FAILURE: Could not setup monitoring")
        return 3

if __name__ == '__main__':
    sys.exit(main())