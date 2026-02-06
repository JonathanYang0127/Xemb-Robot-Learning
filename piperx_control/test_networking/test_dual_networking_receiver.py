#!/usr/bin/env python3
"""
Test Dual Networking Receiver - Simulates VR Puppet Side
Tests the dual-arm VR handshake mechanism from the puppet perspective.
This script receives initialization packets from the sender (relay simulator).
"""

import socket
import pickle
import time
import sys
import os
import argparse
import signal

# Add the parent directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append('..')

# Test configuration
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5005

class VRPuppetReceiver:
    """Simulates the VR puppet side receiving handshake packets"""
    
    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, max_robots=2):
        self.host = host
        self.port = port
        self.max_robots = max_robots
        self.sock = None
        self.running = False
        self.received_packets = []
        self.stats = {
            'dual_init': 0,
            'legacy_init': 0,
            'unknown': 0,
            'total': 0
        }
        
    def start(self):
        """Start the mock puppet receiver"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
            self.running = True
            
            print(f"🤖 VR Puppet Receiver: Started on {self.host}:{self.port}")
            print(f"   Max robots available: {self.max_robots}")
            print("   Waiting for handshake packets...")
            print("   Press Ctrl+C to stop")
            print()
            
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(4096)
                    packet = pickle.loads(data)
                    self.received_packets.append((packet, addr, time.time()))
                    self.stats['total'] += 1
                    
                    print(f"📥 VR Puppet Receiver: Received packet from {addr}")
                    print(f"   Packet size: {len(data)} bytes")
                    print(f"   Mode: {packet.get('mode', 'unknown')}")
                    
                    # Handle dual-arm initialization
                    if packet.get('mode') == 'vr_dual_init':
                        self.handle_dual_init(packet, addr)
                        
                    # Handle legacy initialization
                    elif packet.get('mode') == 'vr_init':
                        self.handle_legacy_init(packet, addr)
                        
                    # Unknown packet type
                    else:
                        self.handle_unknown_packet(packet, addr)
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"❌ VR Puppet Receiver: Error: {e}")
                        
        except KeyboardInterrupt:
            print("\n🛑 VR Puppet Receiver: Stopped by user")
        except Exception as e:
            print(f"❌ VR Puppet Receiver: Failed to start: {e}")
        finally:
            self.stop()
    
    def handle_dual_init(self, packet, addr):
        """Handle dual-arm initialization packet"""
        self.stats['dual_init'] += 1
        
        session_id = packet.get('init_session_id', 'unknown')
        requested_robots = packet.get('active_robot_count', 1)
        single_arm = packet.get('single_arm_mode', True)
        
        print(f"   📊 Dual-arm init:")
        print(f"      Session: {session_id[:8]}...")
        print(f"      Requested robots: {requested_robots}")
        print(f"      Single-arm mode: {single_arm}")
        
        # Simulate puppet confirming available robots
        confirmed_robots = min(requested_robots, self.max_robots)
        
        if confirmed_robots != requested_robots:
            print(f"   ⚠️ Limiting robots: requested {requested_robots}, confirming {confirmed_robots}")
        
        response = {
            'mode': 'vr_dual_init_ack',
            'confirmed_robot_count': confirmed_robots
        }
        
        print(f"   📤 Sending dual-arm ack (confirmed {confirmed_robots} robots)")
        self.sock.sendto(pickle.dumps(response), addr)
        print("   ✅ Dual-arm handshake completed")
    
    def handle_legacy_init(self, packet, addr):
        """Handle legacy single-robot initialization packet"""
        self.stats['legacy_init'] += 1
        
        session_id = packet.get('init_session_id', 'unknown')
        robot_id = packet.get('robot_id', 0)
        
        print(f"   📊 Legacy init:")
        print(f"      Session: {session_id[:8]}...")
        print(f"      Robot ID: {robot_id}")
        
        response = {'mode': 'vr_init_ack'}
        
        print(f"   📤 Sending legacy ack")
        self.sock.sendto(pickle.dumps(response), addr)
        print("   ✅ Legacy handshake completed")
    
    def handle_unknown_packet(self, packet, addr):
        """Handle unknown packet types"""
        self.stats['unknown'] += 1
        
        print(f"   ⚠️ Unknown packet mode: {packet.get('mode', 'missing')}")
        print(f"   📋 Packet keys: {list(packet.keys())}")
        # Don't send response for unknown packets
    
    def stop(self):
        """Stop the receiver"""
        self.running = False
        if self.sock:
            self.sock.close()
            
        # Print statistics
        print("\n" + "=" * 50)
        print("📊 VR Puppet Receiver Statistics:")
        print(f"   Total packets received: {self.stats['total']}")
        print(f"   Dual-arm handshakes: {self.stats['dual_init']}")
        print(f"   Legacy handshakes: {self.stats['legacy_init']}")
        print(f"   Unknown packets: {self.stats['unknown']}")
        
        if self.received_packets:
            print(f"\n📝 Packet History:")
            for i, (packet, addr, timestamp) in enumerate(self.received_packets[-5:], 1):
                mode = packet.get('mode', 'unknown')
                print(f"   {i}. {mode} from {addr} at {time.strftime('%H:%M:%S', time.localtime(timestamp))}")
        
        print("🔚 VR Puppet Receiver terminated")

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n🛑 Received interrupt signal")
    sys.exit(0)

def main():
    """Main function with argument parsing"""
    parser = argparse.ArgumentParser(description='VR Puppet Handshake Receiver Test')
    parser.add_argument('--host', default=DEFAULT_HOST, help=f'Bind host (default: {DEFAULT_HOST})')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help=f'Bind port (default: {DEFAULT_PORT})')
    parser.add_argument('--max-robots', type=int, default=2, help='Maximum robots available (default: 2)')
    parser.add_argument('--single-robot', action='store_true', help='Simulate single robot available (sets max-robots=1)')
    
    args = parser.parse_args()
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    if args.single_robot:
        args.max_robots = 1
        print("🤖 Single robot mode enabled (max robots = 1)")
    
    receiver = VRPuppetReceiver(args.host, args.port, args.max_robots)
    
    try:
        receiver.start()
    except KeyboardInterrupt:
        print("\n🛑 VR Puppet Receiver stopped by user")
    finally:
        receiver.stop()

if __name__ == '__main__':
    main()