#!/usr/bin/env python3
"""
Test script to debug VR server broadcast communication
"""

import socket
import pickle
import time
import threading

def test_broadcast():
    """Test broadcasting packets"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    print("📡 Testing broadcast on port 5006...")
    
    for i in range(10):
        packet = {
            'mode': 'vr_data',
            'position': [0.1, 0.2, 0.3],
            'orientation': [1.0, 0.0, 0.0, 0.0],
            'button_states': {'trigger': False},
            'trigger_value': 0.0,
            'timestamp': time.time()
        }
        
        data = pickle.dumps(packet)
        sock.sendto(data, ('127.0.0.1', 5006))
        print(f"📤 Sent test packet {i+1}/10")
        time.sleep(0.5)
    
    sock.close()
    print("✅ Broadcast test complete")

def test_receive():
    """Test receiving packets"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 5006))
    sock.settimeout(1.0)
    
    print("👂 Listening for packets on port 5006...")
    
    start_time = time.time()
    while time.time() - start_time < 10:
        try:
            data, addr = sock.recvfrom(4096)
            packet = pickle.loads(data)
            print(f"📥 Received: {packet.get('mode', 'unknown')} from {addr}")
        except socket.timeout:
            print("⏰ No packets received")
        except Exception as e:
            print(f"❌ Error: {e}")
    
    sock.close()
    print("✅ Receive test complete")

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'send':
        test_broadcast()
    elif len(sys.argv) > 1 and sys.argv[1] == 'receive':
        test_receive()
    else:
        print("Usage:")
        print("  python test_vr_communication.py send   # Test broadcasting")
        print("  python test_vr_communication.py receive # Test receiving") 