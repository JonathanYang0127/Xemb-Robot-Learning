#!/usr/bin/env python3
"""
VR Server with 360° Video Integration
Combines VR controller data broadcasting with 360° video streaming using Vuer
OPTIMIZED FOR LOW LATENCY
"""

import sys
import time
import numpy as np
import socket
import pickle
import threading
import argparse
import json
import base64
import struct
from asyncio import sleep
import asyncio

try:
    from vuer import Vuer, VuerSession
    from vuer.schemas import MotionControllers, DefaultScene, Sphere
except ImportError:
    print("Error: vuer not found. Please install with: pip install vuer")
    sys.exit(1)

# Default Configuration
DEFAULT_VR_PORT = 8012
DEFAULT_BROADCAST_PORT = 5006
DEFAULT_BROADCAST_ADDRESS = "127.0.0.1"
VIDEO_UDP_PORT = 8083  # Port to receive equirectangular video
TRIGGER_THRESHOLD = 0.1

# Performance optimizations
BROADCAST_RATE = 60  # Optimal for VR (per user feedback)
QUEUE_LEN = 1  # Minimal buffering for lowest latency
DEBUG_PRINTS = False  # Disable prints for testing latency issues

# 360 Video Configuration
SPHERE_RADIUS = 500  # Large sphere to encompass user
VIDEO_TEXTURE_UPDATE_RATE = 30  # FPS for video texture updates

# Global VR data - optimized structure (same as original)
g_vr_data = {
    'position': np.zeros(3, dtype=np.float32),  # Use float32 for speed
    'orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
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
    'last_update': time.time(),
    'update_count': 0  # Track VR data updates
}
g_lock = threading.RLock()  # Use RLock for better performance

# Global 360 video data
g_360_data = {
    'current_frame': None,
    'frame_timestamp': 0,
    'frame_width': 0,
    'frame_height': 0,
    'frames_received': 0,
    'texture_url': None
}
g_360_lock = threading.RLock()

# Global configuration
g_config = {
    'vr_port': DEFAULT_VR_PORT,
    'broadcast_port': DEFAULT_BROADCAST_PORT,
    'broadcast_address': DEFAULT_BROADCAST_ADDRESS,
    'video_port': VIDEO_UDP_PORT
}

# Global Vuer session for video updates
g_vuer_session = None

def rotation_matrix_to_quaternion(R):
    """Convert 3x3 rotation matrix to quaternion [w, x, y, z] - OPTIMIZED."""
    try:
        # Fast path optimization
        R = R.astype(np.float32)  # Ensure float32 for speed
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        quat = np.array([w, x, y, z], dtype=np.float32)
        norm = np.linalg.norm(quat)
        return quat / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    except Exception:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

def broadcast_vr_data():
    """Broadcast VR data to any listening relay scripts - OPTIMIZED FOR LOW LATENCY"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # Increase send buffer
    
    print(f"📡 Starting LOW-LATENCY VR broadcast ({BROADCAST_RATE}Hz) to {g_config['broadcast_address']}:{g_config['broadcast_port']}...")
    packet_count = 0
    last_debug = time.time()
    
    # Pre-allocate packet structure for efficiency
    packet_template = {
        'mode': 'vr_data',
        'position': [0.0, 0.0, 0.0],
        'orientation': [1.0, 0.0, 0.0, 0.0],
        'button_states': {},
        'trigger_value': 0.0,
        'timestamp': 0.0
    }
    
    loop_period = 1.0 / BROADCAST_RATE
    
    while True:
        loop_start = time.time()
        
        should_send = False
        try:
            # Minimize lock time - copy data quickly
            with g_lock:
                if g_vr_data['connected']:
                    # Fast copy using pre-allocated structure
                    packet_template['position'] = g_vr_data['position'].tolist()
                    packet_template['orientation'] = g_vr_data['orientation'].tolist()
                    packet_template['button_states'] = g_vr_data['button_states'].copy()
                    packet_template['trigger_value'] = g_vr_data['trigger_value']
                    packet_template['timestamp'] = time.time()
                    
                    should_send = True
            
            if should_send:
                # Serialize and send outside of lock for better concurrency
                data = pickle.dumps(packet_template)
                sock.sendto(data, (g_config['broadcast_address'], g_config['broadcast_port']))
                packet_count += 1
            
            # Lightweight performance monitoring
            now = time.time()
            if now - last_debug > 10.0:  # Every 10 seconds
                elapsed = now - last_debug
                rate = packet_count / elapsed if elapsed > 0 else 0
                
                if rate < BROADCAST_RATE * 0.9:  # Less than 90% of target
                    print(f"⚠️ VR: {rate:.0f}Hz (target: {BROADCAST_RATE}Hz)")
                elif packet_count > 0:
                    print(f"📡 VR: {rate:.0f}Hz OK")
                
                packet_count = 0
                last_debug = now

            # Precise timing control
            elapsed = time.time() - loop_start
            sleep_time = max(0, loop_period - elapsed)
            
            if sleep_time > 0:
                time.sleep(sleep_time)
                
        except Exception as e:
            print(f"Broadcast error: {e}")
            time.sleep(0.001)  # Minimal recovery delay

class VideoFrameAssembler:
    """Assembles fragmented video frames from UDP"""
    
    def __init__(self):
        self.chunks = {}  # frame_id -> {chunk_index: data}
        self.lock = threading.Lock()
    
    def add_chunk(self, total_chunks, chunk_index, data):
        """Add chunk and return complete frame if ready"""
        frame_id = int(time.time() * 1000) // 100  # Group by 100ms windows
        
        with self.lock:
            if frame_id not in self.chunks:
                self.chunks[frame_id] = {}
            
            self.chunks[frame_id][chunk_index] = data
            
            # Check if frame is complete
            if len(self.chunks[frame_id]) == total_chunks:
                # Assemble complete frame
                complete_data = b''.join(
                    self.chunks[frame_id][i] for i in range(total_chunks)
                )
                
                # Clean up old frames
                old_frames = [fid for fid in self.chunks.keys() if fid < frame_id - 5]
                for old_fid in old_frames:
                    del self.chunks[old_fid]
                
                del self.chunks[frame_id]
                return complete_data
        
        return None

def receive_360_video():
    """Receive equirectangular video frames via UDP"""
    print(f"🎥 Starting 360° video receiver on UDP port {g_config['video_port']}")
    
    assembler = VideoFrameAssembler()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', g_config['video_port']))
    sock.settimeout(1.0)
    
    frames_received = 0
    last_stats = time.time()
    
    while True:
        try:
            data, addr = sock.recvfrom(65536)  # Large buffer for video data
            
            # Parse header
            header_size = struct.calcsize('!III')
            if len(data) < header_size:
                continue
                
            total_chunks, chunk_index, chunk_size = struct.unpack('!III', data[:header_size])
            chunk_data = data[header_size:header_size + chunk_size]
            
            # Assemble frame
            complete_frame = assembler.add_chunk(total_chunks, chunk_index, chunk_data)
            
            if complete_frame:
                try:
                    # Parse JSON frame data
                    frame_json = json.loads(complete_frame.decode('utf-8'))
                    
                    if frame_json.get('type') == 'equirectangular_frame':
                        # Decode base64 image data
                        img_data = base64.b64decode(frame_json['data'])
                        
                        # Update global 360 data
                        with g_360_lock:
                            g_360_data['current_frame'] = img_data
                            g_360_data['frame_timestamp'] = frame_json['timestamp']
                            g_360_data['frame_width'] = frame_json['width']
                            g_360_data['frame_height'] = frame_json['height']
                            g_360_data['frames_received'] += 1
                            
                            # Create data URL for Vuer
                            import base64
                            b64_str = base64.b64encode(img_data).decode('utf-8')
                            g_360_data['texture_url'] = f"data:image/jpeg;base64,{b64_str}"
                        
                        frames_received += 1
                        
                        # Stats
                        now = time.time()
                        if now - last_stats > 10.0:
                            fps = frames_received / (now - last_stats)
                            print(f"🎥 360° Video: {fps:.1f} FPS, {frame_json['width']}x{frame_json['height']}")
                            frames_received = 0
                            last_stats = now
                
                except Exception as e:
                    print(f"Error processing 360° frame: {e}")
        
        except socket.timeout:
            continue
        except Exception as e:
            print(f"Error receiving 360° video: {e}")

async def update_360_sphere():
    """Update the 360° video sphere texture in Vuer"""
    global g_vuer_session
    
    if not g_vuer_session:
        await asyncio.sleep(1)
        return
    
    last_update = 0
    update_interval = 1.0 / VIDEO_TEXTURE_UPDATE_RATE
    
    while True:
        try:
            now = time.time()
            
            # Check if we have a new frame to update
            with g_360_lock:
                frame_timestamp = g_360_data['frame_timestamp']
                texture_url = g_360_data['texture_url']
                
            if texture_url and frame_timestamp > last_update:
                # Update the sphere texture
                g_vuer_session.upsert @ Sphere(
                    key="360_video_sphere",
                    args=[SPHERE_RADIUS, 32, 32],
                    materialType="standard",
                    material={
                        "map": texture_url,
                        "side": 1  # Render inside the sphere
                    },
                    position=[0, 0, 0],
                    rotation=[0.5 * np.pi, 0, 0],  # Rotate for proper orientation
                )
                
                last_update = frame_timestamp
            
            await asyncio.sleep(update_interval)
            
        except Exception as e:
            print(f"Error updating 360° sphere: {e}")
            await asyncio.sleep(1)

def setup_vr_server():
    """Setup VR server with Vuer - OPTIMIZED FOR LOW LATENCY + 360 VIDEO"""
    print("🎮 Starting LOW-LATENCY VR Server with 360° Video...")
    print("📋 Setup Instructions:")
    print(f"  1. Make sure ngrok is running: ngrok http {g_config['vr_port']}")
    print("  2. Access the ngrok URL from Meta Quest Browser")
    print("  3. Move left controller to start data streaming")
    print("  4. Start camera streamer and fisheye converter for 360° view")
    
    # Optimized Vuer configuration for minimal latency
    app = Vuer(
        host='0.0.0.0', 
        port=g_config['vr_port'], 
        queries=dict(grid=False), 
        queue_len=QUEUE_LEN  # Reduced queue length for lower latency
    )
    
    @app.add_handler("CONTROLLER_MOVE")
    async def controller_handler(event, session):
        global g_vr_data
        
        data = event.value
        
        # Minimize lock time and processing (same as original)
        if 'left' in data and isinstance(data['left'], (list, tuple)) and len(data['left']) == 16:
            try:
                # Fast matrix operations
                transform_matrix = np.array(data['left'], dtype=np.float32).reshape(4, 4, order='F')
                position = transform_matrix[:3, 3].copy()
                rotation_matrix = transform_matrix[:3, :3]
                orientation = rotation_matrix_to_quaternion(rotation_matrix)
                
                # Fast lock acquisition and update
                with g_lock:
                    # Update VR data
                    g_vr_data['position'][:] = position  # In-place update
                    g_vr_data['orientation'][:] = orientation  # In-place update
                    g_vr_data['connected'] = True
                    g_vr_data['last_update'] = time.time()
                    g_vr_data['update_count'] += 1
                    
                    # Process button states efficiently
                    if 'leftState' in data:
                        state = data['leftState']
                        trigger_val = state.get('triggerValue', 0.0)
                        g_vr_data['trigger_value'] = trigger_val
                        squeeze_pressed = state.get('squeeze', False)
                        
                        # SWAPPED CONTROLS: trigger=gripper, squeeze=movement, a_button=reset
                        buttons = g_vr_data['button_states']
                        old_buttons = buttons.copy()  # Save previous state for change detection
                        
                        buttons['trigger'] = trigger_val > TRIGGER_THRESHOLD  # Now gripper control
                        buttons['squeeze'] = squeeze_pressed  # Now movement enable  
                        buttons['a_button'] = state.get('aButton', False)  # Now reset button
                        buttons['b_button'] = state.get('bButton', False)
                        buttons['menu'] = state.get('menu', False)
                        buttons['thumbstick'] = state.get('thumbstick', False)
                        
                        # Smart feedback: Show control changes
                        if squeeze_pressed and not old_buttons.get('squeeze', False):
                            print(f"🎯 MOVEMENT ENABLED: Squeeze pressed - Move controller to control robot", flush=True)
                        elif not squeeze_pressed and old_buttons.get('squeeze', False):
                            print(f"🎯 MOVEMENT DISABLED: Squeeze released - Robot stopped", flush=True)
                        
                        if buttons['trigger'] and not old_buttons.get('trigger', False):
                            print(f"🤏 GRIPPER CLOSE: Trigger {trigger_val:.2f} - Close gripper", flush=True)
                        elif not buttons['trigger'] and old_buttons.get('trigger', False):
                            print(f"🤏 GRIPPER OPEN: Trigger released - Open gripper", flush=True)
                            
                        if buttons['a_button'] and not old_buttons.get('a_button', False):
                            print(f"🔄 RESET BUTTON (A) PRESSED - Robot will move to neutral", flush=True)
            
            except Exception as e:
                print(f"VR processing error: {e}")
    
    @app.spawn(start=True)
    async def main_vr(session: VuerSession):
        global g_vuer_session
        g_vuer_session = session
        
        print("🚀 Setting up Motion Controllers and 360° Environment...")
        
        # Setup the scene with a 360° video sphere
        session.set @ DefaultScene(
            # Initial 360° video sphere (will be updated with real video)
            Sphere(
                key="360_video_sphere",
                args=[SPHERE_RADIUS, 32, 32],
                materialType="standard",
                material={
                    "color": 0x222222,  # Dark gray placeholder
                    "side": 1  # Render inside the sphere
                },
                position=[0, 0, 0],
                rotation=[0.5 * np.pi, 0, 0],
            ),
        )
        
        # Setup motion controllers
        session.upsert @ MotionControllers(
            stream=True,
            key="motion-controller",
            left=True,
            right=False
        )
        
        print("✅ LOW-LATENCY VR Motion Controller activated!")
        print(f"📱 Broadcasting VR data at {BROADCAST_RATE}Hz to {g_config['broadcast_address']}:{g_config['broadcast_port']}")
        print(f"🎥 360° video sphere ready for live streaming")
        
        # Start 360° video texture update loop
        asyncio.create_task(update_360_sphere())
        
        while True:
            await sleep(1)
    
    return app

def main():
    """Main function with argument parsing"""
    global BROADCAST_RATE, DEBUG_PRINTS, VIDEO_TEXTURE_UPDATE_RATE
    
    parser = argparse.ArgumentParser(description='LOW-LATENCY VR Server with 360° Video for PiperX Teleoperation')
    parser.add_argument('--vr-port', type=int, default=DEFAULT_VR_PORT,
                        help=f'Port for VR server (default: {DEFAULT_VR_PORT})')
    parser.add_argument('--broadcast-port', type=int, default=DEFAULT_BROADCAST_PORT,
                        help=f'Port for broadcasting VR data (default: {DEFAULT_BROADCAST_PORT})')
    parser.add_argument('--broadcast-address', default=DEFAULT_BROADCAST_ADDRESS,
                        help=f'Address for broadcasting VR data (default: {DEFAULT_BROADCAST_ADDRESS})')
    parser.add_argument('--video-port', type=int, default=VIDEO_UDP_PORT,
                        help=f'Port for receiving 360° video (default: {VIDEO_UDP_PORT})')
    parser.add_argument('--broadcast-rate', type=int, default=BROADCAST_RATE,
                        help=f'Broadcast rate in Hz (default: {BROADCAST_RATE})')
    parser.add_argument('--video-fps', type=int, default=VIDEO_TEXTURE_UPDATE_RATE,
                        help=f'360° video update rate in FPS (default: {VIDEO_TEXTURE_UPDATE_RATE})')
    parser.add_argument('--sphere-radius', type=float, default=SPHERE_RADIUS,
                        help=f'360° video sphere radius (default: {SPHERE_RADIUS})')
    parser.add_argument('--debug-prints', action='store_true',
                        help='Enable debug print statements (may cause latency)')
    args = parser.parse_args()
    
    # Update global configuration
    BROADCAST_RATE = args.broadcast_rate
    DEBUG_PRINTS = args.debug_prints
    VIDEO_TEXTURE_UPDATE_RATE = args.video_fps
    
    g_config.update({
        'vr_port': args.vr_port,
        'broadcast_port': args.broadcast_port,
        'broadcast_address': args.broadcast_address,
        'video_port': args.video_port
    })
    
    print("🌟 LOW-LATENCY VR Server with 360° Video Starting...")
    print(f"📡 VR Server Port: {g_config['vr_port']}")
    print(f"📤 VR Broadcast: {g_config['broadcast_address']}:{g_config['broadcast_port']} @ {BROADCAST_RATE}Hz")
    print(f"🎥 360° Video Port: {g_config['video_port']} @ {VIDEO_TEXTURE_UPDATE_RATE}FPS")
    print(f"⚡ Queue Length: {QUEUE_LEN} (minimal buffering)")
    print(f"🔧 Debug Prints: {'ON' if DEBUG_PRINTS else 'OFF (use --debug-prints to enable)'}")
    
    # Start video receiver thread
    video_thread = threading.Thread(target=receive_360_video, daemon=True)
    video_thread.start()
    
    # Start VR data broadcast thread with high priority
    broadcast_thread = threading.Thread(target=broadcast_vr_data, daemon=True)
    broadcast_thread.start()
    
    # Setup and run VR server with 360° video
    vr_app = setup_vr_server()
    
    try:
        print("🎮 LOW-LATENCY VR Server with 360° Video running. Press Ctrl+C to stop.")
        print("💡 To enable 360° video:")
        print("   1. Run: ./attempt2_insta360_vuer_streamer")
        print("   2. Run: python attempt2_fisheye_to_equirect.py")
        print("   3. The 360° video will appear in your VR view!")
        vr_app.run()
    except KeyboardInterrupt:
        print("\n🛑 VR Server with 360° Video stopped")

if __name__ == '__main__':
    main()