#!/usr/bin/env python3
"""
Dual Arm VR Server with Integrated Video - Handles both left and right VR controllers
Runs Vuer with WebRTC video streaming and broadcasts VR controller data for both arms
OPTIMIZED FOR LOW LATENCY TELEOPERATION
"""

import sys
import time
import numpy as np
import socket
import pickle
import threading
import argparse
from asyncio import sleep
import asyncio
import json
import os
import platform
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender

# Add the parent directory to the path
sys.path.append('..')
from xemb_scripts.cartesian_control.transformations import rotation_matrix_to_quaternion

try:
    from vuer import Vuer, VuerSession
    from vuer.schemas import MotionControllers, Scene, WebRTCVideoPlane, Movable
except ImportError:
    print("Error: vuer not found. Please install with: pip install vuer")
    sys.exit(1)

# Default Configuration
DEFAULT_VR_PORT = 8012
DEFAULT_BROADCAST_PORT = 5006
DEFAULT_BROADCAST_ADDRESS = "127.0.0.1"  # Use localhost for consistency with relay
DEFAULT_WEBCAM_DEVICE = "/dev/video0"  # Insta360 X3 or primary camera
TRIGGER_THRESHOLD = 0.1

# Performance optimizations
BROADCAST_RATE = 60  # Optimal for VR (per user feedback)
QUEUE_LEN = 1  # Minimal buffering for lowest latency
DEBUG_PRINTS = False  # Disable prints for testing latency issues

# Global VR data - optimized structure for DUAL ARMS
g_vr_data = {
    'left': {
        'position': np.zeros(3, dtype=np.float32),
        'orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        'trigger_value': 0.0,
        'joystick': np.zeros(2, dtype=np.float32),  # [X, Y] joystick/thumbstick axes
        'button_states': {
            'trigger': False,
            'a_button': False,
            'b_button': False,
            'squeeze': False,
            'menu': False,
            'thumbstick': False,
            'x_button': False,
            'y_button': False
        },
        'connected': False,
        'last_update': time.time(),
        'update_count': 0
    },
    'right': {
        'position': np.zeros(3, dtype=np.float32),
        'orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        'trigger_value': 0.0,
        'joystick': np.zeros(2, dtype=np.float32),  # [X, Y] joystick/thumbstick axes
        'button_states': {
            'trigger': False,
            'a_button': False,
            'b_button': False,
            'squeeze': False,
            'menu': False,
            'thumbstick': False,
            'x_button': False,
            'y_button': False
        },
        'connected': False,
        'last_update': time.time(),
        'update_count': 0
    }
}
g_lock = threading.RLock()  # Use RLock for better performance

# Global configuration
g_config = {
    'vr_port': DEFAULT_VR_PORT,
    'broadcast_port': DEFAULT_BROADCAST_PORT,
    'broadcast_address': DEFAULT_BROADCAST_ADDRESS,
    'webcam_device': DEFAULT_WEBCAM_DEVICE,
    'show_controller': False
}

# WebRTC globals
relay = None
webcam = None
pcs = set()

def create_local_tracks(device_name):
    """Create webcam tracks for WebRTC"""
    global relay, webcam
   
    if relay is None:
        # Optimized options for low latency
        options = {
            "framerate": "30",
            "video_size": "640x480",
            "fflags": "nobuffer",
            "flags": "low_delay", 
            "probesize": "32",
            "analyzeduration": "0",
        }
       
        if platform.system() == "Windows":
            format = "dshow"
            print(f">>> Setting up webcam: {device_name} with format: {format}")
            try:
                webcam = MediaPlayer(f"video={device_name}", format=format, options=options)
                print(f">>> Webcam created successfully")
            except Exception as e:
                print(f">>> Failed to create webcam: {e}")
                raise
        else:
            # Linux/Mac handling
            format = "v4l2" if platform.system() == "Linux" else "avfoundation"
            print(f">>> Setting up camera: {device_name} with format: {format}")
            try:
                # For Linux, device_name should be like /dev/video0
                webcam = MediaPlayer(device_name, format=format, options=options)
                print(f">>> Camera initialized successfully from {device_name}")
            except Exception as e:
                print(f">>> Failed to create camera from {device_name}: {e}")
                # Try alternative device if primary fails
                if device_name == "/dev/video0":
                    print(f">>> Trying alternative device /dev/video1...")
                    try:
                        webcam = MediaPlayer("/dev/video1", format=format, options=options)
                        print(f">>> Camera initialized from /dev/video1")
                    except:
                        raise e
                else:
                    raise
       
        relay = MediaRelay()
   
    return None, relay.subscribe(webcam.video)

def force_codec(pc, sender, forced_codec):
    """Force specific codec for WebRTC"""
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )

async def webrtc_offer(request):
    """Handle WebRTC offer for video streaming"""
    print(f">>> WebRTC offer received")
   
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)
    print(f">>> New WebRTC connection, total: {len(pcs)}")

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f">>> WebRTC connection state: {pc.connectionState}")
        if pc.connectionState in ["failed", "closed"]:
            pcs.discard(pc)
            print(f">>> WebRTC connection cleaned up, remaining: {len(pcs)}")

    # Add webcam video track
    audio, video = create_local_tracks(g_config['webcam_device'])
    if video:
        pc.addTrack(video)
        print(f">>> Added video track from: {g_config['webcam_device']}")

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })
    )

def broadcast_vr_data():
    """Broadcast VR data for BOTH controllers to any listening relay scripts - OPTIMIZED FOR LOW LATENCY"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Remove broadcast option since we're using localhost
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # Increase send buffer
    
    print(f">>> Starting DUAL-ARM LOW-LATENCY VR broadcast ({BROADCAST_RATE}Hz) to {g_config['broadcast_address']}:{g_config['broadcast_port']}...")
    packet_count = 0
    last_debug = time.time()
    
    # Pre-allocate combined packet structure for efficiency
    combined_packet_template = {
        'mode': 'vr_dual_data',
        'timestamp': 0.0,
        'left': None,
        'right': None
    }
    
    loop_period = 1.0 / BROADCAST_RATE
    
    while True:
        loop_start = time.time()
        
        try:
            # Minimize lock time - copy data quickly for both controllers into single packet
            with g_lock:
                current_timestamp = time.time()
                combined_packet_template['timestamp'] = current_timestamp
                
                # Process LEFT controller
                if g_vr_data['left']['connected']:
                    combined_packet_template['left'] = {
                        'position': g_vr_data['left']['position'].tolist(),
                        'orientation': g_vr_data['left']['orientation'].tolist(),
                        'button_states': g_vr_data['left']['button_states'].copy(),
                        'trigger_value': g_vr_data['left']['trigger_value'],
                        'joystick': g_vr_data['left']['joystick'].tolist(),
                        'robot_id': 0,
                        'connected': True
                    }
                else:
                    combined_packet_template['left'] = None
                
                # Process RIGHT controller
                if g_vr_data['right']['connected']:
                    combined_packet_template['right'] = {
                        'position': g_vr_data['right']['position'].tolist(),
                        'orientation': g_vr_data['right']['orientation'].tolist(),
                        'button_states': g_vr_data['right']['button_states'].copy(),
                        'trigger_value': g_vr_data['right']['trigger_value'],
                        'joystick': g_vr_data['right']['joystick'].tolist(),
                        'robot_id': 1,
                        'connected': True
                    }
                else:
                    combined_packet_template['right'] = None
            
            # Send single combined packet outside of lock
            if combined_packet_template['left'] or combined_packet_template['right']:
                data = pickle.dumps(combined_packet_template.copy())
                sock.sendto(data, (g_config['broadcast_address'], g_config['broadcast_port']))
                packet_count += 1
            
            # Lightweight performance monitoring
            now = time.time()
            if now - last_debug > 10.0:  # Every 10 seconds
                elapsed = now - last_debug
                rate = packet_count / elapsed if elapsed > 0 else 0
                
                # Count active controllers
                with g_lock:
                    active_controllers = sum(1 for ctrl in ['left', 'right'] if g_vr_data[ctrl]['connected'])
                
                # Now we send 1 packet per cycle regardless of active controllers
                target_rate = BROADCAST_RATE
                if rate < target_rate * 0.9:  # Less than 90% of target
                    print(f">>> WARNING: DUAL VR COMBINED: {rate:.0f}Hz (target: {target_rate}Hz, active: {active_controllers})")
                elif packet_count > 0:
                    print(f">>> DUAL VR COMBINED: {rate:.0f}Hz OK (active controllers: {active_controllers})")
                
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

def setup_vr_server():
    """Setup VR server with Vuer - DUAL ARM CONTROLLER SUPPORT WITH VIDEO"""
    print(">>> Starting DUAL-ARM LOW-LATENCY VR Server with Integrated Video...")
    print(">>> Setup Instructions:")
    print(f"  1. Run: ngrok http {g_config['vr_port']}")
    print("  2. Access the ngrok URL from Meta Quest Browser")
    print("  3. Move both left AND right controllers to start data streaming")
    print("  4. Left controller → Robot 0 (Left arm)")
    print("  5. Right controller → Robot 1 (Right arm)")
    print(f"  6. Camera device: {g_config['webcam_device']}")
    
    # Optimized Vuer configuration for minimal latency
    vuer_app = Vuer(
        host='0.0.0.0', 
        port=g_config['vr_port'], 
        queries=dict(grid=False), 
        queue_len=QUEUE_LEN  # Reduced queue length for lower latency
    )
    
    # Add WebRTC routes to the Vuer app's internal web server
    def add_webrtc_routes():
        """Add WebRTC endpoints to Vuer's web server"""
        # Get the internal aiohttp app from Vuer
        web_app = vuer_app.app
       
        # Just add the routes directly - Vuer handles CORS
        web_app.router.add_post("/offer", webrtc_offer)
        # Don't add OPTIONS handler - Vuer already handles it
        print(">>> Added WebRTC routes to VR server")
   
    # Add routes after Vuer app is created
    add_webrtc_routes()
    
    @vuer_app.add_handler("CONTROLLER_MOVE")
    async def controller_handler(event, session):
        global g_vr_data
        
        data = event.value
        current_time = time.time()
        
        # Process LEFT controller
        if 'left' in data and isinstance(data['left'], (list, tuple)) and len(data['left']) == 16:
            try:
                # Fast matrix operations
                transform_matrix = np.array(data['left'], dtype=np.float32).reshape(4, 4, order='F')
                position = transform_matrix[:3, 3].copy()
                rotation_matrix = transform_matrix[:3, :3]
                orientation = rotation_matrix_to_quaternion(rotation_matrix)
                
                # Fast lock acquisition and update
                with g_lock:
                    # Update LEFT VR data
                    g_vr_data['left']['position'][:] = position  # In-place update
                    g_vr_data['left']['orientation'][:] = orientation  # In-place update
                    g_vr_data['left']['connected'] = True
                    g_vr_data['left']['last_update'] = current_time
                    g_vr_data['left']['update_count'] += 1
                    
                    # Process LEFT button states efficiently
                    if 'leftState' in data:
                        state = data['leftState']
                        trigger_val = state.get('triggerValue', 0.0)
                        g_vr_data['left']['trigger_value'] = trigger_val
                        squeeze_pressed = state.get('squeeze', False)
                        
                        # Extract joystick/thumbstick axis data for base control
                        # Try different possible field names for joystick data
                        joystick_x = state.get('thumbstickX', state.get('joystickX', state.get('axisX', 0.0)))
                        joystick_y = state.get('thumbstickY', state.get('joystickY', state.get('axisY', 0.0)))
                        
                        # Alternative: try to get joystick data as an array
                        if joystick_x == 0.0 and joystick_y == 0.0:
                            joystick_data = state.get('joystick', state.get('thumbstick_axis', [0.0, 0.0]))
                            if isinstance(joystick_data, (list, tuple)) and len(joystick_data) >= 2:
                                joystick_x = joystick_data[0]
                                joystick_y = joystick_data[1]
                        
                        g_vr_data['left']['joystick'][0] = joystick_x
                        g_vr_data['left']['joystick'][1] = joystick_y
                        
                        # CONTROLS: trigger=gripper, squeeze=movement, x_button=reset, joystick=base_movement
                        buttons = g_vr_data['left']['button_states']
                        old_buttons = buttons.copy()  # Save previous state for change detection
                        
                        buttons['trigger'] = trigger_val > TRIGGER_THRESHOLD  # Gripper control
                        buttons['squeeze'] = squeeze_pressed  # Movement enable  
                        buttons['x_button'] = state.get('bButton', False)  # X button (reset) - LEFT controller uses bButton
                        buttons['a_button'] = state.get('aButton', False)  # A button (available)
                        buttons['y_button'] = False  # Available for future use
                        buttons['b_button'] = False  # Available for future use
                        buttons['menu'] = state.get('menu', False)
                        buttons['thumbstick'] = state.get('thumbstick', False)
                        
                        # Smart feedback: Show control changes for LEFT (if enabled)
                        show_feedback = g_config.get('show_controller', False)
                        if show_feedback:
                            if squeeze_pressed and not old_buttons.get('squeeze', False):
                                print(f">>> LEFT MOVEMENT ENABLED: Squeeze pressed - Move left controller to control left robot", flush=True)
                            elif not squeeze_pressed and old_buttons.get('squeeze', False):
                                print(f">>> LEFT MOVEMENT DISABLED: Squeeze released - Left robot stopped", flush=True)
                            
                            if buttons['trigger'] and not old_buttons.get('trigger', False):
                                print(f">>> LEFT GRIPPER CLOSE: Trigger {trigger_val:.2f} - Close left gripper", flush=True)
                            elif not buttons['trigger'] and old_buttons.get('trigger', False):
                                print(f">>> LEFT GRIPPER OPEN: Trigger released - Open left gripper", flush=True)
                                
                            if buttons['x_button'] and not old_buttons.get('x_button', False):
                                print(f">>> LEFT RESET BUTTON (X) PRESSED - Left robot will move to neutral", flush=True)
            
            except Exception as e:
                print(f"LEFT VR processing error: {e}")
        
        # Process RIGHT controller
        if 'right' in data and isinstance(data['right'], (list, tuple)) and len(data['right']) == 16:
            try:
                # Fast matrix operations
                transform_matrix = np.array(data['right'], dtype=np.float32).reshape(4, 4, order='F')
                position = transform_matrix[:3, 3].copy()
                rotation_matrix = transform_matrix[:3, :3]
                orientation = rotation_matrix_to_quaternion(rotation_matrix)
                
                # Fast lock acquisition and update
                with g_lock:
                    # Update RIGHT VR data
                    g_vr_data['right']['position'][:] = position  # In-place update
                    g_vr_data['right']['orientation'][:] = orientation  # In-place update
                    g_vr_data['right']['connected'] = True
                    g_vr_data['right']['last_update'] = current_time
                    g_vr_data['right']['update_count'] += 1
                    
                    # Process RIGHT button states efficiently
                    if 'rightState' in data:
                        state = data['rightState']
                        trigger_val = state.get('triggerValue', 0.0)
                        g_vr_data['right']['trigger_value'] = trigger_val
                        squeeze_pressed = state.get('squeeze', False)
                        
                        # Extract joystick/thumbstick axis data for base control
                        # Try different possible field names for joystick data
                        joystick_x = state.get('thumbstickX', state.get('joystickX', state.get('axisX', 0.0)))
                        joystick_y = state.get('thumbstickY', state.get('joystickY', state.get('axisY', 0.0)))
                        
                        # Alternative: try to get joystick data as an array
                        if joystick_x == 0.0 and joystick_y == 0.0:
                            joystick_data = state.get('joystick', state.get('thumbstick_axis', [0.0, 0.0]))
                            if isinstance(joystick_data, (list, tuple)) and len(joystick_data) >= 2:
                                joystick_x = joystick_data[0]
                                joystick_y = joystick_data[1]
                        
                        g_vr_data['right']['joystick'][0] = joystick_x
                        g_vr_data['right']['joystick'][1] = joystick_y
                        
                        # CONTROLS: trigger=gripper, squeeze=movement, a_button=reset, joystick=base_movement
                        buttons = g_vr_data['right']['button_states']
                        old_buttons = buttons.copy()  # Save previous state for change detection
                        
                        buttons['trigger'] = trigger_val > TRIGGER_THRESHOLD  # Gripper control
                        buttons['squeeze'] = squeeze_pressed  # Movement enable  
                        buttons['a_button'] = state.get('aButton', False)  # A button (reset) - RIGHT controller uses aButton
                        buttons['x_button'] = state.get('bButton', False)  # X button (available)
                        buttons['y_button'] = False  # Available for future use
                        buttons['b_button'] = False  # Available for future use
                        buttons['menu'] = state.get('menu', False)
                        buttons['thumbstick'] = state.get('thumbstick', False)
                        
                        # Smart feedback: Show control changes for RIGHT (if enabled)
                        show_feedback = g_config.get('show_controller', False)
                        if show_feedback:
                            if squeeze_pressed and not old_buttons.get('squeeze', False):
                                print(f">>> RIGHT MOVEMENT ENABLED: Squeeze pressed - Move right controller to control right robot", flush=True)
                            elif not squeeze_pressed and old_buttons.get('squeeze', False):
                                print(f">>> RIGHT MOVEMENT DISABLED: Squeeze released - Right robot stopped", flush=True)
                            
                            if buttons['trigger'] and not old_buttons.get('trigger', False):
                                print(f">>> RIGHT GRIPPER CLOSE: Trigger {trigger_val:.2f} - Close right gripper", flush=True)
                            elif not buttons['trigger'] and old_buttons.get('trigger', False):
                                print(f">>> RIGHT GRIPPER OPEN: Trigger released - Open right gripper", flush=True)
                                
                            if buttons['a_button'] and not old_buttons.get('a_button', False):
                                print(f">>> RIGHT RESET BUTTON (A) PRESSED - Right robot will move to neutral", flush=True)
            
            except Exception as e:
                print(f"RIGHT VR processing error: {e}")
    
    @vuer_app.spawn(start=True)
    async def main_vr(session: VuerSession):
        print(">>> Setting up DUAL Motion Controllers with Video...")
        
        # Create video plane that connects to our integrated WebRTC server
        # Use relative URL so it works with both HTTP and HTTPS
        video_plane = WebRTCVideoPlane(
            src="/offer",  # Relative URL - works with any protocol
            key="robot-video",
            position=[0, 1.5, -1.2],  # Closer for teleoperation
            aspect=16/9,
            height=2.5  # Larger for better visibility during teleoperation
        )
       
        # Make video movable so user can reposition it
        movable_video = Movable(
            video_plane,
            key="robot-camera",
            position=[0, 1.5, -1.2],  # Match video plane position
            handle=[0.15, 0.08, 0.03]  # Slightly larger handles for easier manipulation
        )
        
        # Set up the scene with video and dual motion controllers
        session.set @ Scene(
            movable_video,
            frameloop='always',
            bgChildren=[
                MotionControllers(
                    stream=True,
                    key="motion-controller",
                    left=True,   # Enable LEFT controller
                    right=True   # Enable RIGHT controller 
                )
            ]
        )
        
        print(">>> DUAL-ARM LOW-LATENCY VR Motion Controllers activated with Video!")
        print(f">>> Broadcasting LEFT & RIGHT at {BROADCAST_RATE}Hz to {g_config['broadcast_address']}:{g_config['broadcast_port']}")
        print(">>> Left Controller → Robot 0 (Left Arm)")
        print(">>> Right Controller → Robot 1 (Right Arm)")
        print(f">>> Video streaming from camera: {g_config['webcam_device']}")
        print(f">>> Both VR and video on port: {g_config['vr_port']}")
        
        while True:
            await sleep(1)
    
    return vuer_app

def main():
    """Main function with argument parsing"""
    global BROADCAST_RATE, DEBUG_PRINTS
    
    parser = argparse.ArgumentParser(description='DUAL-ARM LOW-LATENCY VR Server with Video for PiperX Teleoperation')
    parser.add_argument('--vr-port', type=int, default=DEFAULT_VR_PORT,
                        help=f'Port for VR server (default: {DEFAULT_VR_PORT})')
    parser.add_argument('--broadcast-port', type=int, default=DEFAULT_BROADCAST_PORT,
                        help=f'Port for broadcasting VR data (default: {DEFAULT_BROADCAST_PORT})')
    parser.add_argument('--broadcast-address', default=DEFAULT_BROADCAST_ADDRESS,
                        help=f'Address for broadcasting VR data (default: {DEFAULT_BROADCAST_ADDRESS})')
    parser.add_argument('--broadcast-rate', type=int, default=BROADCAST_RATE,
                        help=f'Broadcast rate in Hz (default: {BROADCAST_RATE})')
    parser.add_argument('--webcam-device', default=DEFAULT_WEBCAM_DEVICE,
                        help=f'Camera device path (default: {DEFAULT_WEBCAM_DEVICE})')
    parser.add_argument('--debug-prints', action='store_true',
                        help='Enable debug print statements (may cause latency)')
    parser.add_argument('--show-controller', action='store_true',
                        help='Show controller input feedback')
    args = parser.parse_args()
    
    # Update global configuration
    BROADCAST_RATE = args.broadcast_rate
    DEBUG_PRINTS = args.debug_prints
    g_config.update({
        'vr_port': args.vr_port,
        'broadcast_port': args.broadcast_port,
        'broadcast_address': args.broadcast_address,
        'webcam_device': args.webcam_device,
        'show_controller': args.show_controller
    })
    
    print("*** DUAL-ARM LOW-LATENCY VR Server with Integrated Video Starting...")
    print(f">>> VR Server Port: {g_config['vr_port']} (use: ngrok http {g_config['vr_port']})")
    print(f">>> VR Broadcast: {g_config['broadcast_address']}:{g_config['broadcast_port']} @ {BROADCAST_RATE}Hz")
    print(f">>> Camera Device: {g_config['webcam_device']}")
    print(f">>> Queue Length: {QUEUE_LEN} (minimal buffering)")
    print(f">>> Debug Prints: {'ON' if DEBUG_PRINTS else 'OFF (use --debug-prints to enable)'}")
    print(f">>> Controller Feedback: {'ON' if g_config['show_controller'] else 'OFF (use --show-controller to enable)'}")
    print(">>> DUAL ARM MODE: Both left and right controllers enabled")
    
    # Start broadcast thread with high priority
    broadcast_thread = threading.Thread(target=broadcast_vr_data, daemon=True)
    broadcast_thread.start()
    
    # Setup and run VR server
    vr_app = setup_vr_server()
    
    try:
        print(">>> DUAL-ARM LOW-LATENCY VR Server with Video running!")
        print(">>> Next: Run 'ngrok http 8012' and access from Meta Quest")
        vr_app.run()
    except KeyboardInterrupt:
        print("\n>>> VR Server stopped")
    finally:
        # Cleanup WebRTC connections
        async def cleanup():
            coros = [pc.close() for pc in pcs]
            if coros:
                await asyncio.gather(*coros)
            pcs.clear()
       
        if pcs:
            asyncio.run(cleanup())
        print(">>> Cleanup complete")

if __name__ == '__main__':
    main()