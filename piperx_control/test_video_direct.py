#!/usr/bin/env python3
"""
Test script to directly access OBS Virtual Camera and display video
"""

import sys
import cv2
import numpy as np
import base64
from asyncio import sleep
import threading
import time

try:
    from vuer import Vuer, VuerSession
    from vuer.schemas import DefaultScene, Plane, Image
except ImportError:
    print("Error: vuer not found. Please install with: pip install vuer[all]")
    sys.exit(1)

# Global variable to store the latest frame
latest_frame = None
frame_lock = threading.Lock()

def capture_video():
    """Capture video from OBS Virtual Camera in a separate thread."""
    global latest_frame
    
    print("📹 Starting video capture from OBS Virtual Camera...")
    
    # Try to open OBS Virtual Camera
    cap = cv2.VideoCapture(1)  # OBS Virtual Camera is usually index 1
    
    if not cap.isOpened():
        print("❌ Could not open OBS Virtual Camera")
        return
    
    print("✅ Successfully opened OBS Virtual Camera")
    
    while True:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                # Convert BGR to RGB
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                latest_frame = frame_rgb
        else:
            print("⚠️ Failed to read frame from OBS Virtual Camera")
            time.sleep(0.1)
    
    cap.release()

def frame_to_data_url(frame):
    """Convert OpenCV frame to data URL for display."""
    if frame is None:
        return None
    
    # Resize frame for web display
    height, width = frame.shape[:2]
    max_size = 640
    if width > max_size or height > max_size:
        scale = max_size / max(width, height)
        new_width = int(width * scale)
        new_height = int(height * scale)
        frame = cv2.resize(frame, (new_width, new_height))
    
    # Encode as JPEG
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    jpg_data = base64.b64encode(buffer).decode('utf-8')
    return f"data:image/jpeg;base64,{jpg_data}"

def setup_video_display():
    """Setup video display using captured frames."""
    print("🎬 Setting up video display...")
    
    # Create Vuer app
    app = Vuer(host='0.0.0.0', port=8012, queries=dict(grid=False), queue_len=5)
    
    @app.spawn(start=True)
    async def main_video_display(session: VuerSession):
        """Main session that displays video frames."""
        print("🚀 Setting up Video Display...")
        
        # Set up the default scene with video plane
        session.set @ DefaultScene(
            Plane(
                key="video-plane",
                position=[0, 0, -2],
                scale=[1.6, 0.9, 1],
                rotation=[0, 0, 0]
            )
        )
        
        print("✅ Video Display activated!")
        print("📹 Displaying frames from OBS Virtual Camera")
        
        # Update video frames periodically
        while True:
            with frame_lock:
                if latest_frame is not None:
                    data_url = frame_to_data_url(latest_frame)
                    if data_url:
                        # Update the plane with the video frame
                        session.upsert @ Plane(
                            key="video-plane",
                            position=[0, 0, -2],
                            scale=[1.6, 0.9, 1],
                            rotation=[0, 0, 0],
                            material={
                                "map": data_url,
                                "transparent": True
                            }
                        )
            
            await sleep(0.033)  # ~30 FPS
    
    return app

def main():
    """Main function."""
    print("🎬 Direct Video Capture Test")
    print("📹 Capturing from OBS Virtual Camera and displaying in web")
    print("🌐 Server will run on port 8012")
    
    try:
        # Start video capture in background thread
        video_thread = threading.Thread(target=capture_video, daemon=True)
        video_thread.start()
        
        # Wait a moment for video capture to start
        time.sleep(2)
        
        # Setup video display
        video_app = setup_video_display()
        
        # Start video server
        print("🚀 Starting video display server...")
        video_app.run()
        
    except KeyboardInterrupt:
        print("\n🛑 Video test stopped by user")
    except Exception as e:
        print(f"❌ Video test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("🔚 Direct Video Capture Test terminated")

if __name__ == '__main__':
    main() 