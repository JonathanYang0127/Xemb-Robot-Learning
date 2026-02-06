#!/usr/bin/env python3
"""
Simple test script to verify video streaming with WebRTCVideoPlane
"""

import sys
from asyncio import sleep

try:
    from vuer import Vuer, VuerSession
    from vuer.schemas import DefaultScene, WebRTCVideoPlane
except ImportError:
    print("Error: vuer not found. Please install with: pip install vuer[all]")
    sys.exit(1)

def setup_video_test():
    """Setup a simple video streaming test."""
    print("🎬 Setting up video streaming test...")
    
    # Create Vuer app
    app = Vuer(host='0.0.0.0', port=8012, queries=dict(grid=False), queue_len=5)
    
    @app.spawn(start=True)
    async def main_video_test(session: VuerSession):
        """Main session that sets up video stream."""
        print("🚀 Setting up Video Stream...")
        
        # Set up the default scene with video only
        session.set @ DefaultScene(
            WebRTCVideoPlane(
                key="video-stream",
                position=[0, 0, -2],  # Position video plane in front of user
                scale=[1.6, 0.9, 1],  # 16:9 aspect ratio
                rotation=[0, 0, 0]     # No rotation
            )
        )
        
        print("✅ Video Stream activated!")
        print("📹 Video feed should now be streaming from OBS Virtual Camera")
        print("🌐 Access the ngrok URL to see the video")
        
        while True:
            await sleep(1)
    
    return app

def main():
    """Main function."""
    print("🎬 Video Streaming Test")
    print("📹 Testing WebRTCVideoPlane with OBS Virtual Camera")
    print("🌐 Server will run on port 8012")
    
    try:
        # Setup video test
        video_app = setup_video_test()
        
        # Start video server
        print("🚀 Starting video streaming server...")
        video_app.run()
        
    except KeyboardInterrupt:
        print("\n🛑 Video test stopped by user")
    except Exception as e:
        print(f"❌ Video test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("🔚 Video Streaming Test terminated")

if __name__ == '__main__':
    main() 