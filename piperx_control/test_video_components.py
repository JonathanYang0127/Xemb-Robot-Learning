#!/usr/bin/env python3
"""
Test script to check Vuer video components and streaming capabilities
"""

import sys
from asyncio import sleep

try:
    from vuer import Vuer, VuerSession
    from vuer.schemas import DefaultScene, MotionControllers
    # Try to import various video components
    try:
        from vuer.schemas import WebRTCVideoPlane
        print("✅ WebRTCVideoPlane available")
    except ImportError:
        print("❌ WebRTCVideoPlane not available")
    
    try:
        from vuer.schemas import VideoPlane
        print("✅ VideoPlane available")
    except ImportError:
        print("❌ VideoPlane not available")
    
    try:
        from vuer.schemas import Video
        print("✅ Video component available")
    except ImportError:
        print("❌ Video component not available")
        
except ImportError:
    print("Error: vuer not found. Please install with: pip install 'vuer[all]'")
    sys.exit(1)

# Create a simple test app
app = Vuer(host='0.0.0.0', port=8013, queries=dict(grid=False))

@app.spawn(start=True)
async def test_video_components(session: VuerSession):
    """Test different video components."""
    print("🚀 Testing video components...")
    
    # Try different video component configurations
    try:
        session.set @ DefaultScene(
            MotionControllers(
                stream=True, 
                key="motion-controller", 
                left=True,
                right=False
            )
        )
        print("✅ Basic scene with motion controllers set up")
        
        # Try to add video component
        try:
            # Test WebRTCVideoPlane if available
            if 'WebRTCVideoPlane' in globals():
                session.upsert @ WebRTCVideoPlane(
                    key="test-video",
                    position=[0, 0, -2],
                    scale=[1.6, 0.9, 1],
                    rotation=[0, 0, 0]
                )
                print("✅ WebRTCVideoPlane added to scene")
        except Exception as e:
            print(f"❌ Error adding WebRTCVideoPlane: {e}")
        
    except Exception as e:
        print(f"❌ Error setting up scene: {e}")
    
    print("🎯 Test scene ready - check browser at http://localhost:8013")
    
    while True:
        await sleep(1)

if __name__ == '__main__':
    print("🧪 Testing Vuer video components...")
    app.run() 