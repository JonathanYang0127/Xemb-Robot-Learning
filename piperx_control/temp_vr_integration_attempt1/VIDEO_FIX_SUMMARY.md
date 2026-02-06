# VR Server with Video V2 - Video Fix Summary

## Issues Fixed

1. **Camera Device**: Changed from "HD Webcam" to "/dev/video0" (Linux device path)
2. **Port**: Now running on port 8012 as requested
3. **Error Handling**: Added fallback to /dev/video1 if /dev/video0 fails
4. **Camera Support**: Updated to work with Insta360 X3 or any V4L2 camera

## Key Changes Made to vr_server_with_video_v2.py

### 1. Camera Device Configuration
```python
DEFAULT_WEBCAM_DEVICE = "/dev/video0"  # Insta360 X3 or primary camera
```

### 2. Enhanced Camera Initialization
- Better error handling for Linux devices
- Automatic fallback to /dev/video1 if primary fails
- Clear debug messages showing which device is being used

### 3. Optimized Video Options
```python
options = {
    "framerate": "30",
    "video_size": "640x480",
    "fflags": "nobuffer",
    "flags": "low_delay",
    "probesize": "32",
    "analyzeduration": "0",
}
```

## Current Status

✓ Server is running on port 8012
✓ VR controller interface accessible
✓ WebRTC video endpoints integrated
✓ Camera device configured for /dev/video0

## How to Use

### Basic Usage
```bash
# Run with default settings (port 8012, /dev/video0)
python vr_server_with_video_v2.py

# With controller feedback
python vr_server_with_video_v2.py --show-controller

# Specify different camera device
python vr_server_with_video_v2.py --webcam-device /dev/video1
```

### For Insta360 X3 Camera

If you need to use the Insta360 X3 specifically:

1. **Option A: Direct V4L2 Device** (current setup)
   - Uses /dev/video0 or /dev/video1 directly
   - Works if camera is in webcam mode

2. **Option B: With Insta360 SDK** (if needed)
   ```bash
   # First, start Insta360 streaming utility
   cd vision_system
   ./start_insta360_streaming.sh
   
   # Then run VR server
   python vr_server_with_video_v2.py --webcam-device /dev/video1
   ```

## Accessing the VR Interface

1. Start ngrok: `ngrok http 8012`
2. Open the ngrok URL in Meta Quest Browser
3. The video feed will appear as a movable plane in VR
4. Use left controller for robot control

## Troubleshooting

### No Video Showing
1. Check available devices: `ls -la /dev/video*`
2. Test camera directly: `ffplay -f v4l2 -i /dev/video0`
3. Try alternative device: `--webcam-device /dev/video1`

### Camera Permission Issues
```bash
# Add user to video group
sudo usermod -a -G video $USER
# Or temporarily set permissions
sudo chmod 666 /dev/video0
```

### Insta360 X3 Not Detected
1. Check USB connection: `lsusb | grep -i insta`
2. Ensure camera is in correct mode (webcam mode)
3. Try the Insta360 streaming utility in vision_system/

## Next Steps

The server is now running with video capability on port 8012. You should:
1. Run ngrok: `ngrok http 8012`
2. Access from Meta Quest to test video streaming
3. If video doesn't appear, check which /dev/video* device your camera is on