# Dual Arm VR Server with Video Integration - Summary

## What Was Created
Successfully created `vr_server_dual_with_video.py` that combines:
- Dual arm VR controller support (left and right controllers)
- WebRTC video streaming capability
- Optimized teleoperation video settings

## Key Features

### 1. Dual Arm Control
- **Left Controller** → Robot 0 (Left arm)
- **Right Controller** → Robot 1 (Right arm)
- Both controllers work simultaneously
- Independent control of each arm
- Combined packet broadcasting for efficiency

### 2. Integrated Video
- **Position**: [0, 1.5, -1.2] - Closer for better visibility
- **Size**: 2.5 height - Large enough for detailed teleoperation
- **Camera**: /dev/video0 (Insta360 X3)
- **Format**: WebRTC streaming with low latency settings

### 3. Control Scheme
- **Trigger**: Gripper control
- **Squeeze**: Movement enable
- **X Button** (Left): Reset left robot
- **A Button** (Right): Reset right robot
- **Joystick**: Base movement control

## Files Created
- `/home/franka/interbotix_ws/src/xemb_aloha/piperx_control/vr_server_dual_with_video.py` - Main server
- `/home/franka/interbotix_ws/src/xemb_aloha/piperx_control/temp_vr_integration_attempt1/vr_server_dual_with_video.py` - Backup

## Current Status
✓ Server running on port 8012
✓ Dual arm controllers enabled
✓ Video streaming integrated
✓ Broadcasting at 60Hz to 127.0.0.1:5006
✓ Camera initialized from /dev/video0

## Usage

### Basic Command
```bash
# Run with default settings
python vr_server_dual_with_video.py

# With controller feedback
python vr_server_dual_with_video.py --show-controller

# Custom port
python vr_server_dual_with_video.py --vr-port 8013

# Custom camera
python vr_server_dual_with_video.py --webcam-device /dev/video1
```

### Connection Steps
1. Run the server: `python vr_server_dual_with_video.py --show-controller`
2. Start ngrok: `ngrok http 8012`
3. Open ngrok URL in Meta Quest Browser
4. Both controllers will be active for dual arm control
5. Video feed will be displayed prominently for teleoperation

## Key Improvements
- **No Emojis**: Removed all emoji characters per user preferences
- **Optimized Video**: Larger and closer video for teleoperation
- **Efficient Broadcasting**: Single combined packet for both controllers
- **Low Latency**: Maintained 60Hz broadcast rate with minimal buffering
- **Flexible Camera**: Automatic fallback to /dev/video1 if needed

## Differences from Single Arm Version
- Processes both left and right controller data
- Sends combined packets with both controller states
- Different button mappings for each controller
- Supports simultaneous control of two robot arms

## Troubleshooting

### Port Already in Use
```bash
# Kill existing servers
pkill -f "python.*vr_server"

# Or use alternative port
python vr_server_dual_with_video.py --vr-port 8013
```

### Camera Issues
```bash
# Check available cameras
ls -la /dev/video*
v4l2-ctl --list-devices

# Try alternative device
python vr_server_dual_with_video.py --webcam-device /dev/video1
```

### Performance Issues
- Ensure ngrok is running with appropriate plan
- Check network latency
- Verify camera is not being used by other applications

The dual arm server with video is now ready for teleoperation of both robot arms with visual feedback!