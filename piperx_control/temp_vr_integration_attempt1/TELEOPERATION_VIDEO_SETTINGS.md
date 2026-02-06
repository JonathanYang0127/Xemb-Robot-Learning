# Teleoperation Video Settings - Optimized for VR Control

## Video Plane Adjustments Made

### Previous Settings (General VR)
- Position: [0, 1.5, -2] 
- Height: 1.5
- Handle size: [0.1, 0.05, 0.02]

### New Settings (Optimized for Teleoperation)
- **Position: [0, 1.5, -1.2]** - 40% closer for better visibility
- **Height: 2.5** - 67% larger for clearer view during robot control
- **Handle size: [0.15, 0.08, 0.03]** - 50% larger handles for easier repositioning

## Why These Changes

1. **Closer Position (-1.2 instead of -2)**
   - Reduces eye strain during extended teleoperation sessions
   - Makes it easier to see robot gripper details
   - Better depth perception for precise movements

2. **Larger Video (height 2.5)**
   - Provides clearer view of the workspace
   - Easier to see object details for manipulation tasks
   - Better peripheral awareness during operation

3. **Centered at Eye Level (y=1.5)**
   - Natural viewing angle reduces neck strain
   - Optimal for standing or seated VR operation
   - Direct line of sight for intuitive control

## Current Status
✓ Server running on port 8012
✓ Video plane optimized for teleoperation
✓ Insta360 X3 camera active on /dev/video0
✓ VR controller broadcasting at 60Hz

## Usage Tips

1. **Fine-tuning Position**: The video plane is movable - grab the handles to adjust
2. **Optimal Distance**: Start with default position, then adjust based on your arm reach
3. **Multiple Monitors**: You can move the video to the side if using with other displays

## Quick Commands

```bash
# Run server with teleoperation-optimized video
python vr_server_with_video_v2.py --show-controller

# If you need to adjust video size further, edit lines 362-364 in the script:
position=[0, 1.5, -1.2],  # x, y, z coordinates
height=2.5  # Make larger or smaller as needed
```

The video is now positioned optimally for teleoperation tasks!