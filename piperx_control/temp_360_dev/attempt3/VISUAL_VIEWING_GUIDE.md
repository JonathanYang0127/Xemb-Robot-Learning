# Visual Viewing Mode - See Your Camera Output

## 🎉 **NEW FEATURE: Visual Verification**

You asked for a way to **actually SEE the camera output** instead of just CLI stats. Now you can!

## 👁️ **Visual Viewing Mode**

**Show actual video output in a window:**

```bash
# Quick visual test (30 seconds)
sudo ./run_camera_utility.sh --view

# Custom duration
sudo ./camera_manager --view-only --view-duration 60

# Short test
sudo ./camera_manager --view-only --view-duration 10
```

## 📺 **What You'll See**

- **Real-time video window** showing fisheye camera output
- **1920×960 resolution** @ 30fps
- **Window title:** "Insta360 X3 Live View"
- **Live progress updates** in terminal

## 🔧 **How It Works**

1. **Starts camera stream** and saves to temporary H.264 file
2. **Launches ffplay** to display the video in real-time
3. **Shows progress** every 5 seconds
4. **Automatically cleans up** when done

## 📊 **Sample Output**

```bash
sudo ./run_camera_utility.sh --view

👁️  STARTING VISUAL VIEWING MODE
  Duration: 30 seconds
  This will show the actual camera output in a window

🎬 Launching video viewer...
  Command: ffplay with H.264 fisheye stream
  Resolution: 1920×960 @ 30fps
  Window title: 'Insta360 X3 Live View'

👁️  Visual viewing active - you should see video window
  Close the video window or wait 30 seconds to exit

👁️  Viewing: 5/30s - Frames: 150, Data: 640KB
👁️  Viewing: 10/30s - Frames: 300, Data: 1280KB
👁️  Viewing: 15/30s - Frames: 450, Data: 1920KB

✅ VIEWING MODE COMPLETE
  Total frames: 900
  Total data: 3MB
  Stream working: YES

✅ VIEWING COMPLETE - Camera video output successful!
```

## 🚀 **Two Testing Modes Now Available**

### 1. **CLI Verification** (Quick Check)
```bash
sudo ./run_camera_utility.sh --verify
# Shows frame counts and data stats (no video window)
# Good for: Automated testing, headless systems
```

### 2. **Visual Viewing** (See Video)
```bash
sudo ./run_camera_utility.sh --view  
# Opens video window showing actual camera output
# Good for: Visual confirmation, debugging image quality
```

## 🔧 **Requirements**

- **ffplay** (comes with ffmpeg) - ✅ Already installed on your system
- **X11 display** (for video window)
- **Sudo permissions** (for camera access)

## ❓ **Troubleshooting**

### No Video Window Appears
1. **Check if you have display access:**
   ```bash
   echo $DISPLAY
   # Should show something like :0 or :1
   ```

2. **Try running locally (not over SSH):**
   ```bash
   # Run directly on the machine with monitor
   ```

3. **Check ffplay manually:**
   ```bash
   ffplay -f h264 -video_size 1920x960 /tmp/some_file.h264
   ```

### "ffplay not found" Error
```bash
sudo apt-get install ffmpeg
```

### Video Quality Issues
- The output is **raw fisheye** - it will look curved/distorted
- This is **normal** - you're seeing the actual camera data
- For 360° viewing, you'd need fisheye-to-equirectangular conversion

## 🎯 **Perfect for Development**

This visual mode is ideal for:
- **Confirming camera is working** before integration
- **Checking image quality** and exposure
- **Debugging camera positioning**
- **Showing others the camera output**
- **Verifying the stream** before processing

## 🚨 **Important Notes**

1. **Single Stream:** At 1920×960, X3 provides ONE stream (not two)
2. **Fisheye Format:** Video will look curved - this is the raw camera data
3. **Temporary Files:** Automatically cleaned up in /tmp/
4. **Auto-Exit:** Program exits cleanly after viewing period
5. **No Integration Claims:** This is a camera manager, not a complete solution

**Now you can actually SEE what your camera is doing!** 👀✨