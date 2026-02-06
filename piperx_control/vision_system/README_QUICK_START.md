# 🚀 Insta360 X3 Streaming - Quick Start Guide

## The One Command You Need:

```bash
cd ~/interbotix_ws/src/xemb_aloha/piperx_control/vision_system
./start_insta360_streaming.sh
```

That's it! This script handles **everything** for you.

## What It Does:

1. ✅ Kills any old processes (no more "port in use" errors)
2. ✅ Sets up USB permissions automatically  
3. ✅ Builds the code if needed
4. ✅ Starts the camera streaming
5. ✅ Gives you a simple menu to choose viewing options

## Common Commands:

```bash
# Start everything (interactive menu)
./start_insta360_streaming.sh

# Start WebRTC for browser/internet viewing
./start_insta360_streaming.sh --webrtc

# Start direct low-latency streaming only
./start_insta360_streaming.sh --direct

# Stop everything
./start_insta360_streaming.sh --stop

# Check what's running
./start_insta360_streaming.sh --status
```

## Viewing Your Stream:

### Option 1: Browser (Best for Internet/VR)
After running with `--webrtc`, open:
```
http://localhost:8081
```

### Option 2: Direct Low Latency (Best for Local)
After starting the streamer, run:
```bash
curl -s http://localhost:8080 | ffplay -f h264 -fflags nobuffer -flags low_delay -framedrop -
```

## Troubleshooting:

### "Camera not found"
- Make sure camera is connected via USB and powered on
- Try unplugging and reconnecting the camera
- Run: `lsusb | grep -i insta` to check if it's detected

### "Port already in use"
- Just run: `./start_insta360_streaming.sh --stop`
- Then start again

### Need to share over internet?
1. Install ngrok: `snap install ngrok`
2. Run: `ngrok http 8081`
3. Share the ngrok URL

## That's It!

The `start_insta360_streaming.sh` script is designed to handle all the complexity. Just run it and follow the menu! 