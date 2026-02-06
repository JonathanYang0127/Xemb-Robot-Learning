# Insta360 X3 Camera Utility - Usage Guide

## Quick Start

### 1. Basic Status Check
```bash
# Check system requirements and camera detection
./run_camera_utility.sh --diagnose

# Or use direct status check (no sudo required)
make system-check
make usb-check
```

### 2. Install Dependencies (if needed)
```bash
# Automatic installation with sudo
./run_camera_utility.sh --install-deps

# Manual installation
sudo apt-get update
sudo apt-get install libusb-1.0-0-dev libudev-dev usbutils build-essential
```

### 3. Run Camera Utility
```bash
# Full run with automatic setup
sudo ./run_camera_utility.sh

# Direct run (requires manual setup)
export LD_LIBRARY_PATH=../../vision_system/CameraSDK-20250418_145834-2.0.2-Linux/lib:$LD_LIBRARY_PATH
sudo ./camera_manager
```

## Detailed Usage

### System Requirements Verification

**Current Test Results (from our system):**
```
🔍 SYSTEM REQUIREMENTS CHECK
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
libusb-1.0: ✅ Found
libudev: ✅ Found  
USB utils: ✅ Found
Sudo permissions: ⚠️  Not root (will need sudo)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🔌 USB DEVICE CHECK
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Insta360 devices (Vendor ID 2e1a):
Bus 003 Device 022: ID 2e1a:0002 Arashi Vision Insta360 X3 ✅
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### Camera Configuration Output

When running the utility, you'll see:

```
█████████████████████████████████████████████████████████████
🎥 INSTA360 X3 CAMERA CONFIGURATION
█████████████████████████████████████████████████████████████
📺 PREVIEW STREAM SETTINGS:
  • Resolution: 1920x960 @ 30fps (RES_1920_960P30)
  • Bitrate: 0.5MB (1024*1024/2) - TESTED VALUE FOR X3
  • Encoding: H.264 dual fisheye streams
  • Streams: 2 (stream_index 0 and 1)
  • Audio: Disabled
  • LRV: Disabled (using full resolution)

🔧 CRITICAL REQUIREMENTS:
  ⚠️  Camera MUST be in Android mode:
     Camera Settings → General → USB Mode → Android
  ⚠️  Must run with sudo (libusb requirement)
  ⚠️  USB connection required (no WiFi support)

📋 FROM CAMERA SDK DOCUMENTATION:
  • Preview resolution 1920×960 is currently recommended
  • For high-res modes, two video streams are provided
  • Data format is H.265 or H.264 encoded
  • Decoded data consists of two fisheye images
█████████████████████████████████████████████████████████████
```

### Stream Output

The utility automatically saves dual fisheye streams:
```
📹 RECORDING STARTED:
  Stream 1 (Fisheye 1): 20250130_123456_stream1_fisheye.h264
  Stream 2 (Fisheye 2): 20250130_123456_stream2_fisheye.h264

📹 STREAM STATUS: Stream0=100 frames, Stream1=100 frames, Total=2048KB

📹 RECORDING STOPPED
  Final stats - Stream1: 1200 frames, Stream2: 1200 frames
  Total data: 24MB
```

## Command Line Options

### Camera Manager Options
```bash
./camera_manager [options]

Options:
  --debug        Enable verbose logging
  --status-only  Just check status and exit
  --no-cleanup   Skip killing conflicting processes
  --help         Show help message
```

### Wrapper Script Options
```bash
./run_camera_utility.sh [options]

Options:
  --install-deps    Automatically install missing dependencies
  --skip-checks     Skip system requirement checks
  --diagnose        Run diagnostics and exit
  --help           Show help message
```

### Makefile Targets
```bash
make                # Build camera manager
make debug          # Build with debug symbols
make clean          # Clean build files
make install-deps   # Install system dependencies
make system-check   # Check system requirements
make usb-check      # Check USB devices
make diagnose       # Full diagnostic
make help           # Show all targets
```

## Troubleshooting

### Camera Not Detected
**Symptoms:** `Insta360 USB device: ❌` in status check

**Solutions:**
1. **Check USB Connection:**
   ```bash
   lsusb | grep -i insta360
   # Should show: Bus 003 Device 022: ID 2e1a:0002 Arashi Vision Insta360 X3
   ```

2. **Verify Camera Mode:**
   - Camera Settings → General → USB Mode → **Android** ✅
   - NOT "U disk" mode ❌

3. **Try Different USB Port/Cable:**
   - Use USB 3.0 port if available
   - Try a different high-quality USB-C cable

4. **Restart Camera:**
   - Power off camera completely
   - Wait 10 seconds, power on
   - Reconnect USB

### Permission Issues
**Symptoms:** `failed to open camera` or `libusb` errors

**Solutions:**
1. **Use Sudo:**
   ```bash
   sudo ./run_camera_utility.sh
   ```

2. **Add User to Groups (alternative):**
   ```bash
   sudo usermod -a -G dialout $USER
   sudo usermod -a -G plugdev $USER
   # Logout and login again
   ```

### Build Issues
**Symptoms:** Compilation errors

**Solutions:**
1. **Install Dependencies:**
   ```bash
   sudo apt-get install libusb-1.0-0-dev libudev-dev build-essential
   ```

2. **Check Camera SDK Path:**
   ```bash
   ls -la ../../vision_system/CameraSDK-20250418_145834-2.0.2-Linux/
   ```

3. **Clean Rebuild:**
   ```bash
   make clean
   make
   ```

### Library Loading Errors
**Symptoms:** `libCameraSDK.so: cannot open shared object file`

**Solutions:**
1. **Use Wrapper Script (Recommended):**
   ```bash
   ./run_camera_utility.sh  # Automatically sets library path
   ```

2. **Manual Library Path:**
   ```bash
   export LD_LIBRARY_PATH=../../vision_system/CameraSDK-20250418_145834-2.0.2-Linux/lib:$LD_LIBRARY_PATH
   ./camera_manager
   ```

### Process Conflicts
**Symptoms:** Camera busy, connection failures

**Solutions:**
1. **Automatic Cleanup:**
   ```bash
   ./run_camera_utility.sh  # Automatically kills conflicting processes
   ```

2. **Manual Cleanup:**
   ```bash
   sudo pkill -f "camera|insta360"
   ```

### Low Performance/Latency Issues
**Solutions:**
1. **Close Other Camera Applications:**
   - Close any video recording software
   - Close browser tabs using camera
   - Check background processes

2. **USB Port Selection:**
   - Use USB 3.0 ports
   - Connect directly to computer (not through hub)

3. **System Resources:**
   - Ensure sufficient CPU/memory available
   - Close unnecessary applications

## Integration with AI Agents

The utility is designed to work robustly with AI agents:

### Automated Usage
```bash
# Check if camera is ready
if ./run_camera_utility.sh --diagnose | grep -q "✅"; then
    echo "Camera system ready"
    sudo ./run_camera_utility.sh
else
    echo "Camera system not ready"
    ./run_camera_utility.sh --install-deps
fi
```

### Parsing Status Output
The utility provides clear status indicators that can be parsed:
- `✅` indicates success/ready
- `❌` indicates failure/missing
- `⚠️` indicates warning/needs attention
- `📹` indicates streaming status

### Proactive Error Handling
The utility automatically:
- Kills conflicting processes
- Validates USB connections
- Checks system requirements
- Provides detailed error messages
- Attempts recovery where possible

## Stream Data Usage

The utility outputs H.264 encoded dual fisheye streams that can be:

1. **Fed to Vuer for 360° VR display** (your main use case)
2. **Processed for stitching** into equirectangular format
3. **Streamed over network** for remote viewing
4. **Recorded to files** for later processing

### Example Stream Processing
```python
# Example: Read the H.264 streams for processing
import cv2

# The streams are saved as timestamped H.264 files
stream1_path = "20250130_123456_stream1_fisheye.h264"
stream2_path = "20250130_123456_stream2_fisheye.h264"

# Process with OpenCV or feed to Vuer
cap1 = cv2.VideoCapture(stream1_path)
cap2 = cv2.VideoCapture(stream2_path)
```

## Next Steps for Vuer Integration

1. **Modify Stream Delegate:** Instead of saving to files, send data directly to Vuer's WebRTC bridge
2. **Add Network Streaming:** Stream the H.264 data over UDP to Vuer
3. **Implement Fisheye to Equirectangular:** Convert dual fisheye to 360° format
4. **Add Real-time Processing:** Process frames in real-time for low latency

The foundation is now solid and robust for these next integration steps!