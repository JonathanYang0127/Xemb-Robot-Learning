# Insta360 X3 Camera Utility - Attempt 3

## Overview
Robust camera utility for Insta360 X3 using Camera SDK, designed for reliable operation in AI agent environments.

## Key Requirements Met
- ✅ Create camera instance and open connection
- ✅ Set preview mode with 1920x960 resolution  
- ✅ Proactive sudo permission handling
- ✅ USB connection detection and management
- ✅ Conflict resolution (kill/disconnect other instances)
- ✅ Comprehensive error checking and recovery
- ✅ Clear status reporting with context

## Technical Specifications

### Camera Configuration
- **Model**: Insta360 X3
- **Mode**: Preview streaming mode
- **Resolution**: 1920x960 @ 30fps (RES_1920_960P30)
- **Bitrate**: 0.5MB (1024*1024/2) - tested working value for X3
- **Encoding**: H.264 (dual fisheye streams)
- **Streams**: 2 streams (stream_index 0 and 1 for dual fisheye)

### Prerequisites (CRITICAL)
1. **Camera USB Mode**: Must be set to "Android" mode
   - Camera Settings → General → USB Mode → Android
   - DEFAULT mode is "U disk" which will NOT work
2. **Linux Permissions**: Requires sudo for libusb access
3. **Dependencies**: libusb-dev, libudev-dev installed

### USB Device Validation
- Expected USB Vendor ID: 0x2e1a (Insta360)
- USB detection via lsusb command
- Automatic conflict detection and resolution

## Files
- `camera_manager.cpp` - Main camera utility implementation
- `camera_manager.h` - Header with definitions and interface
- `Makefile` - Build configuration
- `run_camera_utility.sh` - Wrapper script with sudo handling

## Usage
```bash
# Run with automatic conflict resolution
sudo ./run_camera_utility.sh

# Run with debug output
sudo ./camera_manager --debug

# Check camera status only
sudo ./camera_manager --status-only
```

## Error Handling
- USB connection validation
- Process conflict detection
- Camera mode verification  
- Stream initialization validation
- Automatic recovery attempts
- Detailed error reporting

## Integration Notes
- Designed for use by other AI agents
- Proactive conflict resolution
- Clear status output for parsing
- Robust error recovery
- No user interaction required