# Insta360 X3 to Vuer 360° Video Integration

This solution integrates live 360° video streaming from the Insta360 X3 camera into the existing VR teleoperation system using Vuer. It provides low-latency dual fisheye to equirectangular conversion and seamless VR integration.

## 🎯 Overview

The system creates a complete 360° video streaming pipeline:
1. **Camera Streamer** (C++): Captures dual fisheye video from Insta360 X3 via CameraSDK
2. **Fisheye Converter** (Python): Converts dual fisheye to equirectangular format in real-time
3. **VR Server** (Python): Enhanced Vuer server with 360° sphere and VR controls

## 📋 Features

- ✅ **Low-latency streaming**: Optimized UDP pipeline with minimal buffering
- ✅ **Real-time conversion**: Dual fisheye to equirectangular transformation
- ✅ **VR integration**: 360° video sphere in Vuer VR environment
- ✅ **Seamless controls**: VR controller data still broadcast for robot teleoperation
- ✅ **Production ready**: Comprehensive error handling and monitoring

## 🔧 Requirements

### Hardware
- Insta360 X3 camera connected via USB
- VR headset (Meta Quest tested)
- Ubuntu 20.04 system (NUC11 tested)

### Software Dependencies
- Insta360 CameraSDK (included in `vision_system/CameraSDK-20250418_145834-2.0.2-Linux/`)
- Python 3.8+
- OpenCV Python (`opencv-python`)
- NumPy
- Vuer (`pip install vuer`)
- ngrok (for VR access)

## 🚀 Quick Start

### 1. Initial Setup
```bash
# Navigate to the attempt2 directory
cd temp_360_dev/attempt2

# One-time setup (builds C++ streamer, installs Python deps)
make setup

# Make startup script executable
chmod +x start_360_streaming.sh
```

### 2. Connect Hardware
- Connect Insta360 X3 via USB
- Ensure camera is powered on
- Verify connection: `lsusb | grep -i insta360`

### 3. Start 360° Streaming Pipeline
```bash
# Start all components with monitoring
./start_360_streaming.sh
```

### 4. Access VR Experience
```bash
# In another terminal, start ngrok
ngrok http 8012

# Open the ngrok URL in Meta Quest Browser
# Move left controller to start VR teleoperation with 360° video
```

### 5. Enable Robot Control (Optional)
```bash
# In separate terminals:
python ../vr_relay.py
python ../puppet_follow_vr.py  # On robot system
```

## 📁 File Structure

```
attempt2/
├── attempt2_insta360_vuer_streamer.cpp    # C++ camera UDP streamer
├── attempt2_fisheye_to_equirect.py        # Python dual fisheye converter
├── attempt2_vr_server_with_360.py         # Enhanced VR server with 360° video
├── Makefile                               # Build system for C++ components
├── start_360_streaming.sh                 # Coordinated startup script
├── README_360_INTEGRATION.md              # This documentation
└── setup_env.sh                           # Runtime environment (auto-generated)
```

## 🔄 System Architecture

```
┌─────────────────┐    UDP 8082     ┌─────────────────────┐    UDP 8083     ┌──────────────────┐
│  Insta360 X3    │ ─────────────► │  Fisheye Converter  │ ─────────────► │   VR Server      │
│  Camera         │  Dual Fisheye   │  (equirectangular)  │  Equirectangular │  (Vuer + 360°)   │
│  (CameraSDK)    │                 │  (OpenCV)           │                 │  (WebRTC + VR)   │
└─────────────────┘                 └─────────────────────┘                 └──────────────────┘
                                                                                      │
                                                                                UDP 5006
                                                                                      ▼
                                                                            ┌──────────────────┐
                                                                            │   VR Relay       │
                                                                            │   (Robot Control)│
                                                                            └──────────────────┘
```

## 🛠️ Individual Components

### Camera Streamer (`attempt2_insta360_vuer_streamer`)
- **Purpose**: Captures dual fisheye video from Insta360 X3
- **Technology**: C++ with Insta360 CameraSDK
- **Output**: UDP stream on port 8082 (dual fisheye H.264)
- **Configuration**: 1440x720@30fps, 0.5MB bitrate for low latency

### Fisheye Converter (`attempt2_fisheye_to_equirect.py`)
- **Purpose**: Real-time dual fisheye to equirectangular conversion
- **Technology**: Python + OpenCV
- **Input**: UDP port 8082 (dual fisheye)
- **Output**: UDP port 8083 (equirectangular JPEG)
- **Features**: Stereographic projection, optimized mapping

### VR Server (`attempt2_vr_server_with_360.py`)
- **Purpose**: Enhanced Vuer server with 360° video integration
- **Technology**: Python + Vuer + asyncio
- **Features**: 
  - 360° video sphere with live texture updates
  - VR controller tracking and broadcasting
  - WebRTC streaming to VR headsets
  - Low-latency optimizations

## ⚙️ Configuration Options

### Camera Settings
- Resolution: `RES_1440_720P30` (optimized for X3)
- Bitrate: `1024 * 1024 / 2` (0.5MB for low latency)
- Audio: Disabled (not needed for 360° view)

### Video Processing
- Equirectangular output: 2048x1024 (configurable)
- Update rate: 30 FPS
- Encoding: JPEG with 80% quality

### Network Ports
- **8082**: Camera to converter (dual fisheye)
- **8083**: Converter to VR server (equirectangular)
- **8012**: VR server HTTP (Vuer web interface)
- **5006**: VR data broadcast (robot control)

## 🔧 Build System (Makefile)

### Common Commands
```bash
make help              # Show all available commands
make check-sdk         # Verify CameraSDK installation
make all               # Build C++ streamer
make setup             # Complete setup
make run-pipeline      # Start complete pipeline
make clean             # Clean build artifacts
make debug-info        # Show debug information
```

### Individual Component Testing
```bash
make run-camera        # Test camera streamer only
make run-converter     # Test fisheye converter only
make run-vr-server     # Test VR server only
make test-camera       # Quick camera connection test
```

## 🐛 Troubleshooting

### Camera Issues
```bash
# Check camera connection
lsusb | grep -i insta360

# Test camera access
make test-camera

# Check CameraSDK
make check-sdk
make debug-info
```

### Build Issues
```bash
# Clean and rebuild
make clean
make all

# Check dependencies
make check-sdk
```

### Network Issues
```bash
# Check ports are available
netstat -un | grep "8082\|8083\|8012\|5006"

# Monitor UDP traffic
sudo tcpdump -i lo port 8082  # Camera stream
sudo tcpdump -i lo port 8083  # Converted stream
```

### Performance Issues
```bash
# Monitor system resources
htop

# Check pipeline latency
# Look for warnings in component outputs about latency > 100ms
```

## 📈 Performance Monitoring

Each component provides real-time statistics:

### Camera Streamer
- Packet transmission rate
- Queue status
- Dual fisheye stream health

### Fisheye Converter
- Processing FPS
- Input/output frame rates
- Conversion latency

### VR Server
- VR data broadcast rate
- 360° video update rate
- WebRTC connection status

## 🔒 Production Considerations

### Security
- VR server runs on all interfaces (0.0.0.0) for ngrok access
- Consider firewall rules for production deployment
- ngrok provides HTTPS tunnel for secure VR access

### Performance
- System optimized for low latency over quality
- Consider SSD storage for better I/O performance
- Monitor CPU usage during conversion process

### Reliability
- All components have error handling and recovery
- Pipeline monitoring detects component failures
- Graceful shutdown on interruption

## 🎛️ Advanced Configuration

### Fisheye Projection Parameters
Edit `attempt2_fisheye_to_equirect.py`:
```python
# Adjust fisheye parameters for different cameras
fisheye_radius = min(fisheye_width, fisheye_height) / 2 * 0.9
```

### VR Sphere Settings
Edit `attempt2_vr_server_with_360.py`:
```python
SPHERE_RADIUS = 500  # Distance from user
VIDEO_TEXTURE_UPDATE_RATE = 30  # FPS
```

### Network Buffering
Adjust UDP buffer sizes for different network conditions:
```cpp
// In C++ streamer
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536);
```

## 🤝 Integration with Existing VR System

This solution is designed to be **non-intrusive**:

- ✅ Existing VR server can still be used (just use `vr_server.py`)
- ✅ VR relay and puppet control remain unchanged
- ✅ Robot calibration and control unchanged
- ✅ 360° video is additive enhancement

### Switching Between Modes
```bash
# Standard VR (no 360° video)
python vr_server.py

# Enhanced VR (with 360° video)
./start_360_streaming.sh
```

## 📝 Development Notes

### Memory Usage from User
Based on previous development, the user noted:
- MediaSDK requires significant initialization time (60-90 seconds)
- RES_1920_960P30 results in zero video packets on X3
- RES_1440_720P30 with 0.5MB bitrate works reliably
- UDP streaming has issues with ffmpeg/HLS bridge interference

### Current Solution Addresses
- ✅ Uses CameraSDK (not MediaSDK) to avoid initialization delays
- ✅ Uses proven RES_1440_720P30 resolution
- ✅ Direct UDP pipeline without ffmpeg interference
- ✅ Real-time processing optimized for VR latency requirements

## 🎉 Success Criteria

When working correctly, you should see:
1. **Camera Streamer**: "Live streaming started successfully"
2. **Fisheye Converter**: "📊 360° Processing: XX FPS"
3. **VR Server**: "📡 VR: XXHz OK" and "🎥 360° Video: XX FPS"
4. **VR Headset**: 360° video sphere surrounding you with live camera feed
5. **Robot Control**: Normal VR teleoperation functionality preserved

---

**🚀 Ready to experience immersive 360° VR teleoperation!**