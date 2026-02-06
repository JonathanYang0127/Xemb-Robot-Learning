#!/bin/bash

echo "🔨 Building Insta360 X3 Streaming Tools"
echo "======================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -f "Makefile" ]; then
    echo -e "${RED}❌ Error: Makefile not found. Please run this script from the vision_system directory.${NC}"
    exit 1
fi

# Check for required SDK
if [ ! -d "CameraSDK-20250418_145834-2.0.2-Linux" ]; then
    echo -e "${RED}❌ Error: Insta360 Camera SDK not found.${NC}"
    echo "Please ensure the SDK is extracted in the current directory."
    exit 1
fi

# Set library path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/CameraSDK-20250418_145834-2.0.2-Linux/lib

# Check for required packages
echo "📦 Checking dependencies..."

# Check for g++
if ! command -v g++ &> /dev/null; then
    echo -e "${RED}❌ g++ not found. Please install it:${NC}"
    echo "   sudo apt-get install build-essential"
    exit 1
fi

# Check for GStreamer (optional, only for RTSP streamer)
if command -v pkg-config &> /dev/null && pkg-config --exists gstreamer-1.0 gstreamer-app-1.0; then
    echo -e "${GREEN}✅ GStreamer found - will build RTSP streamer${NC}"
    BUILD_RTSP=1
else
    echo -e "${YELLOW}⚠️  GStreamer not found - skipping RTSP streamer${NC}"
    echo "   To install: sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev"
    BUILD_RTSP=0
fi

# Clean previous builds
echo "🧹 Cleaning previous builds..."
make clean 2>/dev/null || true

# Build simple HTTP streamer
echo "🔨 Building simple HTTP streamer..."
if g++ -std=c++11 -Wall -O2 \
    -I./CameraSDK-20250418_145834-2.0.2-Linux/include \
    -o insta360_simple_streamer insta360_simple_streamer.cpp \
    -L./CameraSDK-20250418_145834-2.0.2-Linux/lib -lCameraSDK -lpthread; then
    echo -e "${GREEN}✅ Simple HTTP streamer built successfully${NC}"
else
    echo -e "${RED}❌ Failed to build simple HTTP streamer${NC}"
    exit 1
fi

# Build RTSP streamer if GStreamer is available
if [ $BUILD_RTSP -eq 1 ]; then
    echo "🔨 Building RTSP streamer..."
    if make insta360_rtsp_streamer; then
        echo -e "${GREEN}✅ RTSP streamer built successfully${NC}"
    else
        echo -e "${YELLOW}⚠️  Failed to build RTSP streamer${NC}"
    fi
fi

# Make scripts executable
chmod +x rtsp_server.sh 2>/dev/null || true
chmod +x run_insta360_stream.sh 2>/dev/null || true

echo ""
echo -e "${GREEN}✅ Build complete!${NC}"
echo ""
echo "📚 Usage:"
echo "   Simple HTTP Streaming (recommended):"
echo "     ./insta360_simple_streamer"
echo "     Then open VLC and go to: http://localhost:8080"
echo ""
if [ $BUILD_RTSP -eq 1 ]; then
    echo "   RTSP Streaming (advanced):"
    echo "     1. Start RTSP server: ./rtsp_server.sh"
    echo "     2. Start streamer: ./insta360_rtsp_streamer"
    echo "     3. Open in VLC: rtsp://localhost:8554/insta360"
fi
echo ""
echo "💡 Note: Make sure your Insta360 X3 is connected via USB" 