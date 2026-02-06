#!/bin/bash
# Insta360 X3 to Vuer 360° Video Streaming Startup Script
# Coordinates all components for low-latency 360° video in VR teleop

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
CAMERA_STARTUP_DELAY=3
CONVERTER_STARTUP_DELAY=2
VR_SERVER_STARTUP_DELAY=1

# PID tracking
CAMERA_PID=""
CONVERTER_PID=""
VR_SERVER_PID=""

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}🛑 Stopping all 360° streaming components...${NC}"
    
    if [ ! -z "$VR_SERVER_PID" ] && kill -0 $VR_SERVER_PID 2>/dev/null; then
        echo -e "${BLUE}   Stopping VR server (PID: $VR_SERVER_PID)...${NC}"
        kill $VR_SERVER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$CONVERTER_PID" ] && kill -0 $CONVERTER_PID 2>/dev/null; then
        echo -e "${BLUE}   Stopping fisheye converter (PID: $CONVERTER_PID)...${NC}"
        kill $CONVERTER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$CAMERA_PID" ] && kill -0 $CAMERA_PID 2>/dev/null; then
        echo -e "${BLUE}   Stopping camera streamer (PID: $CAMERA_PID)...${NC}"
        kill $CAMERA_PID 2>/dev/null || true
    fi
    
    # Wait a moment for cleanup
    sleep 2
    
    echo -e "${GREEN}✅ All components stopped${NC}"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM EXIT

# Function to check if a process is running
check_process() {
    local pid=$1
    if [ ! -z "$pid" ] && kill -0 $pid 2>/dev/null; then
        return 0  # Process is running
    else
        return 1  # Process is not running
    fi
}

# Function to wait for UDP port to be ready
wait_for_udp_port() {
    local port=$1
    local timeout=$2
    local count=0
    
    echo -e "${CYAN}   Waiting for UDP port $port to be ready...${NC}"
    
    while [ $count -lt $timeout ]; do
        if netstat -un 2>/dev/null | grep -q ":$port "; then
            echo -e "${GREEN}   ✅ UDP port $port is ready${NC}"
            return 0
        fi
        sleep 1
        count=$((count + 1))
        echo -ne "${CYAN}   ⏳ Waiting... ($count/$timeout)\r${NC}"
    done
    
    echo -e "\n${RED}   ❌ Timeout waiting for UDP port $port${NC}"
    return 1
}

# Header
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}🎥 Insta360 X3 to Vuer 360° Video Streaming Pipeline${NC}"
echo -e "${CYAN}💫 Low-latency 360° video for VR teleoperation${NC}"
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"

# Pre-flight checks
echo -e "\n${YELLOW}🔍 Pre-flight checks...${NC}"

# Check if we're in the right directory
if [ ! -f "attempt2_insta360_vuer_streamer.cpp" ]; then
    echo -e "${RED}❌ Error: Not in the correct directory${NC}"
    echo -e "${RED}   Please run this script from the attempt2 directory${NC}"
    exit 1
fi

# Check if binary is built
if [ ! -f "attempt2_insta360_vuer_streamer" ]; then
    echo -e "${YELLOW}⚠️  Camera streamer not built. Building now...${NC}"
    if ! make all; then
        echo -e "${RED}❌ Failed to build camera streamer${NC}"
        exit 1
    fi
fi

# Check if Python scripts exist
if [ ! -f "attempt2_fisheye_to_equirect.py" ]; then
    echo -e "${RED}❌ Error: Fisheye converter script not found${NC}"
    exit 1
fi

if [ ! -f "attempt2_vr_server_with_360.py" ]; then
    echo -e "${RED}❌ Error: VR server script not found${NC}"
    exit 1
fi

# Check if environment is set up
if [ -f "setup_env.sh" ]; then
    echo -e "${BLUE}🔧 Sourcing runtime environment...${NC}"
    source setup_env.sh
else
    echo -e "${YELLOW}⚠️  Runtime environment not set up. Creating now...${NC}"
    make setup-runtime
    source setup_env.sh
fi

# Check for camera
echo -e "${BLUE}📹 Checking for Insta360 X3 camera...${NC}"
if ! lsusb | grep -i insta360 > /dev/null; then
    echo -e "${YELLOW}⚠️  Insta360 camera not detected via USB${NC}"
    echo -e "${YELLOW}   Please ensure the camera is:${NC}"
    echo -e "${YELLOW}   - Connected via USB${NC}"
    echo -e "${YELLOW}   - Powered on${NC}"
    echo -e "${YELLOW}   - Not being used by other software${NC}"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo -e "${GREEN}✅ Pre-flight checks complete${NC}"

# Show pipeline overview
echo -e "\n${CYAN}🔄 Pipeline Overview:${NC}"
echo -e "${BLUE}   1. Camera Streamer:${NC} Insta360 X3 → UDP dual fisheye (port 8082)"
echo -e "${BLUE}   2. Fisheye Converter:${NC} UDP dual fisheye → equirectangular (port 8083)"
echo -e "${BLUE}   3. VR Server:${NC} Vuer with 360° sphere + VR controls (port 8012)"

# Startup sequence
echo -e "\n${YELLOW}🚀 Starting 360° streaming pipeline...${NC}"

# 1. Start camera streamer
echo -e "\n${BLUE}📹 Step 1: Starting camera UDP streamer...${NC}"
./attempt2_insta360_vuer_streamer &
CAMERA_PID=$!

if check_process $CAMERA_PID; then
    echo -e "${GREEN}   ✅ Camera streamer started (PID: $CAMERA_PID)${NC}"
    echo -e "${CYAN}   ⏳ Waiting ${CAMERA_STARTUP_DELAY}s for camera initialization...${NC}"
    sleep $CAMERA_STARTUP_DELAY
else
    echo -e "${RED}   ❌ Failed to start camera streamer${NC}"
    exit 1
fi

# 2. Start fisheye converter
echo -e "\n${BLUE}🔄 Step 2: Starting fisheye to equirectangular converter...${NC}"
python3 attempt2_fisheye_to_equirect.py &
CONVERTER_PID=$!

if check_process $CONVERTER_PID; then
    echo -e "${GREEN}   ✅ Fisheye converter started (PID: $CONVERTER_PID)${NC}"
    echo -e "${CYAN}   ⏳ Waiting ${CONVERTER_STARTUP_DELAY}s for converter initialization...${NC}"
    sleep $CONVERTER_STARTUP_DELAY
else
    echo -e "${RED}   ❌ Failed to start fisheye converter${NC}"
    cleanup
    exit 1
fi

# 3. Start VR server
echo -e "\n${BLUE}🎮 Step 3: Starting VR server with 360° video...${NC}"
python3 attempt2_vr_server_with_360.py &
VR_SERVER_PID=$!

if check_process $VR_SERVER_PID; then
    echo -e "${GREEN}   ✅ VR server started (PID: $VR_SERVER_PID)${NC}"
    echo -e "${CYAN}   ⏳ Waiting ${VR_SERVER_STARTUP_DELAY}s for VR server initialization...${NC}"
    sleep $VR_SERVER_STARTUP_DELAY
else
    echo -e "${RED}   ❌ Failed to start VR server${NC}"
    cleanup
    exit 1
fi

# System status
echo -e "\n${GREEN}🎉 360° Streaming Pipeline Started Successfully!${NC}"
echo -e "\n${CYAN}📊 System Status:${NC}"
echo -e "${BLUE}   📹 Camera Streamer:${NC} Running (PID: $CAMERA_PID)"
echo -e "${BLUE}   🔄 Fisheye Converter:${NC} Running (PID: $CONVERTER_PID)"
echo -e "${BLUE}   🎮 VR Server:${NC} Running (PID: $VR_SERVER_PID)"

# Network information
echo -e "\n${CYAN}🌐 Network Configuration:${NC}"
echo -e "${BLUE}   Dual Fisheye Stream:${NC} UDP port 8082"
echo -e "${BLUE}   Equirectangular Stream:${NC} UDP port 8083"
echo -e "${BLUE}   VR Server:${NC} HTTP port 8012"
echo -e "${BLUE}   VR Data Broadcast:${NC} UDP port 5006"

# Usage instructions
echo -e "\n${CYAN}📋 Usage Instructions:${NC}"
echo -e "${BLUE}   1. Access VR Experience:${NC}"
echo -e "      - Start ngrok: ${YELLOW}ngrok http 8012${NC}"
echo -e "      - Open ngrok URL in Meta Quest Browser"
echo -e "      - Move left controller to start VR teleoperation"
echo -e "\n${BLUE}   2. Robot Control:${NC}"
echo -e "      - Run: ${YELLOW}python ../vr_relay.py${NC} (in another terminal)"
echo -e "      - Run: ${YELLOW}python ../puppet_follow_vr.py${NC} (on robot system)"
echo -e "\n${BLUE}   3. Controls:${NC}"
echo -e "      - ${YELLOW}Squeeze:${NC} Enable robot movement"
echo -e "      - ${YELLOW}Trigger:${NC} Gripper control"
echo -e "      - ${YELLOW}A Button:${NC} Reset robot to neutral"

# Monitoring
echo -e "\n${CYAN}📈 Live Monitoring:${NC}"
echo -e "${BLUE}   Watch for status updates from each component${NC}"
echo -e "${BLUE}   Camera and converter will show FPS statistics${NC}"
echo -e "${BLUE}   VR server will show connection status${NC}"

echo -e "\n${YELLOW}⚠️  Press Ctrl+C to stop all components${NC}"
echo -e "\n${CYAN}════════════════════════════════════════════════════════════════${NC}"

# Main monitoring loop
echo -e "\n${BLUE}🔍 Monitoring pipeline (Ctrl+C to stop)...${NC}"

while true; do
    sleep 5
    
    # Check if all processes are still running
    all_running=true
    
    if ! check_process $CAMERA_PID; then
        echo -e "${RED}❌ Camera streamer stopped unexpectedly${NC}"
        all_running=false
    fi
    
    if ! check_process $CONVERTER_PID; then
        echo -e "${RED}❌ Fisheye converter stopped unexpectedly${NC}"
        all_running=false
    fi
    
    if ! check_process $VR_SERVER_PID; then
        echo -e "${RED}❌ VR server stopped unexpectedly${NC}"
        all_running=false
    fi
    
    if [ "$all_running" = false ]; then
        echo -e "${RED}🛑 Pipeline failure detected - stopping all components${NC}"
        cleanup
        exit 1
    fi
    
    # Optionally show a periodic heartbeat
    # echo -e "${GREEN}💓 Pipeline running...${NC}"
done