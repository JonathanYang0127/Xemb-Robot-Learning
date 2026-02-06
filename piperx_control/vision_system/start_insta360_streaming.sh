#!/bin/bash

# Robust Insta360 X3 Streaming Launcher
# This script handles all the complexity for you

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}🚀 Insta360 X3 Streaming System${NC}"
echo "===================================="
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Function to kill processes safely
cleanup_processes() {
    echo -e "${YELLOW}🧹 Cleaning up old processes...${NC}"
    
    # Kill any existing streaming processes
    pkill -f "insta360_simple_streamer" 2>/dev/null || true
    pkill -f "insta360_low_latency" 2>/dev/null || true
    pkill -f "simple_webrtc.py" 2>/dev/null || true
    pkill -f "browser_viewer.py" 2>/dev/null || true
    pkill -f "webrtc_server.py" 2>/dev/null || true
    
    # Kill processes on specific ports
    fuser -k 8080/tcp 2>/dev/null || true
    fuser -k 8081/tcp 2>/dev/null || true
    fuser -k 8090/tcp 2>/dev/null || true
    
    sleep 2
    echo -e "${GREEN}✅ Cleanup complete${NC}"
}

# Function to check USB permissions
check_usb_permissions() {
    echo -e "${YELLOW}🔍 Checking USB permissions...${NC}"
    
    # Find Insta360 device
    DEVICE=$(lsusb | grep -i "insta\|arashi" | awk '{print $2 ":" $4}' | sed 's/:$//')
    
    if [ -z "$DEVICE" ]; then
        echo -e "${RED}❌ No Insta360 X3 camera found!${NC}"
        echo "Please make sure the camera is:"
        echo "  1. Connected via USB"
        echo "  2. Powered on"
        echo "  3. Not in use by another application"
        return 1
    fi
    
    BUS=$(echo $DEVICE | cut -d: -f1)
    DEV=$(echo $DEVICE | cut -d: -f2)
    
    echo -e "${GREEN}✅ Found Insta360 X3 on bus $BUS device $DEV${NC}"
    
    # Set permissions
    USB_PATH="/dev/bus/usb/$BUS/$DEV"
    if [ -e "$USB_PATH" ]; then
        echo "Setting USB permissions..."
        sudo chmod 666 "$USB_PATH" 2>/dev/null || {
            echo -e "${YELLOW}⚠️  Could not set permissions. You may need to run with sudo.${NC}"
        }
    fi
}

# Function to build if needed
build_if_needed() {
    if [ ! -f "$SCRIPT_DIR/insta360_simple_streamer" ]; then
        echo -e "${YELLOW}🔨 Building streaming applications...${NC}"
        
        # Check for dependencies
        if ! command -v g++ &> /dev/null; then
            echo "Installing g++..."
            sudo apt update && sudo apt install -y g++ build-essential
        fi
        
        # Build
        if [ -f "$SCRIPT_DIR/Makefile" ]; then
            make clean 2>/dev/null || true
            make
        elif [ -f "$SCRIPT_DIR/build_insta360_streamers.sh" ]; then
            ./build_insta360_streamers.sh
        else
            # Direct compilation
            g++ -std=c++11 -o insta360_simple_streamer insta360_simple_streamer.cpp \
                -I./CameraSDK-*/include \
                -L./CameraSDK-*/lib \
                -lCameraSDK -pthread
        fi
        
        echo -e "${GREEN}✅ Build complete${NC}"
    fi
}

# Function to start the HTTP streamer
start_http_streamer() {
    echo -e "${YELLOW}📡 Starting HTTP streamer...${NC}"
    
    # Set library path
    export LD_LIBRARY_PATH="$SCRIPT_DIR/CameraSDK-20250418_145834-2.0.2-Linux/lib:$LD_LIBRARY_PATH"
    
    # Start streamer in background
    nohup ./insta360_simple_streamer > streamer.log 2>&1 &
    STREAMER_PID=$!
    
    # Wait for it to start
    for i in {1..10}; do
        if netstat -tln | grep -q ":8080 "; then
            echo -e "${GREEN}✅ HTTP streamer running on port 8080${NC}"
            return 0
        fi
        sleep 1
    done
    
    echo -e "${RED}❌ Failed to start HTTP streamer${NC}"
    echo "Check streamer.log for details"
    return 1
}

# Function to start WebRTC server
start_webrtc_server() {
    echo -e "${YELLOW}🌐 Starting WebRTC server...${NC}"
    
    # Check Python dependencies
    if ! python3 -c "import aiohttp" 2>/dev/null; then
        echo "Installing Python dependencies..."
        pip install aiohttp aiohttp-cors aiortc av numpy
    fi
    
    # Start WebRTC server
    nohup python3 simple_webrtc.py > webrtc.log 2>&1 &
    WEBRTC_PID=$!
    
    sleep 2
    
    if netstat -tln | grep -q ":8081 "; then
        echo -e "${GREEN}✅ WebRTC server running on port 8081${NC}"
        return 0
    else
        echo -e "${RED}❌ Failed to start WebRTC server${NC}"
        echo "Check webrtc.log for details"
        return 1
    fi
}

# Main menu
show_menu() {
    echo ""
    echo "Choose streaming mode:"
    echo ""
    echo "1) Low Latency Viewing (50-100ms) - Best for local"
    echo "2) WebRTC Browser Viewing (100-200ms) - Best for internet/VR"
    echo "3) Both (runs all servers)"
    echo "4) Stop all streaming"
    echo "5) Show status"
    echo ""
    read -p "Enter choice (1-5): " choice
    
    case $choice in
        1)
            echo -e "\n${GREEN}📺 Direct viewing options:${NC}"
            echo ""
            echo "Option A - FFplay (recommended):"
            echo -e "${YELLOW}curl -s http://localhost:8080 | ffplay -f h264 -fflags nobuffer -flags low_delay -framedrop -${NC}"
            echo ""
            echo "Option B - VLC:"
            echo -e "${YELLOW}curl -s http://localhost:8080 | vlc --demux h264 --network-caching=0 -${NC}"
            echo ""
            echo "Option C - Run helper script:"
            echo -e "${YELLOW}./direct_low_latency_view.sh${NC}"
            ;;
            
        2)
            echo -e "\n${GREEN}🌐 WebRTC viewing:${NC}"
            echo ""
            echo "Local access:"
            echo -e "${YELLOW}http://localhost:8081${NC}"
            echo ""
            echo "For internet access:"
            echo "1. Install ngrok: snap install ngrok"
            echo "2. Run: ngrok http 8081"
            echo "3. Share the ngrok URL"
            ;;
            
        3)
            echo -e "\n${GREEN}✅ All servers running!${NC}"
            echo ""
            echo "HTTP Stream: http://localhost:8080"
            echo "WebRTC View: http://localhost:8081"
            ;;
            
        4)
            cleanup_processes
            echo -e "${GREEN}✅ All streaming stopped${NC}"
            exit 0
            ;;
            
        5)
            echo -e "\n${YELLOW}📊 Current status:${NC}"
            echo ""
            
            if pgrep -f "insta360_simple_streamer" > /dev/null; then
                echo -e "HTTP Streamer: ${GREEN}Running${NC} on port 8080"
            else
                echo -e "HTTP Streamer: ${RED}Not running${NC}"
            fi
            
            if netstat -tln | grep -q ":8081 "; then
                echo -e "WebRTC Server: ${GREEN}Running${NC} on port 8081"
            else
                echo -e "WebRTC Server: ${RED}Not running${NC}"
            fi
            
            echo ""
            echo "Logs available at:"
            echo "  - streamer.log (HTTP streamer)"
            echo "  - webrtc.log (WebRTC server)"
            exit 0
            ;;
            
        *)
            echo -e "${RED}Invalid choice${NC}"
            exit 1
            ;;
    esac
}

# Main execution
main() {
    # Step 1: Clean up old processes
    cleanup_processes
    
    # Step 2: Check USB permissions
    if ! check_usb_permissions; then
        exit 1
    fi
    
    # Step 3: Build if needed
    build_if_needed
    
    # Step 4: Start HTTP streamer
    if ! start_http_streamer; then
        exit 1
    fi
    
    # Get user choice
    if [ "$1" == "--webrtc" ]; then
        choice=2
    elif [ "$1" == "--direct" ]; then
        choice=1
    elif [ "$1" == "--all" ]; then
        choice=3
    else
        show_menu
    fi
    
    # Start WebRTC if requested
    if [ "$choice" == "2" ] || [ "$choice" == "3" ]; then
        start_webrtc_server
    fi
    
    # Show final instructions based on choice
    if [ "$choice" == "1" ]; then
        echo -e "\n${GREEN}📺 Direct viewing options:${NC}"
        echo ""
        echo "Option A - FFplay (recommended):"
        echo -e "${YELLOW}curl -s http://localhost:8080 | ffplay -f h264 -fflags nobuffer -flags low_delay -framedrop -${NC}"
        echo ""
        echo "Option B - VLC:"
        echo -e "${YELLOW}curl -s http://localhost:8080 | vlc --demux h264 --network-caching=0 -${NC}"
        echo ""
        echo "Option C - Run helper script:"
        echo -e "${YELLOW}./direct_low_latency_view.sh${NC}"
    elif [ "$choice" == "2" ]; then
        echo -e "\n${GREEN}🌐 WebRTC viewing:${NC}"
        echo ""
        echo "Local access:"
        echo -e "${YELLOW}http://localhost:8081${NC}"
        echo ""
        echo "For internet access:"
        echo "1. Install ngrok: snap install ngrok"
        echo "2. Run: ngrok http 8081"
        echo "3. Share the ngrok URL"
    elif [ "$choice" == "3" ]; then
        echo -e "\n${GREEN}✅ All servers running!${NC}"
        echo ""
        echo "HTTP Stream: http://localhost:8080"
        echo "WebRTC View: http://localhost:8081"
    fi
    
    echo ""
    echo -e "${GREEN}✅ Streaming system ready!${NC}"
    echo ""
    echo "Process PIDs saved to:"
    echo "  - streamer.pid"
    echo "  - webrtc.pid (if started)"
    echo ""
    echo "To stop later: $0 --stop"
    echo "To check status: $0 --status"
    
    # Save PIDs
    echo $STREAMER_PID > streamer.pid
    [ ! -z "$WEBRTC_PID" ] && echo $WEBRTC_PID > webrtc.pid
}

# Handle command line arguments
case "$1" in
    --stop)
        cleanup_processes
        rm -f streamer.pid webrtc.pid
        echo -e "${GREEN}✅ All streaming stopped${NC}"
        exit 0
        ;;
    --status)
        choice=5
        show_menu
        ;;
    --help)
        echo "Usage: $0 [OPTIONS]"
        echo ""
        echo "Options:"
        echo "  --direct    Start HTTP streamer only (low latency)"
        echo "  --webrtc    Start HTTP + WebRTC servers"
        echo "  --all       Start all servers"
        echo "  --stop      Stop all streaming"
        echo "  --status    Show current status"
        echo "  --help      Show this help"
        echo ""
        echo "Without options, shows interactive menu"
        exit 0
        ;;
esac

# Run main function
main "$@" 