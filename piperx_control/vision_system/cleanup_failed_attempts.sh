#!/bin/bash

# Cleanup script for failed Insta360 streaming attempts
# This removes all the experimental files and keeps only the working solution

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}🧹 Insta360 Streaming Cleanup Script${NC}"
echo "======================================"
echo ""
echo "This script will remove failed attempts and temporary files."
echo "The working streaming system will be preserved."
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Function to ask for confirmation
confirm_delete() {
    local file="$1"
    local description="$2"
    
    if [ -f "$file" ] || [ -d "$file" ]; then
        echo -e "${YELLOW}Delete${NC} $file ${BLUE}($description)${NC}?"
        read -p "  [y/N/q]: " -n 1 -r
        echo
        
        if [[ $REPLY =~ ^[Qq]$ ]]; then
            echo -e "${RED}❌ Cleanup cancelled${NC}"
            exit 0
        elif [[ $REPLY =~ ^[Yy]$ ]]; then
            if [ -d "$file" ]; then
                rm -rf "$file"
            else
                rm -f "$file"
            fi
            echo -e "${GREEN}  ✅ Deleted${NC}"
            return 0
        else
            echo -e "${BLUE}  ⏭️  Skipped${NC}"
            return 1
        fi
    fi
}

# Function to delete without confirmation (for obviously temporary files)
auto_delete() {
    local file="$1"
    local description="$2"
    
    if [ -f "$file" ] || [ -d "$file" ]; then
        echo -e "${YELLOW}Auto-deleting${NC} $file ${BLUE}($description)${NC}"
        if [ -d "$file" ]; then
            rm -rf "$file"
        else
            rm -f "$file"
        fi
        echo -e "${GREEN}  ✅ Deleted${NC}"
    fi
}

echo -e "${BLUE}📝 Files to keep (core working system):${NC}"
echo "  ✅ start_insta360_streaming.sh - Main launcher"
echo "  ✅ simple_webrtc.py - Working WebRTC server"
echo "  ✅ insta360_simple_streamer.cpp - HTTP streamer source"
echo "  ✅ insta360_simple_streamer - Compiled HTTP streamer"
echo "  ✅ CameraSDK-*/ - Required SDK"
echo "  ✅ README_QUICK_START.md - Documentation"
echo ""

# Stop any running processes first
echo -e "${YELLOW}🛑 Stopping any running processes...${NC}"
./start_insta360_streaming.sh --stop 2>/dev/null || true
echo ""

# Auto-delete obvious temporary files
echo -e "${YELLOW}🗑️  Auto-deleting temporary files...${NC}"
auto_delete "streamer.log" "Runtime log file"
auto_delete "webrtc.log" "Runtime log file"
auto_delete "streamer.pid" "Process ID file"
auto_delete "webrtc.pid" "Process ID file"
auto_delete "test_stream.h264" "Large test file (393KB)"
auto_delete ".DS_Store" "macOS system file"
echo ""

# Failed WebRTC/Browser attempts
echo -e "${YELLOW}🌐 Failed WebRTC/Browser attempts:${NC}"
confirm_delete "webrtc_server.py" "Previous failed WebRTC implementation"
confirm_delete "browser_viewer.py" "Alternative browser viewer attempt"
confirm_delete "vlc_stream_wrapper.py" "VLC wrapper attempt"
echo ""

# Failed low-latency streaming attempts
echo -e "${YELLOW}⚡ Failed low-latency streaming attempts:${NC}"
confirm_delete "insta360_low_latency.cpp" "Failed low-latency source"
confirm_delete "insta360_low_latency" "Failed low-latency binary"
confirm_delete "insta360_test_stream.cpp" "Test implementation source"
confirm_delete "insta360_test_stream" "Test implementation binary"
confirm_delete "insta360_rtsp_streamer.cpp" "RTSP attempt"
confirm_delete "gstreamer_udp_stream.cpp" "GStreamer attempt"
echo ""

# Redundant shell scripts
echo -e "${YELLOW}📜 Redundant shell scripts:${NC}"
confirm_delete "direct_low_latency_view.sh" "Redundant (functionality in main script)"
confirm_delete "ultra_low_latency_stream.sh" "Failed attempt"
confirm_delete "low_latency_stream.sh" "Redundant"
confirm_delete "play_stream.sh" "Simple script, redundant"
confirm_delete "vlc_direct.sh" "VLC-specific script"
confirm_delete "stream_to_vlc.sh" "VLC-specific script"
confirm_delete "run_insta360_stream.sh" "Redundant launcher"
confirm_delete "rtsp_server.sh" "RTSP attempt"
echo ""

# Build files (optional)
echo -e "${YELLOW}🔨 Build files (optional):${NC}"
confirm_delete "Makefile" "Build configuration (can regenerate if needed)"
confirm_delete "build_insta360_streamers.sh" "Build script (can regenerate if needed)"
echo ""

# Large directories (be careful)
echo -e "${YELLOW}📁 Large optional directories:${NC}"
echo -e "${BLUE}Note: These contain documentation and examples${NC}"
confirm_delete "Vuer/" "Vuer documentation/examples (large directory)"
confirm_delete "insta360/" "Insta360 SDK documentation (large directory)"
confirm_delete "libMediaSDK-dev-3.0.5.1-20250618_195946-amd64/" "Media SDK package (large directory)"
echo ""

echo -e "${GREEN}✅ Cleanup complete!${NC}"
echo ""
echo -e "${BLUE}📊 Remaining files (core system):${NC}"
echo "$(ls -la | grep -E '\.(sh|py|cpp)$' | grep -E '(start_insta360|simple_webrtc|insta360_simple_streamer)' | wc -l) core files"
echo ""
echo -e "${GREEN}🎉 Your working streaming system is preserved:${NC}"
echo "  🚀 Start streaming: ./start_insta360_streaming.sh"
echo "  🌐 WebRTC browser view: http://localhost:8081"
echo "  📺 Direct stream: curl -s http://localhost:8080 | ffplay -f h264 -"
echo "" 