#!/bin/bash

# Insta360 X3 Camera Recovery Script - Attempt 3
# Comprehensive recovery for SDK connection failures
# This script implements proven recovery methods for common Insta360 X3 issues

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'  
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAX_ATTEMPTS=3
RECOVERY_WAIT_TIME=15  # seconds between attempts

# Logging functions
log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

log_header() {
    echo -e "${BOLD}$1${NC}"
}

check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        log_error "SUDO REQUIRED: Recovery script must run with sudo permissions"
        log_info "Run: sudo $0 $*"
        exit 1
    fi
}

install_usbreset() {
    log_info "📦 Installing usbreset utility for USB device recovery..."
    
    # Check if usbreset is already available
    if command -v usbreset >/dev/null 2>&1; then
        log_success "usbreset already installed"
        return 0
    fi
    
    # Try to install from package
    if command -v apt-get >/dev/null 2>&1; then
        apt-get update -qq
        if apt-get install -y usbutils usb-reset 2>/dev/null; then
            log_success "usbreset installed via package manager"
            return 0
        fi
    fi
    
    # Build from source if package not available
    log_info "Building usbreset from source..."
    cat > /tmp/usbreset.c << 'EOF'
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

int main(int argc, char **argv) {
    const char *filename;
    int fd;
    int rc;

    if (argc != 2) {
        fprintf(stderr, "Usage: usbreset device-filename\n");
        return 1;
    }
    filename = argv[1];

    fd = open(filename, O_WRONLY);
    if (fd < 0) {
        perror("Error opening output file");
        return 1;
    }

    printf("Resetting USB device %s\n", filename);
    rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0) {
        perror("Error in ioctl");
        return 1;
    }
    printf("Reset successful\n");

    close(fd);
    return 0;
}
EOF

    if gcc -o /usr/local/bin/usbreset /tmp/usbreset.c 2>/dev/null; then
        chmod +x /usr/local/bin/usbreset
        log_success "usbreset compiled and installed to /usr/local/bin/"
        rm -f /tmp/usbreset.c
        return 0
    else
        log_warning "Failed to compile usbreset, will use alternative methods"
        rm -f /tmp/usbreset.c
        return 1
    fi
}

force_kill_camera_processes() {
    log_header "🔄 AGGRESSIVE CAMERA PROCESS CLEANUP"
    
    # Kill all possible Insta360/Camera related processes
    local processes_killed=0
    
    # Direct process names
    for proc in "CameraSDKTest" "insta360" "Insta360" "ffmpeg" "ffplay"; do
        if pgrep -f "$proc" > /dev/null; then
            log_info "Killing $proc processes..."
            pkill -TERM -f "$proc" 2>/dev/null || true
            sleep 1
            pkill -KILL -f "$proc" 2>/dev/null || true
            processes_killed=$((processes_killed + 1))
        fi
    done
    
    # Kill processes using video devices
    if lsof /dev/video* 2>/dev/null | grep -q video; then
        log_info "Killing processes using video devices..."
        lsof /dev/video* 2>/dev/null | awk 'NR>1 {print $2}' | xargs -r kill -KILL 2>/dev/null || true
        processes_killed=$((processes_killed + 1))
    fi
    
    # Kill processes with USB devices
    if lsof 2>/dev/null | grep -i "2e1a\|insta360" | grep -q .; then
        log_info "Killing processes using Insta360 USB device..."
        lsof 2>/dev/null | grep -i "2e1a\|insta360" | awk '{print $2}' | xargs -r kill -KILL 2>/dev/null || true
        processes_killed=$((processes_killed + 1))
    fi
    
    # Clear shared memory segments
    log_info "Clearing shared memory segments..."
    ipcs -m 2>/dev/null | awk 'NR>3 {print $2}' | xargs -r ipcrm -m 2>/dev/null || true
    
    # Clear semaphores
    ipcs -s 2>/dev/null | awk 'NR>3 {print $2}' | xargs -r ipcrm -s 2>/dev/null || true
    
    log_success "Process cleanup completed (${processes_killed} process groups affected)"
    sleep 2  # Give processes time to fully terminate
}

reset_usb_device() {
    log_header "🔌 USB DEVICE RESET"
    
    # Find Insta360 device
    local usb_info=$(lsusb | grep -i "2e1a\|insta360")
    if [ -z "$usb_info" ]; then
        log_error "Insta360 device not found in USB"
        return 1
    fi
    
    log_info "Found device: $usb_info"
    
    # Extract bus and device numbers
    local bus_num=$(echo "$usb_info" | sed -n 's/Bus \([0-9]*\) Device \([0-9]*\):.*/\1/p')
    local dev_num=$(echo "$usb_info" | sed -n 's/Bus \([0-9]*\) Device \([0-9]*\):.*/\2/p')
    
    if [ -z "$bus_num" ] || [ -z "$dev_num" ]; then
        log_error "Cannot parse USB device information"
        return 1
    fi
    
    log_info "USB Bus: $bus_num, Device: $dev_num"
    
    # Method 1: usbreset utility
    if command -v usbreset >/dev/null 2>&1; then
        local usb_dev_path="/dev/bus/usb/$(printf "%03d" $bus_num)/$(printf "%03d" $dev_num)"
        if [ -e "$usb_dev_path" ]; then
            log_info "Attempting USB reset with usbreset utility..."
            if usbreset "$usb_dev_path" 2>/dev/null; then
                log_success "USB device reset successful"
                sleep 3
                return 0
            fi
        fi
    fi
    
    # Method 2: sysfs unbind/bind
    log_info "Attempting sysfs unbind/bind reset..."
    local usb_path=$(find /sys/bus/usb/devices -name "${bus_num}-*" -type d 2>/dev/null | head -1)
    if [ -n "$usb_path" ]; then
        local usb_id=$(basename "$usb_path")
        log_info "USB device path: $usb_path (ID: $usb_id)"
        
        # Unbind and rebind
        echo "$usb_id" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null || true
        sleep 2
        echo "$usb_id" > /sys/bus/usb/drivers/usb/bind 2>/dev/null || true
        sleep 3
        
        # Check if device is back
        if lsusb | grep -q "2e1a\|insta360"; then
            log_success "USB device reset via sysfs successful"
            return 0
        fi
    fi
    
    # Method 3: Power management reset
    log_info "Attempting power management reset..."
    for usb_device in /sys/bus/usb/devices/*/; do
        if [ -f "$usb_device/idVendor" ] && [ -f "$usb_device/idProduct" ]; then
            local vendor=$(cat "$usb_device/idVendor" 2>/dev/null)
            if [ "$vendor" = "2e1a" ]; then
                log_info "Found Insta360 device at $usb_device"
                # Try power cycle
                echo auto > "$usb_device/power/control" 2>/dev/null || true
                echo 0 > "$usb_device/power/autosuspend_delay_ms" 2>/dev/null || true
                echo mem > "$usb_device/power/level" 2>/dev/null || true
                sleep 1
                echo on > "$usb_device/power/level" 2>/dev/null || true
                sleep 2
                break
            fi
        fi
    done
    
    # Verify device is back
    if lsusb | grep -q "2e1a\|insta360"; then
        log_success "USB device reset completed"
        return 0
    else
        log_warning "USB device reset may have failed"
        return 1
    fi
}

attempt_camera_connection() {
    local attempt_num=$1
    
    log_header "🎥 CAMERA CONNECTION ATTEMPT $attempt_num"
    
    # Pre-connection cleanup
    force_kill_camera_processes
    
    # USB reset (skip on first attempt unless specifically requested)
    if [ "$attempt_num" -gt 1 ] || [ "$1" = "--force-usb-reset" ]; then
        reset_usb_device
    fi
    
    # Try camera connection
    log_info "Launching camera utility..."
    cd "$SCRIPT_DIR"
    
    # Run with timeout to prevent hanging
    timeout 60s ./run_camera_utility.sh --verify
    local result=$?
    
    if [ $result -eq 0 ]; then
        log_success "CAMERA CONNECTION SUCCESSFUL on attempt $attempt_num!"
        return 0
    elif [ $result -eq 124 ]; then
        log_error "Camera connection timed out after 60 seconds"
        return 1
    else
        log_warning "Camera connection failed (exit code: $result)"
        return 1
    fi
}

main() {
    log_header "🔧 INSTA360 X3 CAMERA RECOVERY UTILITY - ATTEMPT 3"
    log_header "Comprehensive recovery for SDK connection failures"
    echo ""
    
    # Parse arguments
    local force_usb_reset=false
    local max_attempts=$MAX_ATTEMPTS
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --force-usb-reset)
                force_usb_reset=true
                shift
                ;;
            --max-attempts)
                max_attempts="$2"
                shift 2
                ;;
            --help)
                echo "Usage: $0 [options]"
                echo ""
                echo "Options:"
                echo "  --force-usb-reset     Force USB reset on first attempt"
                echo "  --max-attempts N      Maximum recovery attempts (default: $MAX_ATTEMPTS)"
                echo "  --help               Show this help"
                echo ""
                echo "This script implements proven recovery methods for Insta360 X3 SDK failures:"
                echo "1. Aggressive process cleanup"
                echo "2. USB device reset (multiple methods)"
                echo "3. Shared memory cleanup"
                echo "4. Progressive retry with increasing wait times"
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done
    
    # Ensure sudo
    check_sudo
    
    # Install recovery tools
    install_usbreset
    
    # Show initial status
    log_info "📋 INITIAL SYSTEM STATUS:"
    if lsusb | grep -i "2e1a\|insta360"; then
        log_success "Insta360 device detected in USB"
        lsusb | grep -i "2e1a\|insta360"
    else
        log_error "Insta360 device NOT detected - check physical connection"
        exit 1
    fi
    
    # Recovery attempts
    for attempt in $(seq 1 $max_attempts); do
        echo ""
        log_header "━━━━━ RECOVERY ATTEMPT $attempt/$max_attempts ━━━━━"
        
        if [ "$force_usb_reset" = true ] && [ $attempt -eq 1 ]; then
            if attempt_camera_connection "--force-usb-reset"; then
                log_success "🎉 CAMERA RECOVERY SUCCESSFUL!"
                exit 0
            fi
        else
            if attempt_camera_connection $attempt; then
                log_success "🎉 CAMERA RECOVERY SUCCESSFUL!"
                exit 0
            fi
        fi
        
        # Wait before next attempt (increasing wait time)
        if [ $attempt -lt $max_attempts ]; then
            local wait_time=$((RECOVERY_WAIT_TIME + (attempt * 5)))
            log_info "⏰ Waiting ${wait_time} seconds before next attempt..."
            log_info "💡 This gives the camera time to release resources completely"
            sleep $wait_time
        fi
    done
    
    # All attempts failed
    echo ""
    log_header "❌ ALL RECOVERY ATTEMPTS FAILED"
    log_error "The camera SDK connection could not be established after $max_attempts attempts"
    echo ""
    log_header "🔧 MANUAL RECOVERY REQUIRED:"
    echo "1. 🔋 POWER CYCLE the camera: Turn OFF, wait 30 seconds, turn ON"
    echo "2. 🔌 UNPLUG and RECONNECT the USB cable"
    echo "3. ⏰ WAIT 60 seconds for the camera to fully initialize"
    echo "4. 🔄 Run this recovery script again"
    echo "5. 🔍 If still failing, check camera mode:"
    echo "   Camera Settings → General → USB Mode → Android (NOT U disk)"
    echo ""
    log_header "🐛 FOR DEBUGGING:"
    echo "   sudo ps aux | grep -E '[C]ameraSDK|[i]nsta360'"
    echo "   sudo lsof | grep -i insta360"
    echo "   dmesg | tail -20 | grep -i usb"
    echo ""
    
    exit 1
}

# Trap for clean exit
trap 'log_warning "Recovery interrupted"; exit 130' INT TERM

# Run main function
main "$@"