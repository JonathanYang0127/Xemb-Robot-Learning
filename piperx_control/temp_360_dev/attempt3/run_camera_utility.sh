#!/bin/bash

# Insta360 X3 Camera Utility Runner - Attempt 3
# Wrapper script with automatic sudo handling and comprehensive checks

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'  
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_MANAGER="$SCRIPT_DIR/camera_manager"
REQUIRED_PACKAGES=("libusb-1.0-0-dev" "libudev-dev" "usbutils")

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

# CHECK sudo permissions (non-interactive)
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        # Check if we can sudo without password (cached or NOPASSWD)
        if sudo -n true 2>/dev/null; then
            log_info "Sudo available, relaunching with privileges..."
            exec sudo "$0" "$@"
        else
            # No cached sudo - provide clear guidance
            log_error "❌ SUDO REQUIRED: Camera utility MUST run with sudo permissions"
            log_info "💡 SOLUTION: Run this command manually:"
            log_info "   sudo $0 $*"
            log_info ""
            log_info "ℹ️  Camera hardware requires root permissions for USB access"
            exit 1
        fi
    fi
    log_success "✅ Running with root permissions"
}

# Check system requirements
check_requirements() {
    log_header "🔍 CHECKING SYSTEM REQUIREMENTS"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    local all_good=true
    
    # Check required packages
    for package in "${REQUIRED_PACKAGES[@]}"; do
        if dpkg -l | grep -q "^ii  $package"; then
            log_success "$package: Installed"
        else
            log_error "$package: Missing"
            all_good=false
        fi
    done
    
    # Check USB utils
    if command -v lsusb >/dev/null 2>&1; then
        log_success "lsusb: Available"
    else
        log_error "lsusb: Missing (install usbutils)"
        all_good=false
    fi
    
    # Check for Camera SDK
    local sdk_path="$SCRIPT_DIR/../../vision_system/CameraSDK-20250418_145834-2.0.2-Linux"
    if [ -d "$sdk_path" ]; then
        log_success "Camera SDK: Found at $sdk_path"
    else
        log_error "Camera SDK: Not found at expected location"
        log_info "Expected: $sdk_path"
        all_good=false
    fi
    
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    if [ "$all_good" = false ]; then
        log_error "System requirements not met"
        return 1
    fi
    
    log_success "All system requirements satisfied"
    return 0
}

# Install missing dependencies
install_dependencies() {
    log_header "📦 INSTALLING DEPENDENCIES"
    
    log_info "Updating package list..."
    apt-get update -qq
    
    log_info "Installing required packages..."
    apt-get install -y "${REQUIRED_PACKAGES[@]}"
    
    log_success "Dependencies installed successfully"
}

# Check USB connection
check_usb_connection() {
    log_header "🔌 CHECKING USB CONNECTION"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    log_info "Scanning USB devices..."
    lsusb
    echo ""
    
    # Check for Insta360 device
    if lsusb | grep -i "2e1a\|insta360" >/dev/null; then
        log_success "Insta360 camera detected"
        lsusb | grep -i "2e1a\|insta360"
        return 0
    else
        log_error "Insta360 camera NOT detected"
        echo ""
        log_warning "TROUBLESHOOTING STEPS:"
        echo "1. 🔌 Check USB cable connection"
        echo "2. 🔋 Ensure camera is powered on"
        echo "3. ⚙️  Set camera to Android mode:"
        echo "   Camera Settings → General → USB Mode → Android"
        echo "   (DEFAULT is U disk mode which will NOT work)"
        echo "4. 🔄 Try different USB port/cable"
        echo "5. 🔁 Restart camera and reconnect"
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        return 1
    fi
}

# Build camera manager if needed
build_camera_manager() {
    if [ ! -f "$CAMERA_MANAGER" ]; then
        log_info "Camera manager not found, building..."
        cd "$SCRIPT_DIR"
        make clean
        make
        log_success "Build completed"
    else
        log_success "Camera manager executable found"
    fi
}

# Kill conflicting processes
cleanup_processes() {
    log_header "🔄 CLEANING UP CONFLICTING PROCESSES"
    
    # Find camera-related processes but exclude our own process tree and scripts
    local all_pids=$(pgrep -f "camera|insta360|Camera" || true)
    local filtered_pids=""
    
    if [ -z "$all_pids" ]; then
        log_success "No conflicting processes found"
        return 0
    fi
    
    # Filter out our own processes and scripts
    for pid in $all_pids; do
        # Skip our own PID and parent PID
        if [ "$pid" = "$$" ] || [ "$pid" = "$PPID" ]; then
            continue
        fi
        
        # Check what this PID actually is
        local cmd=$(ps -p "$pid" -o cmd= 2>/dev/null || true)
        
        # Skip if it's our own scripts
        if echo "$cmd" | grep -q "run_camera_utility.sh\|camera_manager"; then
            continue
        fi
        
        # Skip if it's sudo of our scripts  
        if echo "$cmd" | grep -q "sudo.*run_camera_utility\|sudo.*camera_manager"; then
            continue
        fi
        
        # Only include actual camera processes
        if echo "$cmd" | grep -qE "insta360|CameraSDK|ffmpeg.*insta|ffplay.*insta"; then
            filtered_pids="$filtered_pids $pid"
        fi
    done
    
    if [ -z "$filtered_pids" ]; then
        log_success "No conflicting camera processes found (excluded our own scripts)"
        return 0
    fi
    
    log_warning "Found camera-related processes to clean up:"
    for pid in $filtered_pids; do
        ps -p "$pid" -o pid,ppid,cmd 2>/dev/null || true
    done
    
    log_info "Terminating camera processes..."
    for pid in $filtered_pids; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    
    sleep 2
    
    # Force kill if still running
    log_info "Checking for remaining processes..."
    local remaining_pids=""
    for pid in $filtered_pids; do
        if kill -0 "$pid" 2>/dev/null; then
            remaining_pids="$remaining_pids $pid"
        fi
    done
    
    if [ -n "$remaining_pids" ]; then
        log_warning "Force killing remaining camera processes..."
        for pid in $remaining_pids; do
            kill -KILL "$pid" 2>/dev/null || true
        done
    fi
    
    log_success "Process cleanup completed"
}

# Main function
main() {
    log_header "🎥 INSTA360 X3 CAMERA UTILITY - ATTEMPT 3"
    log_header "Robust camera manager for AI agent environments"
    echo ""
    
    # Parse arguments
    local auto_install=false
    local skip_checks=false
    local run_diagnostics=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --install-deps)
                auto_install=true
                shift
                ;;
            --skip-checks)
                skip_checks=true
                shift
                ;;
            --diagnose)
                run_diagnostics=true
                shift
                ;;
            --verify)
                # Pass through to camera manager
                break
                ;;
            --view)
                # Pass through to camera manager  
                break
                ;;
            --help)
                echo "Usage: $0 [options]"
                echo ""
                echo "Options:"
                echo "  --install-deps    Automatically install missing dependencies"
                echo "  --skip-checks     Skip system requirement checks"
                echo "  --diagnose        Run diagnostics and exit"
                echo "  --verify          Run camera verification mode (10s CLI test)"
                echo "  --view            Show actual video output (30s visual test)"
                echo "  --help           Show this help"
                echo ""
                echo "The script will:"
                echo "1. Check and request sudo permissions"
                echo "2. Verify system requirements"
                echo "3. Check USB connection"
                echo "4. Build camera manager if needed"
                echo "5. Clean up conflicting processes"
                echo "6. Launch camera utility"
                exit 0
                ;;
            *)
                # Pass unknown arguments to camera manager
                break
                ;;
        esac
    done
    
    # Ensure we have root permissions
    check_sudo "$@"
    
    # Run diagnostics if requested
    if [ "$run_diagnostics" = true ]; then
        cd "$SCRIPT_DIR"
        make diagnose 2>/dev/null || true
        exit 0
    fi
    
    # Check system requirements
    if [ "$skip_checks" = false ]; then
        if ! check_requirements; then
            if [ "$auto_install" = true ]; then
                install_dependencies
            else
                log_error "Use --install-deps to automatically install missing packages"
                exit 1
            fi
        fi
    fi
    
    # Check USB connection
    if ! check_usb_connection; then
        log_error "Cannot proceed without camera connection"
        log_info "Resolve USB issues and try again"
        exit 1
    fi
    
    # Build if necessary
    build_camera_manager
    
    # Cleanup processes
    cleanup_processes
    
    # Launch camera manager
    log_header "🚀 LAUNCHING CAMERA MANAGER"
    echo ""
    
    # Set LD_LIBRARY_PATH for Camera SDK
    local sdk_lib_path="$SCRIPT_DIR/../../vision_system/CameraSDK-20250418_145834-2.0.2-Linux/lib"
    if [ -d "$sdk_lib_path" ]; then
        export LD_LIBRARY_PATH="$sdk_lib_path:$LD_LIBRARY_PATH"
        log_info "Camera SDK library path set: $sdk_lib_path"
    fi
    
    # Execute camera manager with any remaining arguments
    log_info "Starting camera manager..."
    
    # Convert wrapper script options to C++ program options
    args_for_camera=()
    for arg in "$@"; do
        if [ "$arg" = "--verify" ]; then
            args_for_camera+=("--verify-only")
        elif [ "$arg" = "--view" ]; then
            args_for_camera+=("--view-only")
        else
            args_for_camera+=("$arg")
        fi
    done
    
    exec "$CAMERA_MANAGER" "${args_for_camera[@]}"
}

# Trap signals for clean exit
trap 'log_warning "Script interrupted"; exit 130' INT TERM

# Run main function
main "$@"