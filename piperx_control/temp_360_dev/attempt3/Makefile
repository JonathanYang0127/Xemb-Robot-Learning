# Makefile for Insta360 X3 Camera Manager - Attempt 3
# Robust camera utility for AI agent environments

# Project settings
TARGET = camera_manager
SOURCES = camera_manager.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Camera SDK paths - adjust these to match your SDK installation
CAMERA_SDK_DIR = ../../vision_system/CameraSDK-20250418_145834-2.0.2-Linux
CAMERA_SDK_INCLUDE = $(CAMERA_SDK_DIR)/include
CAMERA_SDK_LIB = $(CAMERA_SDK_DIR)/lib

# Compiler settings
CXX = g++
CXXFLAGS = -std=c++14 -Wall -Wextra -O2
INCLUDES = -I$(CAMERA_SDK_INCLUDE) -I.
LIBS = -L$(CAMERA_SDK_LIB) -lCameraSDK -lusb-1.0 -ludev -lpthread -ldl

# Debug build settings
DEBUG_FLAGS = -g -DDEBUG -O0
RELEASE_FLAGS = -DNDEBUG -O2

# Default to release build
BUILD_TYPE ?= release

ifeq ($(BUILD_TYPE),debug)
    CXXFLAGS += $(DEBUG_FLAGS)
else
    CXXFLAGS += $(RELEASE_FLAGS)
endif

# Build rules
.PHONY: all clean debug release install check-deps

all: check-deps $(TARGET)

$(TARGET): $(OBJECTS)
	@echo "🔗 Linking $(TARGET)..."
	$(CXX) $(OBJECTS) -o $(TARGET) $(LIBS)
	@echo "✅ Build complete: $(TARGET)"

%.o: %.cpp
	@echo "🔧 Compiling $<..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

debug:
	$(MAKE) BUILD_TYPE=debug

release:
	$(MAKE) BUILD_TYPE=release

clean:
	@echo "🧹 Cleaning build files..."
	rm -f $(OBJECTS) $(TARGET)
	@echo "✅ Clean complete"

# Check dependencies
check-deps:
	@echo "🔍 Checking dependencies..."
	@if [ ! -d "$(CAMERA_SDK_DIR)" ]; then \
		echo "❌ Camera SDK not found at $(CAMERA_SDK_DIR)"; \
		echo "   Please ensure Camera SDK is installed"; \
		exit 1; \
	fi
	@if [ ! -f "$(CAMERA_SDK_LIB)/libCameraSDK.so" ]; then \
		echo "❌ Camera SDK library not found"; \
		echo "   Expected: $(CAMERA_SDK_LIB)/libCameraSDK.so"; \
		exit 1; \
	fi
	@echo "✅ Dependencies check passed"

# Install system dependencies
install-deps:
	@echo "📦 Installing system dependencies..."
	sudo apt-get update
	sudo apt-get install -y libusb-1.0-0-dev libudev-dev build-essential
	@echo "✅ System dependencies installed"

# System checks
system-check:
	@echo "🔍 SYSTEM REQUIREMENTS CHECK"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo -n "libusb-1.0: "
	@if pkg-config --exists libusb-1.0; then echo "✅ Found"; else echo "❌ Missing"; fi
	@echo -n "libudev: "
	@if pkg-config --exists libudev; then echo "✅ Found"; else echo "❌ Missing"; fi
	@echo -n "USB utils: "
	@if which lsusb > /dev/null; then echo "✅ Found"; else echo "❌ Missing"; fi
	@echo -n "Sudo permissions: "
	@if [ "$$(id -u)" = "0" ]; then echo "✅ Root"; else echo "⚠️  Not root (will need sudo)"; fi
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# USB device check
usb-check:
	@echo "🔌 USB DEVICE CHECK"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo "All USB devices:"
	@lsusb || echo "❌ lsusb command not found"
	@echo ""
	@echo "Insta360 devices (Vendor ID 2e1a):"
	@lsusb | grep -i "2e1a\|insta360" || echo "❌ No Insta360 devices found"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Test build and run
test: $(TARGET)
	@echo "🧪 Running camera utility test..."
	sudo ./$(TARGET) --status-only
	@echo "✅ Test complete"

# Full diagnostic
diagnose: system-check usb-check
	@echo ""
	@echo "📋 TROUBLESHOOTING GUIDE"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo "If camera not detected:"
	@echo "1. Check USB cable connection"
	@echo "2. Ensure camera is powered on"
	@echo "3. Set camera to Android mode:"
	@echo "   Camera Settings → General → USB Mode → Android"
	@echo "4. Try different USB port/cable"
	@echo "5. Restart camera and reconnect"
	@echo ""
	@echo "If permission errors:"
	@echo "1. Run with sudo: sudo ./$(TARGET)"
	@echo "2. Or add user to dialout group:"
	@echo "   sudo usermod -a -G dialout $$USER"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Help
help:
	@echo "🎥 Insta360 X3 Camera Manager - Build System"
	@echo ""
	@echo "Available targets:"
	@echo "  all          - Build the camera manager (default)"
	@echo "  debug        - Build with debug symbols"
	@echo "  release      - Build optimized release version"
	@echo "  clean        - Remove build files"
	@echo "  install-deps - Install system dependencies"
	@echo "  system-check - Check system requirements"
	@echo "  usb-check    - Check for USB devices"
	@echo "  test         - Build and run basic test"
	@echo "  diagnose     - Run full diagnostic"
	@echo "  help         - Show this help"
	@echo ""
	@echo "Usage examples:"
	@echo "  make                    # Build release version"
	@echo "  make debug              # Build debug version"
	@echo "  make install-deps       # Install dependencies"
	@echo "  make diagnose           # Run diagnostics"
	@echo "  sudo ./camera_manager   # Run the utility"