# Makefile for Insta360 X3 to Vuer 360° Video Streaming
# Builds the C++ UDP streamer using CameraSDK

# Paths
CAMERA_SDK_PATH = ../../vision_system/CameraSDK-20250418_145834-2.0.2-Linux
INCLUDE_PATH = $(CAMERA_SDK_PATH)/include
LIB_PATH = $(CAMERA_SDK_PATH)/lib

# Compiler settings
CXX = g++
CXXFLAGS = -std=c++14 -Wall -O3 -pthread
INCLUDES = -I$(INCLUDE_PATH)
LIBS = -L$(LIB_PATH) -lCameraSDK -lpthread

# Output binary
TARGET = attempt2_insta360_vuer_streamer
SOURCE = attempt2_insta360_vuer_streamer.cpp

# Default target
all: $(TARGET)

# Build the main streamer
$(TARGET): $(SOURCE)
	@echo "🔨 Building Insta360 to Vuer UDP streamer..."
	@echo "📁 CameraSDK path: $(CAMERA_SDK_PATH)"
	@echo "📚 Include path: $(INCLUDE_PATH)"
	@echo "🔗 Library path: $(LIB_PATH)"
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SOURCE) $(LIBS)
	@echo "✅ Built $(TARGET) successfully!"

# Check if CameraSDK is available
check-sdk:
	@echo "🔍 Checking CameraSDK availability..."
	@if [ ! -d "$(CAMERA_SDK_PATH)" ]; then \
		echo "❌ CameraSDK not found at $(CAMERA_SDK_PATH)"; \
		echo "💡 Please ensure the CameraSDK is properly installed"; \
		exit 1; \
	fi
	@if [ ! -f "$(INCLUDE_PATH)/camera/camera.h" ]; then \
		echo "❌ CameraSDK headers not found"; \
		exit 1; \
	fi
	@if [ ! -f "$(LIB_PATH)/libCameraSDK.so" ] && [ ! -f "$(LIB_PATH)/libCameraSDK.a" ]; then \
		echo "❌ CameraSDK library not found"; \
		echo "📁 Looking for libCameraSDK.so or libCameraSDK.a in $(LIB_PATH)"; \
		ls -la $(LIB_PATH)/ || echo "❌ Library directory not accessible"; \
		exit 1; \
	fi
	@echo "✅ CameraSDK found and appears complete"

# Clean build artifacts
clean:
	@echo "🧹 Cleaning build artifacts..."
	rm -f $(TARGET)
	@echo "✅ Clean complete"

# Install Python dependencies
install-python-deps:
	@echo "📦 Installing Python dependencies for fisheye converter..."
	pip install opencv-python numpy
	@echo "✅ Python dependencies installed"

# Test the camera connection
test-camera: $(TARGET)
	@echo "🎥 Testing camera connection..."
	@echo "⚠️  Make sure Insta360 X3 is connected via USB"
	@echo "⏰ This test will run for 10 seconds then exit"
	timeout 10s ./$(TARGET) || echo "🔚 Camera test completed (or timed out)"

# Setup library path for runtime
setup-runtime:
	@echo "🔧 Setting up runtime library path..."
	@echo "export LD_LIBRARY_PATH=$(LIB_PATH):\$$LD_LIBRARY_PATH" > setup_env.sh
	@chmod +x setup_env.sh
	@echo "✅ Created setup_env.sh - source this before running"

# Full build and setup
setup: check-sdk install-python-deps $(TARGET) setup-runtime
	@echo "🎉 Setup complete!"
	@echo ""
	@echo "📋 Next steps:"
	@echo "  1. Connect your Insta360 X3 camera via USB"
	@echo "  2. Source the environment: source setup_env.sh"
	@echo "  3. Start the streaming pipeline: make run-pipeline"
	@echo ""

# Run the complete streaming pipeline
run-pipeline: $(TARGET)
	@echo "🚀 Starting complete 360° video streaming pipeline..."
	@echo "⚠️  Make sure:"
	@echo "   - Insta360 X3 is connected via USB"
	@echo "   - You have sourced setup_env.sh"
	@echo "   - No other applications are using the camera"
	@echo ""
	@echo "🎬 Pipeline components:"
	@echo "   1. Camera UDP Streamer (dual fisheye)"
	@echo "   2. Fisheye to Equirectangular Converter"
	@echo "   3. VR Server with 360° Video"
	@echo ""
	@echo "Press Ctrl+C to stop all components"
	@echo ""
	@# Start camera streamer in background
	./$(TARGET) & \
	CAMERA_PID=$$!; \
	sleep 3; \
	echo "📹 Camera streamer started (PID: $$CAMERA_PID)"; \
	python3 attempt2_fisheye_to_equirect.py & \
	CONVERTER_PID=$$!; \
	sleep 2; \
	echo "🔄 Fisheye converter started (PID: $$CONVERTER_PID)"; \
	python3 attempt2_vr_server_with_360.py & \
	VR_PID=$$!; \
	echo "🎮 VR server started (PID: $$VR_PID)"; \
	echo ""; \
	echo "🎉 All components running!"; \
	echo "💡 Access VR server via ngrok URL on Meta Quest"; \
	echo ""; \
	trap 'echo "🛑 Stopping all components..."; kill $$CAMERA_PID $$CONVERTER_PID $$VR_PID 2>/dev/null; exit' INT; \
	wait

# Individual component runners
run-camera: $(TARGET)
	@echo "📹 Starting camera UDP streamer only..."
	source setup_env.sh && ./$(TARGET)

run-converter:
	@echo "🔄 Starting fisheye converter only..."
	python3 attempt2_fisheye_to_equirect.py

run-vr-server:
	@echo "🎮 Starting VR server with 360° video only..."
	python3 attempt2_vr_server_with_360.py

# Debug information
debug-info:
	@echo "🔍 Debug Information:"
	@echo "  CameraSDK Path: $(CAMERA_SDK_PATH)"
	@echo "  Include Path: $(INCLUDE_PATH)"
	@echo "  Library Path: $(LIB_PATH)"
	@echo "  Target: $(TARGET)"
	@echo ""
	@echo "📁 CameraSDK Directory Contents:"
	@ls -la $(CAMERA_SDK_PATH)/ || echo "❌ CameraSDK directory not accessible"
	@echo ""
	@echo "📚 Include Directory Contents:"
	@ls -la $(INCLUDE_PATH)/ || echo "❌ Include directory not accessible"
	@echo ""
	@echo "🔗 Library Directory Contents:"
	@ls -la $(LIB_PATH)/ || echo "❌ Library directory not accessible"

# Help target
help:
	@echo "🔧 Insta360 X3 to Vuer 360° Video Streaming Build System"
	@echo ""
	@echo "📋 Available targets:"
	@echo "  all              - Build the C++ UDP streamer (default)"
	@echo "  check-sdk        - Verify CameraSDK installation"
	@echo "  install-python-deps - Install Python dependencies"
	@echo "  setup            - Complete setup (check SDK, install deps, build)"
	@echo "  test-camera      - Test camera connection"
	@echo "  run-pipeline     - Run complete 360° streaming pipeline"
	@echo "  run-camera       - Run camera streamer only"
	@echo "  run-converter    - Run fisheye converter only"
	@echo "  run-vr-server    - Run VR server only"
	@echo "  clean            - Clean build artifacts"
	@echo "  debug-info       - Show debug information"
	@echo "  help             - Show this help"
	@echo ""
	@echo "🚀 Quick start:"
	@echo "  make setup       # One-time setup"
	@echo "  make run-pipeline # Start streaming"

.PHONY: all check-sdk install-python-deps clean test-camera setup-runtime setup run-pipeline run-camera run-converter run-vr-server debug-info help