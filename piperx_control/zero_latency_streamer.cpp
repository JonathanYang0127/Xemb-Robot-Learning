#include <iostream>
#include <thread>
#include <regex>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>
#include <unistd.h>

std::shared_ptr<ins_camera::Camera> cam = nullptr;

// ZERO LATENCY STREAM DELEGATE - Pipes H.264 directly to stdout
class ZeroLatencyStreamDelegate : public ins_camera::StreamDelegate {
public:
    ZeroLatencyStreamDelegate() = default;

    virtual ~ZeroLatencyStreamDelegate() {
        std::cerr << "🔚 Stream delegate destroyed" << std::endl;
    }

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        // Skip audio for now to minimize latency
    }

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        // CRITICAL: Stream 0 is the main video stream - send directly to stdout
        if (stream_index == 0 && data && size > 0) {
            // Write raw H.264 data directly to stdout (no file buffering!)
            ssize_t written = write(STDOUT_FILENO, data, size);
            if (written != static_cast<ssize_t>(size)) {
                std::cerr << "⚠️  Warning: Only wrote " << written << " of " << size << " bytes" << std::endl;
            }
            
            // Flush immediately to minimize latency
            fsync(STDOUT_FILENO);
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {}
    void OnExposureData(const ins_camera::ExposureData& data) override {}
};

void signalHandle(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        std::cerr << "🛑 Signal " << sig << " received, shutting down..." << std::endl;
        if (cam) {
            cam->Close();
        }
        exit(0);
    }
}

int main() {
    std::cerr << "🚀 Zero Latency Insta360 X3 Streamer Starting..." << std::endl;
    
    signal(SIGINT, signalHandle);
    signal(SIGTERM, signalHandle);

    // Discover camera
    auto cam_list = ins_camera::DeviceDiscovery::GetAvailableDevices();
    if (cam_list.empty()) {
        std::cerr << "❌ No camera found!" << std::endl;
        return 1;
    }

    std::cerr << "✅ Found camera: " << cam_list[0].camera_name 
              << " (Serial: " << cam_list[0].serial_number << ")" << std::endl;

    // Create camera instance
    cam = std::make_shared<ins_camera::Camera>(cam_list[0].info);
    
    // Open camera
    if (!cam->Open()) {
        std::cerr << "❌ Failed to open camera!" << std::endl;
        return 1;
    }

    std::cerr << "✅ Camera opened successfully" << std::endl;

    // Create zero-latency stream delegate
    auto stream_delegate = std::make_shared<ZeroLatencyStreamDelegate>();
    
    // Start live streaming
    std::cerr << "🎬 Starting zero-latency live stream..." << std::endl;
    std::cerr << "📺 Raw H.264 data will be streamed to stdout" << std::endl;
    std::cerr << "⚡ No file buffering - direct camera → stdout → ffplay!" << std::endl;
    
    if (!cam->StartLiveStreaming(stream_delegate)) {
        std::cerr << "❌ Failed to start live streaming!" << std::endl;
        return 1;
    }

    std::cerr << "✅ Live streaming started successfully!" << std::endl;
    std::cerr << "📡 Streaming raw H.264 data... (Ctrl+C to stop)" << std::endl;

    // Keep streaming until interrupted
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}