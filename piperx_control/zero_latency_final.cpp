#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <csignal>
#include <unistd.h>
#include <camera/camera.h>
#include <camera/device_discovery.h>

std::shared_ptr<ins_camera::Camera> cam = nullptr;

// ZERO LATENCY STREAM DELEGATE - Direct H.264 to stdout
class ZeroLatencyDelegate : public ins_camera::StreamDelegate {
public:
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        // Stream 0 = main video - pipe directly to stdout with NO buffering
        if (stream_index == 0 && data && size > 0) {
            ssize_t written = write(STDOUT_FILENO, data, size);
            if (written != static_cast<ssize_t>(size)) {
                std::cerr << "⚠️  Write incomplete: " << written << "/" << size << std::endl;
            }
            // Immediate flush for zero latency
            fsync(STDOUT_FILENO);
        }
    }
    
    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {}
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {}
    void OnExposureData(const ins_camera::ExposureData& data) override {}
};

void signalHandler(int sig) {
    std::cerr << "🛑 Signal " << sig << " - shutting down..." << std::endl;
    if (cam) cam->Close();
    exit(0);
}

int main() {
    std::cerr << "🚀 ZERO LATENCY Insta360 X3 Streamer" << std::endl;
    std::cerr << "⚡ Direct camera → stdout pipeline (NO FILE I/O)" << std::endl;
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Discover camera
    ins_camera::DeviceDiscovery discovery;
    auto devices = discovery.GetAvailableDevices();
    if (devices.empty()) {
        std::cerr << "❌ No camera found!" << std::endl;
        return 1;
    }

    std::cerr << "✅ Found: " << devices[0].camera_name << std::endl;

    // Open camera
    cam = std::make_shared<ins_camera::Camera>(devices[0].info);
    if (!cam->Open()) {
        std::cerr << "❌ Failed to open camera!" << std::endl;
        return 1;
    }

    discovery.FreeDeviceDescriptors(devices);

    // Set zero-latency delegate
    auto delegate = std::make_shared<ZeroLatencyDelegate>();
    cam->SetStreamDelegate(delegate);

    // Configure for maximum performance
    ins_camera::LiveStreamParam param;
    param.video_resolution = ins_camera::VideoResolution::RES_3840_1920P30;
    param.video_bitrate = 1024 * 1024; // 1MB/s for responsiveness  
    param.enable_audio = false;        // Audio adds latency
    param.using_lrv = false;          // Use full resolution

    std::cerr << "🎬 Starting ZERO LATENCY stream..." << std::endl;
    
    if (!cam->StartLiveStreaming(param)) {
        std::cerr << "❌ Failed to start streaming!" << std::endl;
        return 1;
    }

    std::cerr << "✅ ZERO LATENCY streaming active!" << std::endl;
    std::cerr << "📡 H.264 data flowing directly to stdout..." << std::endl;

    // Keep running
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}