#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

// Include Insta360 CameraSDK headers
#include <camera/camera.h>
#include <camera/device_discovery.h>

// Configuration for low latency streaming
const int UDP_PORT = 8082;  // Port to send video data
const char* UDP_ADDRESS = "127.0.0.1";
const int MAX_PACKET_SIZE = 1400;  // Safe UDP packet size
const int MAX_QUEUE_SIZE = 10;     // Limit buffering for low latency

// Global variables
std::shared_ptr<ins_camera::Camera> g_camera = nullptr;
bool g_running = true;
int g_udp_socket = -1;
struct sockaddr_in g_server_addr;

// Thread-safe video data queue
struct VideoPacket {
    std::vector<uint8_t> data;
    int64_t timestamp;
    uint8_t stream_type;
    int stream_index;
};

class UDPVideoStreamer : public ins_camera::StreamDelegate {
private:
    std::queue<VideoPacket> video_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread sender_thread_;
    uint32_t packet_id_;
    
public:
    UDPVideoStreamer() : packet_id_(0) {
        sender_thread_ = std::thread(&UDPVideoStreamer::SendThread, this);
    }
    
    ~UDPVideoStreamer() {
        g_running = false;
        queue_cv_.notify_all();
        if (sender_thread_.joinable()) {
            sender_thread_.join();
        }
    }
    
    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        // Audio not needed for 360 view
    }
    
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        // Only process video data from both lenses (dual fisheye)
        if (stream_index == 0 || stream_index == 1) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            
            // Limit queue size for low latency
            while (video_queue_.size() >= MAX_QUEUE_SIZE) {
                video_queue_.pop();
            }
            
            VideoPacket packet;
            packet.data = std::vector<uint8_t>(data, data + size);
            packet.timestamp = timestamp;
            packet.stream_type = streamType;
            packet.stream_index = stream_index;
            
            video_queue_.push(std::move(packet));
            queue_cv_.notify_one();
            
            if (video_queue_.size() % 30 == 0) {  // Log every 30 packets
                std::cout << "Stream " << stream_index << " queue size: " << video_queue_.size() 
                         << ", packet size: " << size << " bytes" << std::endl;
            }
        }
    }
    
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {}
    void OnExposureData(const ins_camera::ExposureData& data) override {}
    
private:
    void SendThread() {
        std::cout << "UDP sender thread started" << std::endl;
        
        while (g_running) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !video_queue_.empty() || !g_running; });
            
            if (!g_running) break;
            
            VideoPacket packet = std::move(video_queue_.front());
            video_queue_.pop();
            lock.unlock();
            
            SendVideoPacket(packet);
        }
        
        std::cout << "UDP sender thread stopped" << std::endl;
    }
    
    void SendVideoPacket(const VideoPacket& packet) {
        // Create header for the packet
        struct PacketHeader {
            uint32_t packet_id;
            uint32_t stream_index;
            uint32_t total_size;
            uint32_t chunk_index;
            uint32_t total_chunks;
            int64_t timestamp;
        } __attribute__((packed));
        
        const size_t header_size = sizeof(PacketHeader);
        const size_t payload_size = MAX_PACKET_SIZE - header_size;
        const uint32_t total_chunks = (packet.data.size() + payload_size - 1) / payload_size;
        
        for (uint32_t chunk = 0; chunk < total_chunks; ++chunk) {
            PacketHeader header;
            header.packet_id = packet_id_++;
            header.stream_index = packet.stream_index;
            header.total_size = packet.data.size();
            header.chunk_index = chunk;
            header.total_chunks = total_chunks;
            header.timestamp = packet.timestamp;
            
            // Calculate chunk data
            size_t chunk_start = chunk * payload_size;
            size_t chunk_size = std::min(payload_size, packet.data.size() - chunk_start);
            
            // Create UDP packet
            std::vector<uint8_t> udp_packet(header_size + chunk_size);
            memcpy(udp_packet.data(), &header, header_size);
            memcpy(udp_packet.data() + header_size, packet.data.data() + chunk_start, chunk_size);
            
            // Send UDP packet
            ssize_t sent = sendto(g_udp_socket, udp_packet.data(), udp_packet.size(), 0,
                                (struct sockaddr*)&g_server_addr, sizeof(g_server_addr));
            
            if (sent < 0) {
                std::cerr << "Error sending UDP packet: " << strerror(errno) << std::endl;
                break;
            }
        }
    }
};

void signalHandler(int signal) {
    std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
    
    if (g_camera) {
        g_camera->StopLiveStreaming();
        g_camera->Close();
    }
    
    if (g_udp_socket >= 0) {
        close(g_udp_socket);
    }
}

bool setupUDP() {
    g_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_socket < 0) {
        std::cerr << "Failed to create UDP socket: " << strerror(errno) << std::endl;
        return false;
    }
    
    memset(&g_server_addr, 0, sizeof(g_server_addr));
    g_server_addr.sin_family = AF_INET;
    g_server_addr.sin_port = htons(UDP_PORT);
    g_server_addr.sin_addr.s_addr = inet_addr(UDP_ADDRESS);
    
    std::cout << "UDP socket configured to send to " << UDP_ADDRESS << ":" << UDP_PORT << std::endl;
    return true;
}

bool initializeCamera() {
    std::cout << "Initializing Insta360 camera..." << std::endl;
    
    ins_camera::SetLogLevel(ins_camera::LogLevel::ERR);
    
    ins_camera::DeviceDiscovery discovery;
    auto device_list = discovery.GetAvailableDevices();
    
    if (device_list.empty()) {
        std::cerr << "No Insta360 camera found!" << std::endl;
        return false;
    }
    
    // Print discovered cameras
    for (const auto& device : device_list) {
        std::cout << "Found camera - Serial: " << device.serial_number 
                 << ", Type: " << device.camera_name 
                 << ", FW: " << device.fw_version << std::endl;
    }
    
    // Use the first camera
    g_camera = std::make_shared<ins_camera::Camera>(device_list[0].info);
    
    if (!g_camera->Open()) {
        std::cerr << "Failed to open camera!" << std::endl;
        discovery.FreeDeviceDescriptors(device_list);
        return false;
    }
    
    discovery.FreeDeviceDescriptors(device_list);
    
    std::cout << "Camera opened successfully" << std::endl;
    return true;
}

bool startLiveStreaming() {
    auto streamer = std::make_shared<UDPVideoStreamer>();
    std::shared_ptr<ins_camera::StreamDelegate> delegate = streamer;
    g_camera->SetStreamDelegate(delegate);
    
    // Configure live streaming parameters for X3
    ins_camera::LiveStreamParam param;
    param.video_resolution = ins_camera::VideoResolution::RES_1440_720P30;  // Better for X3
    param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
    param.video_bitrate = 1024 * 1024 / 2;  // 0.5MB bitrate for low latency
    param.enable_audio = false;  // No audio needed
    param.using_lrv = false;     // Use full resolution
    
    std::cout << "Starting live streaming with resolution 1440x720@30fps..." << std::endl;
    
    if (!g_camera->StartLiveStreaming(param)) {
        std::cerr << "Failed to start live streaming!" << std::endl;
        return false;
    }
    
    std::cout << "Live streaming started successfully" << std::endl;
    std::cout << "Streaming dual fisheye video to UDP " << UDP_ADDRESS << ":" << UDP_PORT << std::endl;
    return true;
}

int main(int argc, char* argv[]) {
    std::cout << "=== Insta360 X3 to Vuer UDP Streamer ===" << std::endl;
    std::cout << "Low latency dual fisheye video streaming for VR teleop" << std::endl;
    
    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Initialize components
    if (!setupUDP()) {
        return -1;
    }
    
    if (!initializeCamera()) {
        return -1;
    }
    
    if (!startLiveStreaming()) {
        return -1;
    }
    
    std::cout << "=== System Ready ===" << std::endl;
    std::cout << "Press Ctrl+C to stop streaming" << std::endl;
    
    // Keep running until signal
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    std::cout << "Shutting down..." << std::endl;
    
    if (g_camera) {
        g_camera->StopLiveStreaming();
        g_camera->Close();
    }
    
    if (g_udp_socket >= 0) {
        close(g_udp_socket);
    }
    
    std::cout << "Shutdown complete" << std::endl;
    return 0;
}