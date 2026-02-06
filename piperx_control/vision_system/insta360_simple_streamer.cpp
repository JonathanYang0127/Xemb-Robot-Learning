#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <cstring>
#include <memory>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <fstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <camera/camera.h>
#include <camera/device_discovery.h>

// Global variables
std::shared_ptr<ins_camera::Camera> g_cam = nullptr;
bool g_running = true;

// Stream configuration
const int HTTP_PORT = 8080;
const int MAX_CLIENTS = 5;
const int STREAM_WIDTH = 1440;  // Updated to match working resolution
const int STREAM_HEIGHT = 720;  // Updated to match working resolution

// H264 buffer management
struct H264Frame {
    std::vector<uint8_t> data;
    int64_t timestamp;
};

std::queue<H264Frame> g_frame_queue;
std::mutex g_queue_mutex;
std::condition_variable g_queue_cv;
const size_t MAX_QUEUE_SIZE = 30; // About 1 second of video at 30fps

// Simple HTTP streaming server
class HTTPStreamServer {
private:
    int server_fd;
    std::vector<std::thread> client_threads;
    
public:
    HTTPStreamServer() : server_fd(-1) {}
    
    ~HTTPStreamServer() {
        stop();
    }
    
    bool start(int port) {
        // Create socket
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }
        
        // Allow socket reuse
        int opt = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "Failed to set socket options" << std::endl;
            close(server_fd);
            return false;
        }
        
        // Bind
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);
        
        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            std::cerr << "Failed to bind to port " << port << std::endl;
            close(server_fd);
            return false;
        }
        
        // Listen
        if (listen(server_fd, MAX_CLIENTS) < 0) {
            std::cerr << "Failed to listen on socket" << std::endl;
            close(server_fd);
            return false;
        }
        
        std::cout << "✅ HTTP server listening on port " << port << std::endl;
        
        // Start accept thread
        std::thread accept_thread(&HTTPStreamServer::accept_clients, this);
        accept_thread.detach();
        
        return true;
    }
    
    void stop() {
        if (server_fd >= 0) {
            close(server_fd);
            server_fd = -1;
        }
    }
    
private:
    void accept_clients() {
        while (g_running && server_fd >= 0) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
            if (client_fd < 0) {
                if (g_running) {
                    std::cerr << "Failed to accept client" << std::endl;
                }
                continue;
            }
            
            std::cout << "📱 New client connected from " 
                      << inet_ntoa(client_addr.sin_addr) << std::endl;
            
            // Handle client in new thread
            std::thread client_thread(&HTTPStreamServer::handle_client, this, client_fd);
            client_thread.detach();
        }
    }
    
    void handle_client(int client_fd) {
        // Read HTTP request (we'll ignore it for simplicity)
        char buffer[1024];
        recv(client_fd, buffer, sizeof(buffer), 0);
        
        // Send HTTP response headers for streaming
        const char* response = 
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/octet-stream\r\n"
            "Cache-Control: no-cache, no-store, must-revalidate\r\n"
            "Pragma: no-cache\r\n"
            "Expires: 0\r\n"
            "Connection: close\r\n"
            "\r\n";
        
        send(client_fd, response, strlen(response), 0);
        
        // Stream H264 data to client
        while (g_running) {
            std::unique_lock<std::mutex> lock(g_queue_mutex);
            
            // Wait for frames
            g_queue_cv.wait_for(lock, std::chrono::milliseconds(100), 
                               []{ return !g_frame_queue.empty() || !g_running; });
            
            if (!g_running) break;
            
            if (!g_frame_queue.empty()) {
                H264Frame frame = g_frame_queue.front();
                g_frame_queue.pop();
                lock.unlock();
                
                // Send frame data
                ssize_t sent = send(client_fd, frame.data.data(), frame.data.size(), MSG_NOSIGNAL);
                if (sent < 0) {
                    std::cout << "📱 Client disconnected" << std::endl;
                    break;
                }
            }
        }
        
        close(client_fd);
    }
};

class SimpleStreamDelegate : public ins_camera::StreamDelegate {
public:
    SimpleStreamDelegate() = default;
    virtual ~SimpleStreamDelegate() = default;

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        // We're only handling video
    }

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        if (stream_index == 0 && g_running) {
            // Create H264 frame
            H264Frame frame;
            frame.data.resize(size);
            memcpy(frame.data.data(), data, size);
            frame.timestamp = timestamp;
            
            // Add to queue
            {
                std::lock_guard<std::mutex> lock(g_queue_mutex);
                
                // Drop old frames if queue is full
                while (g_frame_queue.size() >= MAX_QUEUE_SIZE) {
                    g_frame_queue.pop();
                }
                
                g_frame_queue.push(std::move(frame));
            }
            g_queue_cv.notify_all();
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {}
    void OnExposureData(const ins_camera::ExposureData& data) override {}
};

void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        std::cout << "\nReceived signal " << sig << ", shutting down..." << std::endl;
        g_running = false;
        g_queue_cv.notify_all();
        if (g_cam) {
            g_cam->Close();
        }
    }
}

int main(int argc, char* argv[]) {
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "🎥 Insta360 X3 Simple HTTP Streamer" << std::endl;
    std::cout << "====================================" << std::endl;
    
    // Parse command line arguments
    int port = HTTP_PORT;
    bool use_low_res = false;
    
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--low-res") == 0) {
            use_low_res = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --port <port>   HTTP server port (default: 8080)" << std::endl;
            std::cout << "  --low-res       Use lower resolution (1440x720)" << std::endl;
            std::cout << "  --help          Show this help" << std::endl;
            return 0;
        }
    }
    
    // Initialize camera
    ins_camera::DeviceDiscovery discovery;
    auto list = discovery.GetAvailableDevices();
    
    if (list.empty()) {
        std::cerr << "❌ No Insta360 camera found. Please connect your X3." << std::endl;
        return -1;
    }
    
    std::cout << "📷 Found camera: " << list[0].camera_name 
              << " (Serial: " << list[0].serial_number << ")" << std::endl;
    
    // Open camera
    g_cam = std::make_shared<ins_camera::Camera>(list[0].info);
    if (!g_cam->Open()) {
        std::cerr << "❌ Failed to open camera" << std::endl;
        return -1;
    }
    
    discovery.FreeDeviceDescriptors(list);
    
    // Create HTTP server
    HTTPStreamServer server;
    if (!server.start(port)) {
        g_cam->Close();
        return -1;
    }
    
    // Create and set stream delegate
    std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<SimpleStreamDelegate>();
    g_cam->SetStreamDelegate(delegate);
    
    // Configure live stream parameters
    ins_camera::LiveStreamParam param;
    if (use_low_res) {
        param.video_resolution = ins_camera::VideoResolution::RES_1440_720P30;
        std::cout << "📺 Using low resolution: 1440x720 @ 30fps" << std::endl;
    } else {
        // Use 1440x720 as default since 3840x1920 times out on some cameras
        param.video_resolution = ins_camera::VideoResolution::RES_1440_720P30;
        std::cout << "📺 Using resolution: 1440x720 @ 30fps" << std::endl;
    }
    param.video_bitrate = 1024 * 1024 * 2; // 2 Mbps (reduced for stability)
    param.enable_audio = false;
    param.using_lrv = false;
    
    // Start live streaming
    if (!g_cam->StartLiveStreaming(param)) {
        std::cerr << "❌ Failed to start live streaming" << std::endl;
        g_cam->Close();
        return -1;
    }
    
    std::cout << "✅ Streaming started!" << std::endl;
    std::cout << "\n🎬 To view in VLC:" << std::endl;
    std::cout << "   1. Open VLC → Media → Open Network Stream" << std::endl;
    std::cout << "   2. Enter: http://localhost:" << port << std::endl;
    std::cout << "   3. Click Play" << std::endl;
    std::cout << "\n💡 Tips:" << std::endl;
    std::cout << "   - You can also use: http://<your-ip>:" << port << " from other devices" << std::endl;
    std::cout << "   - Use --low-res flag for better performance" << std::endl;
    std::cout << "   - Multiple clients can connect simultaneously" << std::endl;
    std::cout << "\nPress Ctrl+C to stop streaming..." << std::endl;
    
    // Keep running until interrupted
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // Cleanup
    std::cout << "\n🛑 Stopping stream..." << std::endl;
    g_cam->StopLiveStreaming();
    g_cam->Close();
    
    std::cout << "✅ Cleanup complete" << std::endl;
    return 0;
} 