#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

// Camera SDK includes
#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>
#include <stream/stream_delegate.h>

/**
 * CAMERA CONFIGURATION - CRITICAL SETTINGS
 * Based on Camera SDK documentation and X3 testing
 */
namespace CameraConfig {
    // Preview stream settings - VALIDATED for Insta360 X3
    constexpr auto PREVIEW_RESOLUTION = ins_camera::VideoResolution::RES_1920_960P30;
    constexpr int32_t PREVIEW_BITRATE = 1024 * 1024 / 2; // 0.5MB - tested working
    constexpr bool ENABLE_AUDIO = false;
    constexpr bool USE_LRV = false; // Low resolution version disabled
    
    // Expected camera specs
    constexpr uint16_t INSTA360_USB_VENDOR_ID = 0x2e1a;
    constexpr const char* EXPECTED_CAMERA_TYPE = "X3";
    
    // Connection timeouts
    constexpr int CONNECTION_TIMEOUT_MS = 60000;  // 60 seconds - SDK can take 45s
    constexpr int DISCOVERY_TIMEOUT_MS = 10000;   // 10 seconds - reasonable for discovery
    constexpr int STREAM_START_TIMEOUT_MS = 2000;
    
    // Process management
    constexpr const char* CAMERA_PROCESS_PATTERN = "camera|insta360|Camera";
    constexpr int MAX_CLEANUP_ATTEMPTS = 3;
}

/**
 * Stream delegate for handling camera data
 * Receives dual fisheye H.264 streams from X3
 */
class CameraStreamDelegate : public ins_camera::StreamDelegate {
public:
    CameraStreamDelegate();
    virtual ~CameraStreamDelegate();
    
    // StreamDelegate interface
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, 
                     uint8_t streamType, int stream_index) override;
    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override;
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override;
    void OnExposureData(const ins_camera::ExposureData& data) override;
    
    // Stream management
    bool StartRecording(const std::string& output_prefix = "");
    void StopRecording();
    
    // Status
    size_t GetFrameCount() const;
    size_t GetTotalBytesReceived() const;
    bool IsReceivingData() const;
    
private:
    FILE* stream_file_ = nullptr;
    std::string output_prefix_;
    
    // Statistics  
    size_t frame_count_ = 0;
    size_t bytes_received_ = 0;
    std::chrono::steady_clock::time_point last_frame_time_;
    bool receiving_data_ = false;
    
    void CloseFiles();
    std::string GenerateTimestamp();
};

/**
 * Main camera manager class
 * Handles all camera operations with robust error handling
 */
class CameraManager {
public:
    CameraManager();
    ~CameraManager();
    
    // Main operations
    bool Initialize(bool force_cleanup = true);
    bool StartPreviewStream();
    bool StopPreviewStream(); 
    void Shutdown();
    
    // Verification and viewing modes
    bool RunVerificationMode(int duration_seconds = 10);
    bool RunViewingMode(int duration_seconds = 30);
    
    // Status and diagnostics
    bool IsConnected() const;
    bool IsStreaming() const;
    void PrintStatus() const;
    void PrintConfiguration() const;
    std::string GetCameraInfo() const;
    
    // Stream access
    std::shared_ptr<CameraStreamDelegate> GetStreamDelegate() const { return stream_delegate_; }
    
    // Error handling
    std::string GetLastError() const { return last_error_; }
    
private:
    // Core camera objects
    std::shared_ptr<ins_camera::Camera> camera_;
    std::shared_ptr<CameraStreamDelegate> stream_delegate_;
    ins_camera::DeviceDescriptor camera_descriptor_;
    
    // State tracking
    bool initialized_ = false;
    bool streaming_ = false;
    std::string last_error_;
    
    // System management
    bool CheckSudoPermissions();
    bool ValidateUSBConnection();
    bool KillConflictingProcesses();
    bool ResetUSBDevice();
    bool ForceKillInsta360Processes();
    bool DiscoverCamera();
    bool OpenCamera();
    bool ValidateCameraMode();
    
    // Utility methods
    void SetError(const std::string& error);
    void LogInfo(const std::string& message);
    void LogWarning(const std::string& message);
    void LogError(const std::string& message);
    
    // System commands
    std::string ExecuteCommand(const std::string& command);
    bool KillProcessByPattern(const std::string& pattern);
    std::vector<int> FindProcessesByPattern(const std::string& pattern);
};

/**
 * Utility functions for external use
 */
namespace CameraUtils {
    // System checks
    bool CheckUSBDevice(uint16_t vendor_id);
    bool CheckLibUSBInstalled();
    bool CheckSudoPermissions();
    
    // Process management
    std::vector<int> FindCameraProcesses();
    bool KillCameraProcesses();
    
    // Status helpers
    void PrintSystemInfo();
    void PrintUSBDevices();
    void PrintRequirements();
}

#endif // CAMERA_MANAGER_H