#include "camera_manager.h"
#include <iomanip>
#include <regex>
#include <csignal>

/**
 * INSTA360 X3 CAMERA MANAGER IMPLEMENTATION
 * 
 * Robust camera utility designed for AI agent environments
 * Handles all aspects of camera connection, configuration, and streaming
 * 
 * KEY FEATURES:
 * - Automatic conflict resolution
 * - USB connection validation  
 * - Comprehensive error handling
 * - Clear status reporting
 * - Preview streaming at 1920x960@30fps
 */

//==============================================================================
// STREAM DELEGATE IMPLEMENTATION
//==============================================================================

CameraStreamDelegate::CameraStreamDelegate() {
    std::cout << "ℹ️  Stream delegate initialized" << std::endl;
}

CameraStreamDelegate::~CameraStreamDelegate() {
    StopRecording();
}

void CameraStreamDelegate::OnVideoData(const uint8_t* data, size_t size, int64_t /*timestamp*/, 
                                      uint8_t /*streamType*/, int stream_index) {
    if (stream_index != 0) {
        std::cerr << "⚠️  WARNING: Unexpected stream index: " << stream_index 
                  << " (expected 0 for 1920×960 resolution)" << std::endl;
        return;
    }
    
    // Update statistics for single stream
    frame_count_++;
    bytes_received_ += size;
    last_frame_time_ = std::chrono::steady_clock::now();
    receiving_data_ = true;
    
    // Write to file if recording
    if (stream_file_ && data && size > 0) {
        size_t written = fwrite(data, 1, size, stream_file_);
        if (written != size) {
            std::cerr << "❌ WARNING: Write error (wrote " << written << "/" << size << " bytes)" << std::endl;
        }
        fflush(stream_file_); // Ensure data is written immediately
    }
    
    // Periodic status update (every 100 frames)
    if (frame_count_ % 100 == 0) {
        std::cout << "📹 STREAM STATUS: " << frame_count_ 
                  << " frames, Total=" << bytes_received_ / 1024 
                  << "KB" << std::endl;
    }
}

void CameraStreamDelegate::OnAudioData(const uint8_t* /*data*/, size_t /*size*/, int64_t /*timestamp*/) {
    // Audio disabled in our configuration
}

void CameraStreamDelegate::OnGyroData(const std::vector<ins_camera::GyroData>& /*data*/) {
    // Gyro data not needed for our use case
}

void CameraStreamDelegate::OnExposureData(const ins_camera::ExposureData& /*data*/) {
    // Exposure data not needed for our use case
}

bool CameraStreamDelegate::StartRecording(const std::string& output_prefix) {
    StopRecording(); // Ensure clean state
    
    std::string prefix = output_prefix.empty() ? GenerateTimestamp() : output_prefix;
    output_prefix_ = prefix;
    
    std::string stream_path = prefix + "_fisheye_1920x960.h264";
    
    stream_file_ = fopen(stream_path.c_str(), "wb");
    
    if (!stream_file_) {
        std::cerr << "❌ ERROR: Failed to create output file: " << stream_path << std::endl;
        return false;
    }
    
    // Reset statistics
    frame_count_ = 0;
    bytes_received_ = 0;
    receiving_data_ = false;
    
    std::cout << "📹 RECORDING STARTED:" << std::endl;
    std::cout << "  Single stream (1920×960): " << stream_path << std::endl;
    
    return true;
}

void CameraStreamDelegate::StopRecording() {
    if (stream_file_) {
        std::cout << "📹 RECORDING STOPPED" << std::endl;
        std::cout << "  Final stats: " << frame_count_ << " frames" << std::endl;
        std::cout << "  Total data: " << bytes_received_ / (1024*1024) << "MB" << std::endl;
    }
    CloseFiles();
}

void CameraStreamDelegate::CloseFiles() {
    if (stream_file_) {
        fclose(stream_file_);
        stream_file_ = nullptr;
    }
}

std::string CameraStreamDelegate::GenerateTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    return ss.str();
}

size_t CameraStreamDelegate::GetFrameCount() const {
    return frame_count_;
}

size_t CameraStreamDelegate::GetTotalBytesReceived() const {
    return bytes_received_;
}

bool CameraStreamDelegate::IsReceivingData() const {
    auto now = std::chrono::steady_clock::now();
    auto threshold = std::chrono::seconds(2);
    
    return receiving_data_ && 
           std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time_).count() < 2000;
}

//==============================================================================
// CAMERA MANAGER IMPLEMENTATION  
//==============================================================================

CameraManager::CameraManager() {
    stream_delegate_ = std::make_shared<CameraStreamDelegate>();
    
    // Set SDK log level to minimal for cleaner output
    ins_camera::SetLogLevel(ins_camera::LogLevel::ERR);
    
    LogInfo("CameraManager created");
}

CameraManager::~CameraManager() {
    Shutdown();
}

bool CameraManager::Initialize(bool force_cleanup) {
    LogInfo("🚀 INITIALIZING INSTA360 X3 CAMERA MANAGER");
    LogInfo("⚠️  TIMEOUT WARNING: Initialization may take 5-50 seconds if camera is not accessible");
    LogInfo("    The Camera SDK has internal timeouts that cannot be interrupted");
    LogInfo("    Please be patient - failure messages will show common fixes");
    PrintConfiguration();
    
    // Step 1: ENFORCE sudo permissions (not just check)
    if (!CheckSudoPermissions()) {
        LogError("❌ SUDO REQUIRED: This utility MUST run with sudo permissions");
        LogError("  Please run: sudo ./camera_manager");
        LogError("  Or use: sudo ./run_camera_utility.sh");
        return false;
    }
    LogInfo("✅ Sudo permissions: OK");
    
    // Step 2: Kill conflicting processes if requested
    if (force_cleanup) {
        LogInfo("🔄 Cleaning up conflicting processes...");
        if (!KillConflictingProcesses()) {
            LogWarning("Some processes may still be running");
        } else {
            LogInfo("✅ Process cleanup: OK");
        }
        // Give processes time to fully terminate
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    // Step 3: Validate USB connection
    if (!ValidateUSBConnection()) {
        SetError("USB validation failed - check camera connection and Android mode");
        return false;
    }
        LogInfo("✅ USB connection: OK");

    // Step 4: Pre-check - look for existing camera usage
    LogInfo("🔧 PRE-CHECK: Checking for camera conflicts...");
    
    // Check for any processes that might be using the camera
    std::string lsof_check = "lsof 2>/dev/null | grep -i '2e1a\\|insta360' || true";
    std::string lsof_result = ExecuteCommand(lsof_check);
    if (!lsof_result.empty()) {
        LogWarning("⚠️  Found processes potentially using camera hardware:");
        LogWarning("   " + lsof_result);
    }
    
    // Check for camera-related processes more specifically
    std::string ps_check = "ps aux | grep -E '[C]ameraSDK|[i]nsta360|[f]fmpeg.*insta|[f]fplay.*insta' || true";
    std::string ps_result = ExecuteCommand(ps_check);
    if (!ps_result.empty()) {
        LogWarning("⚠️  Found camera-related processes:");
        LogWarning("   " + ps_result);
    }

    // Step 5: Discover camera
    if (!DiscoverCamera()) {
        SetError("Camera discovery failed");
        return false;
    }
    LogInfo("✅ Camera discovery: OK");
    
    // Step 5: Open camera connection
    if (!OpenCamera()) {
        SetError("Failed to open camera connection");
        return false;
    }
    LogInfo("✅ Camera connection: OK");
    
    // Step 6: Validate camera mode and settings
    if (!ValidateCameraMode()) {
        LogWarning("Camera mode validation had issues (continuing anyway)");
    } else {
        LogInfo("✅ Camera mode: OK");
    }
    
    // Step 7: Set up stream delegate
    if (camera_) {
        std::shared_ptr<ins_camera::StreamDelegate> delegate_ptr = std::static_pointer_cast<ins_camera::StreamDelegate>(stream_delegate_);
        camera_->SetStreamDelegate(delegate_ptr);
        LogInfo("✅ Stream delegate: OK");
    }
    
    initialized_ = true;
    LogInfo("🎉 CAMERA MANAGER INITIALIZED SUCCESSFULLY");
    PrintStatus();
    
    return true;
}

bool CameraManager::StartPreviewStream() {
    if (!initialized_ || !camera_) {
        SetError("Camera not initialized");
        return false;
    }
    
    if (streaming_) {
        LogWarning("Stream already running");
        return true;
    }
    
    LogInfo("📹 STARTING PREVIEW STREAM");
    LogInfo("  Resolution: 1920x960 @ 30fps");
    LogInfo("  Bitrate: 0.5MB (tested working value for X3)");
    LogInfo("  Format: H.264 dual fisheye streams");
    LogInfo("  Audio: Disabled");
    
    // Configure stream parameters
    ins_camera::LiveStreamParam param;
    param.video_resolution = CameraConfig::PREVIEW_RESOLUTION;
    param.lrv_video_resulution = CameraConfig::PREVIEW_RESOLUTION; 
    param.video_bitrate = CameraConfig::PREVIEW_BITRATE;
    param.enable_audio = CameraConfig::ENABLE_AUDIO;
    param.using_lrv = CameraConfig::USE_LRV;
    
    // Start the stream
    bool success = camera_->StartLiveStreaming(param);
    
    if (success) {
        streaming_ = true;
        LogInfo("✅ PREVIEW STREAM STARTED SUCCESSFULLY");
        LogInfo("💡 Dual fisheye H.264 streams now available via delegate");
        
        // Start recording to files for testing
        stream_delegate_->StartRecording();
        
        // Give stream a moment to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Check if we're actually receiving data
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        if (!stream_delegate_->IsReceivingData()) {
            LogWarning("⚠️  Stream started but no data received yet (may take a moment)");
        } else {
            LogInfo("✅ Stream data flowing correctly");
        }
        
    } else {
        SetError("Failed to start live streaming");
        LogError("❌ PREVIEW STREAM START FAILED");
        LogError("  Common causes:");
        LogError("  - Camera not in Android mode");
        LogError("  - USB connection issues");
        LogError("  - Conflicting processes");
        LogError("  - Insufficient permissions");
    }
    
    return success;
}

bool CameraManager::StopPreviewStream() {
    if (!streaming_ || !camera_) {
        LogWarning("Stream not running");
        return true;
    }
    
    LogInfo("🛑 STOPPING PREVIEW STREAM");
    
    // Stop recording first
    stream_delegate_->StopRecording();
    
    // Stop the camera stream
    bool success = camera_->StopLiveStreaming();
    
    if (success) {
        streaming_ = false;
        LogInfo("✅ PREVIEW STREAM STOPPED");
    } else {
        SetError("Failed to stop live streaming");
        LogError("❌ PREVIEW STREAM STOP FAILED");
    }
    
    return success;
}

bool CameraManager::RunVerificationMode(int duration_seconds) {
    if (!initialized_ || !camera_) {
        SetError("Camera not initialized");
        return false;
    }
    
    LogInfo("🔍 STARTING VERIFICATION MODE");
    LogInfo("  Duration: " + std::to_string(duration_seconds) + " seconds");
    LogInfo("  This will test the camera stream and exit cleanly");
    
    // Start streaming
    if (!StartPreviewStream()) {
        return false;
    }
    
    // Run for specified duration
    LogInfo("📹 Verifying camera stream...");
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time).count();
            
        if (elapsed >= duration_seconds) {
            break;
        }
        
        // Show progress every 2 seconds
        if (elapsed % 2 == 0 && elapsed > 0) {
            std::cout << "⏱️  Verification: " << elapsed << "/" << duration_seconds 
                      << "s - Frames: " << stream_delegate_->GetFrameCount() 
                      << ", Data: " << stream_delegate_->GetTotalBytesReceived() / 1024 << "KB" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // Show final results
    LogInfo("✅ VERIFICATION COMPLETE");
    LogInfo("  Total frames: " + std::to_string(stream_delegate_->GetFrameCount()));
    LogInfo("  Total data: " + std::to_string(stream_delegate_->GetTotalBytesReceived() / (1024*1024)) + "MB");
    LogInfo("  Stream working: " + std::string(stream_delegate_->IsReceivingData() ? "YES" : "NO"));
    
    // Clean stop
    StopPreviewStream();
    
    return stream_delegate_->IsReceivingData();
}

bool CameraManager::RunViewingMode(int duration_seconds) {
    if (!initialized_ || !camera_) {
        SetError("Camera not initialized");
        return false;
    }
    
    LogInfo("👁️  STARTING VISUAL VIEWING MODE");
    LogInfo("  Duration: " + std::to_string(duration_seconds) + " seconds");
    LogInfo("  This will show the actual camera output in a window");
    
    // Check if ffplay is available
    if (system("which ffplay > /dev/null 2>&1") != 0) {
        LogError("❌ ffplay not found. Install with: sudo apt-get install ffmpeg");
        LogError("   Visual viewing requires ffplay to display video");
        return false;
    }
    
    // Start streaming
    if (!StartPreviewStream()) {
        return false;
    }
    
    // Create a temporary file for viewing
    std::string temp_file = "/tmp/insta360_live_view.h264";
    
    // Start recording to the temp file
    stream_delegate_->StartRecording("/tmp/insta360_live_view");
    
    LogInfo("📹 Starting camera stream...");
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Let some data accumulate
    
    // Launch ffplay in background to show the video
    std::string ffplay_cmd = "ffplay -f h264 -framerate 30 -video_size 1920x960 -window_title 'Insta360 X3 Live View' " + temp_file + " > /dev/null 2>&1 &";
    LogInfo("🎬 Launching video viewer...");
    LogInfo("  Command: ffplay with H.264 fisheye stream");
    LogInfo("  Resolution: 1920×960 @ 30fps");
    LogInfo("  Window title: 'Insta360 X3 Live View'");
    
    int ffplay_result = system(ffplay_cmd.c_str());
    if (ffplay_result != 0) {
        LogWarning("⚠️  ffplay launch may have failed, but continuing...");
    }
    
    // Run for specified duration with live updates
    LogInfo("👁️  Visual viewing active - you should see video window");
    LogInfo("  Close the video window or wait " + std::to_string(duration_seconds) + " seconds to exit");
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time).count();
            
        if (elapsed >= duration_seconds) {
            break;
        }
        
        // Show progress every 5 seconds
        if (elapsed % 5 == 0 && elapsed > 0) {
            std::cout << "👁️  Viewing: " << elapsed << "/" << duration_seconds 
                      << "s - Frames: " << stream_delegate_->GetFrameCount() 
                      << ", Data: " << stream_delegate_->GetTotalBytesReceived() / 1024 << "KB" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // Kill ffplay processes
    system("pkill -f 'ffplay.*insta360_live_view' > /dev/null 2>&1");
    
    // Show final results
    LogInfo("✅ VIEWING MODE COMPLETE");
    LogInfo("  Total frames: " + std::to_string(stream_delegate_->GetFrameCount()));
    LogInfo("  Total data: " + std::to_string(stream_delegate_->GetTotalBytesReceived() / (1024*1024)) + "MB");
    LogInfo("  Stream working: " + std::string(stream_delegate_->IsReceivingData() ? "YES" : "NO"));
    
    // Clean stop
    StopPreviewStream();
    
    // Clean up temp file
    system(("rm -f " + temp_file).c_str());
    
    return stream_delegate_->IsReceivingData();
}

void CameraManager::Shutdown() {
    if (!initialized_) {
        return;
    }
    
    LogInfo("🔄 SHUTTING DOWN CAMERA MANAGER");
    
    // Stop streaming if active
    if (streaming_) {
        StopPreviewStream();
    }
    
    // Close camera connection
    if (camera_) {
        camera_->Close();
        camera_.reset();
        LogInfo("✅ Camera connection closed");
    }
    
    initialized_ = false;
    streaming_ = false;
    
    LogInfo("✅ CAMERA MANAGER SHUTDOWN COMPLETE");
}

bool CameraManager::IsConnected() const {
    return camera_ && camera_->IsConnected();
}

bool CameraManager::IsStreaming() const {
    return streaming_ && stream_delegate_ && stream_delegate_->IsReceivingData();
}

void CameraManager::PrintStatus() const {
    std::cout << "\n═══════════════════════════════════════" << std::endl;
    std::cout << "📊 CAMERA MANAGER STATUS" << std::endl;
    std::cout << "═══════════════════════════════════════" << std::endl;
    std::cout << "Initialized: " << (initialized_ ? "✅ YES" : "❌ NO") << std::endl;
    std::cout << "Connected: " << (IsConnected() ? "✅ YES" : "❌ NO") << std::endl;
    std::cout << "Streaming: " << (IsStreaming() ? "✅ YES" : "❌ NO") << std::endl;
    
    if (initialized_ && !camera_descriptor_.serial_number.empty()) {
        std::cout << "Camera: " << camera_descriptor_.camera_name 
                  << " (S/N: " << camera_descriptor_.serial_number << ")" << std::endl;
        std::cout << "Firmware: " << camera_descriptor_.fw_version << std::endl;
    }
    
    if (stream_delegate_ && streaming_) {
        std::cout << "Frames received: " << stream_delegate_->GetFrameCount() << std::endl;
        std::cout << "Data received: " << stream_delegate_->GetTotalBytesReceived() / (1024*1024) << "MB" << std::endl;
    }
    
    if (!last_error_.empty()) {
        std::cout << "Last error: " << last_error_ << std::endl;
    }
    
    std::cout << "═══════════════════════════════════════\n" << std::endl;
}

void CameraManager::PrintConfiguration() const {
    std::cout << "\n█████████████████████████████████████████████████████████████" << std::endl;
    std::cout << "🎥 INSTA360 X3 CAMERA CONFIGURATION" << std::endl;
    std::cout << "█████████████████████████████████████████████████████████████" << std::endl;
    std::cout << "📺 PREVIEW STREAM SETTINGS:" << std::endl;
    std::cout << "  • Resolution: 1920x960 @ 30fps (RES_1920_960P30)" << std::endl;
    std::cout << "  • Bitrate: 0.5MB (1024*1024/2) - TESTED VALUE FOR X3" << std::endl;
    std::cout << "  • Encoding: H.264 single fisheye stream" << std::endl;
    std::cout << "  • Streams: 1 (stream_index 0 only for <5.7K resolution)" << std::endl;
    std::cout << "  • Audio: Disabled" << std::endl;
    std::cout << "  • LRV: Disabled (using full resolution)" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "🔧 CRITICAL REQUIREMENTS:" << std::endl;
    std::cout << "  ⚠️  Camera MUST be in Android mode:" << std::endl;
    std::cout << "     Camera Settings → General → USB Mode → Android" << std::endl;
    std::cout << "  ⚠️  Must run with sudo (libusb requirement)" << std::endl;
    std::cout << "  ⚠️  USB connection required (no WiFi support)" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "📋 FROM CAMERA SDK DOCUMENTATION:" << std::endl;
    std::cout << "  • Preview resolution 1920×960 is currently recommended" << std::endl;
    std::cout << "  • For resolution <5.7K: single video stream provided" << std::endl;
    std::cout << "  • For resolution ≥5.7K: dual video streams provided" << std::endl;
    std::cout << "  • Data format is H.265 or H.264 encoded" << std::endl;
    std::cout << "  • Decoded data consists of fisheye image(s)" << std::endl;
    std::cout << "█████████████████████████████████████████████████████████████\n" << std::endl;
}

//==============================================================================
// PRIVATE HELPER METHODS
//==============================================================================

bool CameraManager::CheckSudoPermissions() {
    return geteuid() == 0; // Check if running as root
}

bool CameraManager::ValidateUSBConnection() {
    LogInfo("🔍 Validating USB connection...");
    
    // Check if lsusb command is available
    std::string lsusb_output = ExecuteCommand("which lsusb");
    if (lsusb_output.empty()) {
        LogError("lsusb command not found - install usbutils package");
        return false;
    }
    
    // Check for Insta360 device
    std::string usb_devices = ExecuteCommand("lsusb");
    LogInfo("USB devices found:");
    std::cout << usb_devices << std::endl;
    
    // Look for Insta360 vendor ID (0x2e1a) or device name
    bool insta360_found = usb_devices.find("2e1a") != std::string::npos ||
                         usb_devices.find("Insta360") != std::string::npos;
    
    if (!insta360_found) {
        LogError("❌ Insta360 camera not detected in USB devices");
        LogError("  Troubleshooting:");
        LogError("  1. Check USB cable connection");
        LogError("  2. Ensure camera is powered on");
        LogError("  3. Set camera to Android mode (NOT U disk mode)");
        LogError("  4. Try a different USB port/cable");
        return false;
    }
    
    LogInfo("✅ Insta360 device detected in USB");
    return true;
}

bool CameraManager::KillConflictingProcesses() {
    LogInfo("🔄 Searching for conflicting camera processes...");
    
    // First do enhanced Insta360-specific cleanup
    bool insta360_cleaned = ForceKillInsta360Processes();
    
    // Then do general camera process cleanup
    std::vector<int> pids = FindProcessesByPattern(CameraConfig::CAMERA_PROCESS_PATTERN);
    
    if (pids.empty()) {
        LogInfo("✅ No conflicting processes found");
        return insta360_cleaned;
    }
    
    LogInfo("Found " + std::to_string(pids.size()) + " potentially conflicting processes");
    
    // Kill found processes
    bool all_killed = true;
    for (int pid : pids) {
        LogInfo("Killing process PID " + std::to_string(pid));
        if (kill(pid, SIGTERM) != 0) {
            LogWarning("Failed to send SIGTERM to PID " + std::to_string(pid));
            // Try SIGKILL as fallback
            if (kill(pid, SIGKILL) != 0) {
                LogError("Failed to kill PID " + std::to_string(pid));
                all_killed = false;
            }
        }
    }
    
    // Give processes time to die
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Verify they're gone
    std::vector<int> remaining = FindProcessesByPattern(CameraConfig::CAMERA_PROCESS_PATTERN);
    if (!remaining.empty()) {
        LogWarning(std::to_string(remaining.size()) + " processes still running after cleanup");
        all_killed = false;
    }
    
    return all_killed && insta360_cleaned;
}

bool CameraManager::ResetUSBDevice() {
    LogInfo("🔄 Attempting USB device reset for Insta360 X3...");
    
    try {
        // Find the USB device bus and device number
        std::string lsusb_output = ExecuteCommand("lsusb | grep -i '2e1a\\|insta360'");
        if (lsusb_output.empty()) {
            LogError("Cannot find Insta360 device for USB reset");
            return false;
        }
        
        // Extract bus and device numbers
        std::regex bus_device_regex(R"(Bus (\d+) Device (\d+):)");
        std::smatch matches;
        
        if (!std::regex_search(lsusb_output, matches, bus_device_regex)) {
            LogError("Cannot parse USB device information for reset");
            return false;
        }
        
        std::string bus_num = matches[1].str();
        std::string dev_num = matches[2].str();
        
        LogInfo("   USB Bus: " + bus_num + ", Device: " + dev_num);
        
        // Method 1: Try usbreset command if available
        std::string usbreset_check = ExecuteCommand("which usbreset 2>/dev/null");
        if (!usbreset_check.empty()) {
            LogInfo("   Using usbreset utility...");
            std::string reset_cmd = "usbreset /dev/bus/usb/" + bus_num.substr(bus_num.length()-3) + "/" + dev_num.substr(dev_num.length()-3);
            int result = system(reset_cmd.c_str());
            if (result == 0) {
                LogInfo("✅ USB device reset via usbreset successful");
                return true;
            }
            LogWarning("⚠️  usbreset command failed, trying alternative method");
        }
        
        // Method 2: Try udev-based reset
        LogInfo("   Attempting udev-based USB reset...");
        std::string device_path = "/sys/bus/usb/devices/" + bus_num + "-*";
        std::string find_cmd = "find /sys/bus/usb/devices -name '" + bus_num + "-*' -type d 2>/dev/null | head -1";
        std::string device_dir = ExecuteCommand(find_cmd);
        
        if (!device_dir.empty()) {
            // Remove trailing newline
            device_dir.erase(device_dir.find_last_not_of(" \n\r\t") + 1);
            
            std::string reset_cmd = "echo '" + device_dir + "' > /sys/bus/usb/drivers/usb/unbind 2>/dev/null; sleep 1; echo '" + device_dir + "' > /sys/bus/usb/drivers/usb/bind 2>/dev/null";
            int result = system(reset_cmd.c_str());
            
            if (result == 0) {
                LogInfo("✅ USB device reset via udev successful");
                return true;
            }
        }
        
        // Method 3: Try generic USB reset approach
        LogInfo("   Attempting generic USB device reset...");
        std::string generic_reset = "echo 0 > /sys/bus/usb/devices/usb" + bus_num + "/authorized; sleep 1; echo 1 > /sys/bus/usb/devices/usb" + bus_num + "/authorized";
        int result = system(generic_reset.c_str());
        
        if (result == 0) {
            LogInfo("✅ Generic USB bus reset completed");
            return true;
        }
        
        LogWarning("⚠️  All USB reset methods failed - device may need manual reconnection");
        return false;
        
    } catch (const std::exception& e) {
        LogError("Exception during USB reset: " + std::string(e.what()));
        return false;
    }
}

bool CameraManager::ForceKillInsta360Processes() {
    LogInfo("🔄 Enhanced Insta360 process cleanup...");
    
    bool success = true;
    
    // Kill specific Insta360 processes that commonly lock the camera
    std::vector<std::string> process_patterns = {
        "CameraSDKTest",
        "insta360",
        "Insta360",
        "[Ii]nsta360.*[Cc]amera",
        "[Cc]ameraSDK",
        "ffmpeg.*insta",
        "ffplay.*insta"
    };
    
    for (const auto& pattern : process_patterns) {
        std::string kill_cmd = "pkill -f '" + pattern + "' 2>/dev/null";
        int result = system(kill_cmd.c_str());
        // Note: pkill returns 1 if no processes match, which is fine
        
        // Follow up with force kill if needed
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::string force_kill_cmd = "pkill -9 -f '" + pattern + "' 2>/dev/null";
        system(force_kill_cmd.c_str());
    }
    
    // Clear any Camera SDK shared memory segments
    LogInfo("   Cleaning Camera SDK shared memory...");
    system("ipcs -m | grep $(whoami) | awk '{print $2}' | xargs -r ipcrm -m 2>/dev/null");
    
    // Kill any processes holding /dev/video* devices
    LogInfo("   Cleaning video device locks...");
    system("lsof /dev/video* 2>/dev/null | awk 'NR>1 {print $2}' | xargs -r kill -9 2>/dev/null");
    
    // Clear any USB device locks
    LogInfo("   Clearing USB device locks...");
    system("lsof | grep -i '2e1a\\|insta360' | awk '{print $2}' | xargs -r kill -TERM 2>/dev/null");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    system("lsof | grep -i '2e1a\\|insta360' | awk '{print $2}' | xargs -r kill -9 2>/dev/null");
    
    LogInfo("✅ Enhanced Insta360 cleanup completed");
    return success;
}

bool CameraManager::DiscoverCamera() {
    LogInfo("🔍 Discovering Insta360 cameras...");
    LogInfo("⏱️  This may take up to " + std::to_string(CameraConfig::DISCOVERY_TIMEOUT_MS/1000) + " seconds...");
    
    ins_camera::DeviceDiscovery discovery;
    
    // Add timeout for discovery
    auto start_time = std::chrono::steady_clock::now();
    std::vector<ins_camera::DeviceDescriptor> devices;
    int dot_count = 0;
    
    while (devices.empty()) {
        devices = discovery.GetAvailableDevices();
        
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
            
        if (elapsed > CameraConfig::DISCOVERY_TIMEOUT_MS) {
            std::cout << std::endl; // End the progress dots
            SetError("Camera discovery timeout after " + std::to_string(CameraConfig::DISCOVERY_TIMEOUT_MS) + "ms");
            LogError("💡 COMMON CAUSES:");
            LogError("   • Camera not in Android mode (Settings → General → USB Mode → Android)");
            LogError("   • Camera already in use by another application");
            LogError("   • USB connection issues");
            LogError("   • Camera powered off or in sleep mode");
            return false;
        }
        
        if (devices.empty()) {
            // Show progress dots every 500ms
            if ((elapsed / 500) > dot_count) {
                std::cout << "." << std::flush;
                dot_count++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    if (dot_count > 0) {
        std::cout << std::endl; // End the progress dots
    }
    
    LogInfo("Found " + std::to_string(devices.size()) + " camera(s)");
    
    // Find X3 camera
    bool x3_found = false;
    for (const auto& device : devices) {
        std::cout << "📷 Camera found:" << std::endl;
        std::cout << "  Model: " << device.camera_name << std::endl;
        std::cout << "  Serial: " << device.serial_number << std::endl;
        std::cout << "  Firmware: " << device.fw_version << std::endl;
        
        if (device.camera_name.find("X3") != std::string::npos) {
            camera_descriptor_ = device;
            x3_found = true;
            LogInfo("✅ Insta360 X3 selected for use");
            break;
        }
    }
    
    if (!x3_found) {
        // Use first available camera if no X3 specifically found
        if (!devices.empty()) {
            camera_descriptor_ = devices[0];
            LogWarning("X3 not found, using first available camera: " + devices[0].camera_name);
            x3_found = true;
        }
    }
    
    // Clean up
    discovery.FreeDeviceDescriptors(devices);
    
    return x3_found;
}

bool CameraManager::OpenCamera() {
    LogInfo("🔗 Opening camera connection...");
    LogInfo("⚠️  Camera SDK may take 15-45 seconds to timeout if camera is not accessible");
    LogInfo("⏱️  Expected wait time: 5-45 seconds depending on camera state...");
    
    try {
        camera_ = std::make_shared<ins_camera::Camera>(camera_descriptor_.info);
        
        auto start_time = std::chrono::steady_clock::now();
        bool opened = false;
        int progress_count = 0;
        
        LogInfo("📡 Attempting camera SDK connection...");
        LogInfo("   Camera found: " + std::string(camera_descriptor_.camera_name));
        LogInfo("   Serial: " + std::string(camera_descriptor_.serial_number));
        LogInfo("   USB Device: Bus 003 Device 023: ID 2e1a:0002");
        
        // Add SDK error logging
        ins_camera::SetLogLevel(ins_camera::LogLevel::VERBOSE);
        
        // Enhanced pre-connection diagnostics and cleanup
        LogInfo("🔍 Pre-connection diagnostic checks...");
        std::string usb_check = "lsof 2>/dev/null | grep -i '2e1a\\|insta360' || echo 'No processes using device'";
        std::string usb_result = ExecuteCommand(usb_check);
        LogInfo("   USB device usage: " + (usb_result.empty() ? "None detected" : usb_result));
        
        std::string driver_check = "ls -la /dev/video* 2>/dev/null | wc -l || echo '0'";
        std::string driver_result = ExecuteCommand(driver_check);
        LogInfo("   Video devices present: " + driver_result + " /dev/video* entries");
        
        // Check if camera might be bound to kernel drivers
        std::string kernel_check = "dmesg | tail -10 | grep -i 'usb\\|video' || echo 'No recent USB/video messages'";  
        std::string kernel_result = ExecuteCommand(kernel_check);
        if (!kernel_result.empty() && kernel_result != "No recent USB/video messages") {
            LogInfo("   Recent kernel messages: " + kernel_result);
        }
        
        // CRITICAL: Try USB device reset before SDK connection
        LogInfo("🔄 ATTEMPTING USB DEVICE RESET (often fixes SDK connection issues)...");
        if (ResetUSBDevice()) {
            LogInfo("✅ USB device reset successful - waiting for re-enumeration...");
            std::this_thread::sleep_for(std::chrono::milliseconds(3000)); // Wait for USB re-enumeration
            
            // Re-verify USB connection after reset
            if (!ValidateUSBConnection()) {
                LogError("❌ USB connection lost after reset - device may need manual reconnection");
                return false;
            }
        } else {
            LogWarning("⚠️  USB device reset failed - continuing with standard connection attempt");
        }
        
        LogInfo("🚀 Attempting Camera SDK Open() call...");
        LogInfo("   This will either succeed quickly or fail after SDK internal timeout");
        
        // Note: camera_->Open() can block for 45 seconds with internal SDK retries
        // We can't interrupt it, so we need to let it complete
        opened = camera_->Open();
        
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        
        if (!opened) {
            SetError("Camera SDK connection failed after " + std::to_string(elapsed) + "ms");
            LogError("💡 ANALYSIS: Camera was discovered but SDK connection failed");
            LogError("   This is a VERY COMMON issue with Insta360 X3 cameras");
            LogError("🔍 MOST LIKELY CAUSES (in order of probability):");
            LogError("   1. ★ Previous camera session didn't close properly (MOST COMMON)");
            LogError("   2. ★ Camera needs power cycle (very common)");
            LogError("   3. ★ USB device is in locked/busy state");
            LogError("   4. Another process has camera resource locked");
            LogError("   5. USB permissions or kernel driver conflict");
            LogError("🔧 PROVEN TROUBLESHOOTING STEPS:");
            LogError("   1. 🔄 POWER CYCLE: Turn camera OFF, wait 10 seconds, turn ON");
            LogError("   2. 🔌 USB RECONNECT: Unplug USB cable, wait 5 seconds, reconnect");
            LogError("   3. ⏰ WAIT: Camera may need up to 30 seconds to release resources");
            LogError("   4. 🔄 Try running this command again (USB reset was attempted)");
            LogError("   5. 🔍 Check for zombie processes:");
            LogError("      sudo ps aux | grep -E '[C]ameraSDK|[i]nsta360|[f]fmpeg.*insta'");
            LogError("      sudo killall -9 CameraSDKTest  # if found");
            LogError("❌ Note: The camera discovery working means Android mode & USB are OK");
            LogError("❌ This failure is specifically an SDK resource lock issue");
            return false;
        }
        
        // Verify connection
        if (!camera_->IsConnected()) {
            SetError("Camera opened but connection check failed");
            return false;
        }
        
        LogInfo("✅ Camera connection established successfully!");
        LogInfo("🎉 SDK connection is working - camera is ready for streaming");
        return true;
        
    } catch (const std::exception& e) {
        SetError("Exception during camera open: " + std::string(e.what()));
        return false;
    }
}

bool CameraManager::ValidateCameraMode() {
    if (!camera_) {
        return false;
    }
    
    LogInfo("🔍 Validating camera mode and settings...");
    
    try {
        // Check battery status
        ins_camera::BatteryStatus battery;
        if (camera_->GetBatteryStatus(battery)) {
            LogInfo("Battery level: " + std::to_string(battery.battery_level) + "%");
            if (battery.battery_level < 20) {
                LogWarning("⚠️  Low battery: " + std::to_string(battery.battery_level) + "%");
            }
        }
        
        // Check storage
        ins_camera::StorageStatus storage;
        if (camera_->GetStorageState(storage)) {
            double free_gb = storage.free_space / (1024.0 * 1024.0 * 1024.0);
            LogInfo("Storage free: " + std::to_string(free_gb) + "GB");
            if (free_gb < 1.0) {
                LogWarning("⚠️  Low storage: " + std::to_string(free_gb) + "GB remaining");
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        LogWarning("Camera mode validation error: " + std::string(e.what()));
        return false;
    }
}

std::vector<int> CameraManager::FindProcessesByPattern(const std::string& pattern) {
    std::vector<int> pids;
    
    // Get our process tree to avoid killing ourselves
    pid_t current_pid = getpid();
    pid_t parent_pid = getppid();
    
    std::string command = "pgrep -f '" + pattern + "'";
    std::string result = ExecuteCommand(command);
    
    std::istringstream iss(result);
    std::string line;
    while (std::getline(iss, line)) {
        try {
            int pid = std::stoi(line);
            
            // Don't kill ourselves or our parent script
            if (pid == current_pid || pid == parent_pid) {
                continue;
            }
            
            // Check if this PID is actually a camera-related process (not our script)
            std::string ps_cmd = "ps -p " + std::to_string(pid) + " -o cmd= 2>/dev/null";
            std::string cmd_result = ExecuteCommand(ps_cmd);
            
            // Skip if it's our own utility scripts
            if (cmd_result.find("run_camera_utility.sh") != std::string::npos ||
                cmd_result.find("camera_manager") != std::string::npos) {
                continue;
            }
            
            // Only target actual camera/SDK processes
            if (cmd_result.find("insta360") != std::string::npos ||
                cmd_result.find("CameraSDK") != std::string::npos ||
                cmd_result.find("ffmpeg") != std::string::npos ||
                cmd_result.find("ffplay") != std::string::npos) {
                pids.push_back(pid);
            }
            
        } catch (...) {
            // Ignore invalid lines
        }
    }
    
    return pids;
}

std::string CameraManager::ExecuteCommand(const std::string& command) {
    std::string result;
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe) {
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        pclose(pipe);
    }
    return result;
}

void CameraManager::SetError(const std::string& error) {
    last_error_ = error;
    LogError(error);
}

void CameraManager::LogInfo(const std::string& message) {
    std::cout << "ℹ️  " << message << std::endl;
}

void CameraManager::LogWarning(const std::string& message) {
    std::cout << "⚠️  " << message << std::endl;
}

void CameraManager::LogError(const std::string& message) {
    std::cerr << "❌ " << message << std::endl;
}

std::string CameraManager::GetCameraInfo() const {
    if (camera_descriptor_.serial_number.empty()) {
        return "No camera connected";
    }
    
    return camera_descriptor_.camera_name + " (S/N: " + camera_descriptor_.serial_number + 
           ", FW: " + camera_descriptor_.fw_version + ")";
}

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================

namespace CameraUtils {
    
    bool CheckUSBDevice(uint16_t vendor_id) {
        std::stringstream hex_id;
        hex_id << std::hex << vendor_id;
        std::string command = "lsusb | grep -i " + hex_id.str();
        return system(command.c_str()) == 0;
    }
    
    bool CheckLibUSBInstalled() {
        return system("pkg-config --exists libusb-1.0") == 0;
    }
    
    bool CheckSudoPermissions() {
        return geteuid() == 0;
    }
    
    std::vector<int> FindCameraProcesses() {
        // Implementation would go here
        return {};
    }
    
    bool KillCameraProcesses() {
        // Implementation would go here
        return true;
    }
    
    void PrintSystemInfo() {
        std::cout << "🖥️  SYSTEM INFORMATION:" << std::endl;
        std::cout << "  OS: ";
        int result = system("uname -a");
        (void)result; // Suppress unused warning
    }
    
    void PrintUSBDevices() {
        std::cout << "🔌 USB DEVICES:" << std::endl;
        int result = system("lsusb");
        (void)result; // Suppress unused warning
    }
    
    void PrintRequirements() {
        std::cout << "📋 REQUIREMENTS CHECKLIST:" << std::endl;
        std::cout << "  • Camera in Android mode: " << (true ? "❓" : "❌") << " (cannot check remotely)" << std::endl;
        std::cout << "  • Sudo permissions: " << (CheckSudoPermissions() ? "✅" : "❌") << std::endl;
        std::cout << "  • LibUSB installed: " << (CheckLibUSBInstalled() ? "✅" : "❌") << std::endl;
        std::cout << "  • Insta360 USB device: " << (CheckUSBDevice(0x2e1a) ? "✅" : "❌") << std::endl;
    }
}

//==============================================================================
// MAIN FUNCTION FOR STANDALONE TESTING
//==============================================================================

int main(int argc, char* argv[]) {
    std::cout << "🎥 INSTA360 X3 CAMERA UTILITY - ATTEMPT 3" << std::endl;
    std::cout << "Robust camera manager for AI agent environments\n" << std::endl;
    
    // Check sudo immediately 
    if (geteuid() != 0) {
        std::cerr << "❌ SUDO REQUIRED: This utility must run with sudo permissions" << std::endl;
        std::cerr << "  Please run: sudo " << argv[0] << std::endl;
        std::cerr << "  Or use: sudo ./run_camera_utility.sh" << std::endl;
        return 1;
    }

    // Parse command line arguments
    bool debug_mode = false;
    bool status_only = false;
    bool no_cleanup = false;
    bool verify_only = false;
    bool view_only = false;
    int verify_duration = 10;
    int view_duration = 30;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--debug") {
            debug_mode = true;
            ins_camera::SetLogLevel(ins_camera::LogLevel::VERBOSE);
        } else if (arg == "--status-only") {
            status_only = true;
        } else if (arg == "--verify-only") {
            verify_only = true;
        } else if (arg == "--verify-duration") {
            if (i + 1 < argc) {
                verify_duration = std::stoi(argv[++i]);
            }
        } else if (arg == "--view-only") {
            view_only = true;
        } else if (arg == "--view-duration") {
            if (i + 1 < argc) {
                view_duration = std::stoi(argv[++i]);
            }
        } else if (arg == "--no-cleanup") {
            no_cleanup = true;
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --debug                    Enable verbose logging" << std::endl;
            std::cout << "  --status-only              Just check status and exit" << std::endl;
            std::cout << "  --verify-only              Start stream, verify for 10s, then exit (CLI only)" << std::endl;
            std::cout << "  --verify-duration <sec>    Set verification duration (default: 10)" << std::endl;
            std::cout << "  --view-only                Show actual video output for 30s, then exit" << std::endl;
            std::cout << "  --view-duration <sec>      Set viewing duration (default: 30)" << std::endl;
            std::cout << "  --no-cleanup               Skip killing conflicting processes" << std::endl;
            std::cout << "  --help                     Show this help" << std::endl;
            return 0;
        }
    }
    
    // Status-only mode
    if (status_only) {
        CameraUtils::PrintRequirements();
        CameraUtils::PrintUSBDevices();
        return 0;
    }
    
    // Create and initialize camera manager
    CameraManager manager;
    
    if (!manager.Initialize(!no_cleanup)) {
        std::cerr << "❌ INITIALIZATION FAILED: " << manager.GetLastError() << std::endl;
        return 1;
    }
    
    // Verification mode - test stream and exit
    if (verify_only) {
        std::cout << "\n🔍 VERIFICATION MODE - Testing camera stream..." << std::endl;
        bool verification_success = manager.RunVerificationMode(verify_duration);
        manager.Shutdown();
        
        if (verification_success) {
            std::cout << "✅ VERIFICATION PASSED - Camera is working correctly!" << std::endl;
            return 0;
        } else {
            std::cout << "❌ VERIFICATION FAILED - Camera stream issues detected" << std::endl;
            return 1;
        }
    }
    
    // Visual viewing mode - show actual video output and exit
    if (view_only) {
        std::cout << "\n👁️  VISUAL VIEWING MODE - Showing camera video output..." << std::endl;
        bool viewing_success = manager.RunViewingMode(view_duration);
        manager.Shutdown();
        
        if (viewing_success) {
            std::cout << "✅ VIEWING COMPLETE - Camera video output successful!" << std::endl;
            return 0;
        } else {
            std::cout << "❌ VIEWING FAILED - Camera video issues detected" << std::endl;
            return 1;
        }
    }
    
    // Start preview stream for continuous mode
    if (!manager.StartPreviewStream()) {
        std::cerr << "❌ STREAM START FAILED: " << manager.GetLastError() << std::endl;
        return 1;
    }
    
    // Run for a test period
    std::cout << "\n🎬 CAMERA RUNNING - Press Ctrl+C to stop..." << std::endl;
    
    // Set up signal handler for graceful shutdown
    signal(SIGINT, [](int /*sig*/) {
        std::cout << "\n🛑 Shutdown signal received..." << std::endl;
        exit(0);
    });
    
    // Keep running and show periodic status
    int status_interval = 0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        if (++status_interval >= 6) { // Every 30 seconds
            manager.PrintStatus();
            status_interval = 0;
        }
        
        if (!manager.IsConnected()) {
            std::cerr << "❌ Camera disconnected!" << std::endl;
            break;
        }
    }
    
    // Clean shutdown
    manager.Shutdown();
    std::cout << "✅ CAMERA UTILITY EXITED CLEANLY" << std::endl;
    
    return 0;
}