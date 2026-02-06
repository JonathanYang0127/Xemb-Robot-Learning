#!/bin/bash

echo "🎬 Setting up Video Teleoperation System for PiperX"
echo "=================================================="

# Check if we're on macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "✅ Detected macOS"
else
    echo "⚠️  This script is optimized for macOS"
fi

echo ""
echo "📋 Prerequisites Check:"
echo "======================="

# Check if OBS is installed
if command -v obs &> /dev/null || [ -d "/Applications/OBS.app" ]; then
    echo "✅ OBS Studio found"
else
    echo "❌ OBS Studio not found"
    echo "📥 Please install OBS Studio from: https://obsproject.com/"
    echo "   After installation, run this script again"
    exit 1
fi

# Check if ngrok is installed
if command -v ngrok &> /dev/null; then
    echo "✅ ngrok found"
else
    echo "❌ ngrok not found"
    echo "📥 Please install ngrok from: https://ngrok.com/"
    echo "   After installation, run this script again"
    exit 1
fi

# Check if Python dependencies are installed
echo ""
echo "🐍 Checking Python dependencies..."

python3 -c "import cv2" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ OpenCV (cv2) found"
else
    echo "❌ OpenCV not found"
    echo "📦 Installing OpenCV..."
    pip3 install opencv-python
fi

python3 -c "import vuer" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Vuer found"
else
    echo "❌ Vuer not found"
    echo "📦 Installing Vuer..."
    pip3 install vuer
fi

echo ""
echo "🎮 OBS Setup Instructions:"
echo "=========================="
echo "1. Open OBS Studio"
echo "2. Add your Insta360 X3 as a video source:"
echo "   - Click '+' in Sources panel"
echo "   - Select 'Video Capture Device'"
echo "   - Choose your Insta360 X3 camera"
echo "3. Start Virtual Camera:"
echo "   - Go to Tools → Start Virtual Camera"
echo "   - Or use the 'Start Virtual Camera' button"
echo ""
echo "📱 Quest 3 Setup Instructions:"
echo "=============================="
echo "1. Open Meta Quest Browser"
echo "2. Navigate to the ngrok URL (will be shown when you start the relay)"
echo "3. Point your left controller at the screen"
echo "4. Move the controller to see it respond"
echo ""

echo "🚀 Ready to start video teleoperation!"
echo ""
echo "To start the system:"
echo "1. Start OBS Virtual Camera"
echo "2. Run: python3 relay_vr_controller_with_video.py"
echo "3. Start ngrok: ngrok http 8012"
echo "4. Access the ngrok URL from Quest 3 browser"
echo ""

# Check if OBS Virtual Camera is running
echo "🔍 Checking OBS Virtual Camera status..."
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    ret, frame = cap.read()
    cap.release()
    if ret:
        print('✅ OBS Virtual Camera is running and streaming')
    else:
        print('⚠️  OBS Virtual Camera detected but not streaming')
        print('   Please start OBS Virtual Camera')
else:
    print('❌ OBS Virtual Camera not found')
    print('   Please start OBS Virtual Camera')
"

echo ""
echo "🎯 Quick Start Commands:"
echo "========================"
echo "# Terminal 1 - Start ngrok tunnel"
echo "ngrok http 8012"
echo ""
echo "# Terminal 2 - Start VR relay with video"
echo "python3 relay_vr_controller_with_video.py"
echo ""
echo "# Terminal 3 - Start VR relay without video (if video fails)"
echo "python3 relay_vr_controller_with_video.py --no-video"
echo "" 