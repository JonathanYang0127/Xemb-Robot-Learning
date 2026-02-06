#!/usr/bin/env python3
"""
Simple WebRTC server for Insta360 X3
Reads from HTTP stream and serves via WebRTC with minimal latency
"""

import asyncio
import json
import logging
from aiohttp import web
import aiohttp_cors
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer
import av
import requests
import threading
import time
import subprocess
import numpy as np
from fractions import Fraction

logging.basicConfig(level=logging.INFO)

class HTTPStreamTrack(VideoStreamTrack):
    """Read H.264 from HTTP using ffmpeg subprocess and convert to WebRTC frames"""
    
    def __init__(self):
        super().__init__()
        self.process = None
        self.container = None
        self._start_time = None
        self._frame_count = 0
        self._lock = asyncio.Lock()
        
    async def recv(self):
        async with self._lock:
            if self.process is None:
                try:
                    print("🔄 Starting ffmpeg process for HTTP stream...")
                    
                    # Use ffmpeg to convert HTTP stream to raw video
                    self.process = subprocess.Popen([
                        'ffmpeg',
                        '-f', 'h264',
                        '-fflags', '+genpts',
                        '-i', 'http://localhost:8080',
                        '-f', 'rawvideo',
                        '-pix_fmt', 'yuv420p',
                        '-an',  # no audio
                        '-loglevel', 'quiet',
                        'pipe:1'
                    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    
                    # Create av container from subprocess stdout
                    self.container = av.open(self.process.stdout, format='rawvideo', mode='r', options={
                        'pixel_format': 'yuv420p',
                        'video_size': '1440x720',
                        'framerate': '30'
                    })
                    
                    self._start_time = time.time()
                    print("✅ FFmpeg process started successfully")
                    
                except Exception as e:
                    print(f"❌ Failed to start ffmpeg process: {e}")
                    if self.process:
                        self.process.terminate()
                        self.process = None
                    raise
        
        try:
            if self.container:
                for packet in self.container.demux():
                    for frame in packet.decode():
                        self._frame_count += 1
                        
                        # Log first few frames
                        if self._frame_count <= 3:
                            print(f"📹 Received frame {self._frame_count}: {frame.width}x{frame.height}, format={frame.format.name}")
                        elif self._frame_count == 10:
                            print(f"📹 Frame reception working (received {self._frame_count} frames)")
                        
                        # Set proper timing for WebRTC
                        if self._start_time is None:
                            self._start_time = time.time()
                            
                        pts = int((time.time() - self._start_time) * 90000)
                        frame.pts = pts
                        frame.time_base = Fraction(1, 90000)
                        
                        return frame
                        
        except Exception as e:
            print(f"❌ Frame processing error: {e}")
            # Clean up and try to restart
            if self.process:
                self.process.terminate()
                self.process = None
            if self.container:
                self.container.close()
                self.container = None
            raise
            
        return None
    
    def __del__(self):
        if self.process:
            self.process.terminate()
        if self.container:
            self.container.close()

# Store peer connections
pcs = set()

async def offer(request):
    try:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        
        config = RTCConfiguration(
            iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
        )
        pc = RTCPeerConnection(configuration=config)
        pcs.add(pc)
        
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"Connection state: {pc.connectionState}")
            if pc.connectionState == "failed" or pc.connectionState == "closed":
                await pc.close()
                pcs.discard(pc)
        
        @pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            print(f"ICE connection state: {pc.iceConnectionState}")
        
        # Set remote description first
        await pc.setRemoteDescription(offer)
        
        # Add video track after setting remote description
        video_track = HTTPStreamTrack()
        pc.addTrack(video_track)
        
        # Create and set answer
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        
        return web.Response(
            content_type="application/json",
            text=json.dumps({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            })
        )
    except Exception as e:
        print(f"❌ WebRTC offer error: {e}")
        return web.Response(
            status=500,
            content_type="application/json",
            text=json.dumps({"error": str(e)})
        )

async def index(request):
    html = """
<!DOCTYPE html>
<html>
<head>
    <title>Insta360 X3 - Low Latency WebRTC</title>
    <style>
        body { margin: 0; background: #000; color: #fff; font-family: Arial; }
        #video { width: 100vw; height: 100vh; object-fit: contain; }
        #info { position: absolute; top: 10px; left: 10px; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; }
        .good { color: #4CAF50; }
        .warning { color: #FFC107; }
        .bad { color: #F44336; }
        #error { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); background: rgba(255,0,0,0.8); padding: 20px; border-radius: 10px; display: none; }
    </style>
</head>
<body>
    <video id="video" autoplay playsinline muted></video>
    <div id="info">
        <div>Status: <span id="status">Connecting...</span></div>
        <div>Latency: <span id="latency">-</span> ms</div>
        <div>FPS: <span id="fps">-</span></div>
    </div>
    <div id="error">
        <h3>Connection Error</h3>
        <p id="errorMsg"></p>
        <button onclick="location.reload()">Retry</button>
    </div>
    
    <script>
        const video = document.getElementById('video');
        const statusEl = document.getElementById('status');
        const latencyEl = document.getElementById('latency');
        const fpsEl = document.getElementById('fps');
        const errorEl = document.getElementById('error');
        const errorMsgEl = document.getElementById('errorMsg');
        
        let frameCount = 0;
        let lastTime = performance.now();
        
        function showError(msg) {
            errorMsgEl.textContent = msg;
            errorEl.style.display = 'block';
            statusEl.textContent = 'Error';
            statusEl.className = 'bad';
        }
        
        async function start() {
            try {
                const pc = new RTCPeerConnection({
                    iceServers: [{urls: 'stun:stun.l.google.com:19302'}]
                });
                
                pc.addEventListener('track', (evt) => {
                    console.log('Received track:', evt.track.kind);
                    video.srcObject = evt.streams[0];
                    statusEl.textContent = 'Connected';
                    statusEl.className = 'good';
                });
                
                pc.addEventListener('connectionstatechange', () => {
                    console.log('Connection state:', pc.connectionState);
                    statusEl.textContent = pc.connectionState;
                    if (pc.connectionState === 'connected') {
                        statusEl.className = 'good';
                    } else if (pc.connectionState === 'connecting') {
                        statusEl.className = 'warning';
                    } else if (pc.connectionState === 'failed') {
                        statusEl.className = 'bad';
                        showError('WebRTC connection failed. Check console for details.');
                    }
                });
                
                pc.addEventListener('iceconnectionstatechange', () => {
                    console.log('ICE connection state:', pc.iceConnectionState);
                });
                
                // Add transceiver for video
                pc.addTransceiver('video', {direction: 'recvonly'});
                
                // Create offer
                const offer = await pc.createOffer();
                await pc.setLocalDescription(offer);
                
                // Send to server
                const response = await fetch('/offer', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({sdp: offer.sdp, type: offer.type})
                });
                
                if (!response.ok) {
                    const error = await response.json();
                    throw new Error(`Server error: ${error.error || response.statusText}`);
                }
                
                const answer = await response.json();
                await pc.setRemoteDescription(answer);
                
                // Monitor stats
                setInterval(async () => {
                    try {
                        const stats = await pc.getStats();
                        stats.forEach(report => {
                            if (report.type === 'inbound-rtp' && report.mediaType === 'video') {
                                // Calculate FPS
                                if (report.framesDecoded !== undefined) {
                                    const now = performance.now();
                                    const elapsed = (now - lastTime) / 1000;
                                    if (elapsed >= 1) {
                                        const fps = ((report.framesDecoded - frameCount) / elapsed).toFixed(1);
                                        fpsEl.textContent = fps;
                                        frameCount = report.framesDecoded;
                                        lastTime = now;
                                    }
                                }
                                
                                // Estimate latency
                                const jitterBufferDelay = report.jitterBufferDelay || 0;
                                const jitterBufferEmittedCount = report.jitterBufferEmittedCount || 1;
                                if (jitterBufferEmittedCount > 0) {
                                    const avgDelay = Math.round((jitterBufferDelay / jitterBufferEmittedCount) * 1000);
                                    latencyEl.textContent = avgDelay;
                                    if (avgDelay < 100) {
                                        latencyEl.className = 'good';
                                    } else if (avgDelay < 250) {
                                        latencyEl.className = 'warning';
                                    } else {
                                        latencyEl.className = 'bad';
                                    }
                                }
                            }
                        });
                    } catch (e) {
                        console.warn('Stats error:', e);
                    }
                }, 1000);
                
            } catch (error) {
                console.error('WebRTC error:', error);
                showError(error.message);
            }
        }
        
        // Auto-retry on video error
        video.addEventListener('error', (e) => {
            console.error('Video error:', e);
            showError('Video playback error. Retrying...');
            setTimeout(() => location.reload(), 3000);
        });
        
        start();
    </script>
</body>
</html>
    """
    return web.Response(text=html, content_type='text/html')

async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

def main():
    print("🚀 Simple WebRTC Server for Insta360 X3")
    print("======================================")
    
    # Check if HTTP stream is available
    try:
        r = requests.get('http://localhost:8080', timeout=2, stream=True)
        r.close()
        print("✅ Insta360 HTTP stream detected")
    except:
        print("❌ No stream on port 8080. Start the Insta360 streamer first!")
        print("   Run: ./insta360_simple_streamer")
        return
    
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    
    # Setup CORS
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*"
        )
    })
    
    # Routes
    app.router.add_get('/', index)
    app.router.add_post('/offer', offer)
    
    for route in list(app.router.routes()):
        cors.add(route)
    
    # Use different port to avoid conflict
    port = 8081
    print(f"\n🌐 WebRTC server starting on http://localhost:{port}")
    print("\n📱 Open in browser for low-latency viewing")
    print("   Expected latency: 100-200ms")
    print("\nPress Ctrl+C to stop...")
    
    web.run_app(app, host='0.0.0.0', port=port)

if __name__ == '__main__':
    main() 