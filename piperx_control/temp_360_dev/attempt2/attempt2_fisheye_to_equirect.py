#!/usr/bin/env python3
"""
Dual Fisheye to Equirectangular Converter for Insta360 X3
Receives dual fisheye video from UDP and converts to equirectangular for Vuer
OPTIMIZED FOR LOW LATENCY
"""

import cv2
import numpy as np
import socket
import struct
import threading
import time
import argparse
from collections import defaultdict
import io
import base64

# Configuration
UDP_LISTEN_PORT = 8082
OUTPUT_UDP_PORT = 8083
OUTPUT_ADDRESS = "127.0.0.1"
MAX_PACKET_SIZE = 1400

# Processing configuration for low latency
EQUIRECT_WIDTH = 2048   # Lower resolution for speed
EQUIRECT_HEIGHT = 1024
TARGET_FPS = 30

class VideoPacketAssembler:
    """Assembles fragmented UDP video packets"""
    
    def __init__(self):
        self.packets = defaultdict(dict)  # packet_id -> {chunk_index: data}
        self.completed_packets = {}
        self.packet_lock = threading.Lock()
        
    def add_chunk(self, packet_id, chunk_index, total_chunks, data, timestamp, stream_index):
        """Add a chunk and return completed packet if available"""
        with self.packet_lock:
            self.packets[packet_id][chunk_index] = data
            
            # Check if packet is complete
            if len(self.packets[packet_id]) == total_chunks:
                # Assemble complete packet
                complete_data = b''.join(
                    self.packets[packet_id][i] for i in range(total_chunks)
                )
                
                # Clean up
                del self.packets[packet_id]
                
                return {
                    'data': complete_data,
                    'timestamp': timestamp,
                    'stream_index': stream_index
                }
                
        return None

class DualFisheyeConverter:
    """Converts dual fisheye to equirectangular projection"""
    
    def __init__(self, output_width=EQUIRECT_WIDTH, output_height=EQUIRECT_HEIGHT):
        self.output_width = output_width
        self.output_height = output_height
        self.maps_initialized = False
        self.front_map_x = None
        self.front_map_y = None
        self.back_map_x = None
        self.back_map_y = None
        
    def initialize_maps(self, fisheye_width, fisheye_height):
        """Initialize the remapping coordinates for fisheye to equirectangular"""
        print(f"Initializing fisheye maps for {fisheye_width}x{fisheye_height} -> {self.output_width}x{self.output_height}")
        
        # Create coordinate maps for equirectangular output
        self.front_map_x = np.zeros((self.output_height, self.output_width), dtype=np.float32)
        self.front_map_y = np.zeros((self.output_height, self.output_width), dtype=np.float32)
        self.back_map_x = np.zeros((self.output_height, self.output_width), dtype=np.float32)
        self.back_map_y = np.zeros((self.output_height, self.output_width), dtype=np.float32)
        
        # Fisheye parameters (these may need adjustment for X3)
        fisheye_center_x = fisheye_width / 2
        fisheye_center_y = fisheye_height / 2
        fisheye_radius = min(fisheye_width, fisheye_height) / 2 * 0.9  # 90% of radius
        
        for y in range(self.output_height):
            for x in range(self.output_width):
                # Convert equirectangular coordinates to spherical
                lon = (x / self.output_width) * 2 * np.pi - np.pi  # [-π, π]
                lat = (y / self.output_height) * np.pi - np.pi/2   # [-π/2, π/2]
                
                # Convert spherical to 3D Cartesian
                cart_x = np.cos(lat) * np.cos(lon)
                cart_y = np.cos(lat) * np.sin(lon)
                cart_z = np.sin(lat)
                
                # Determine which fisheye (front or back)
                # Front fisheye: -π/2 < lon < π/2
                # Back fisheye: π/2 < lon < 3π/2 (or -π/2 < lon < π/2 after wrapping)
                
                if -np.pi/2 <= lon <= np.pi/2:  # Front hemisphere
                    # Project to front fisheye
                    if cart_z >= 0:  # Visible in front camera
                        # Stereographic projection
                        if cart_z < 0.99:  # Avoid division by zero
                            proj_x = cart_x / (1 + cart_z)
                            proj_y = cart_y / (1 + cart_z)
                            
                            # Convert to fisheye coordinates
                            r = np.sqrt(proj_x**2 + proj_y**2)
                            if r <= 1.0:  # Within fisheye circle
                                theta = np.arctan2(proj_y, proj_x)
                                
                                # Map to fisheye image coordinates
                                fish_x = fisheye_center_x + r * fisheye_radius * np.cos(theta)
                                fish_y = fisheye_center_y + r * fisheye_radius * np.sin(theta)
                                
                                if 0 <= fish_x < fisheye_width and 0 <= fish_y < fisheye_height:
                                    self.front_map_x[y, x] = fish_x
                                    self.front_map_y[y, x] = fish_y
                else:  # Back hemisphere
                    # Adjust longitude for back camera
                    back_lon = lon + np.pi if lon < 0 else lon - np.pi
                    
                    # Recalculate Cartesian for back view
                    back_cart_x = np.cos(lat) * np.cos(back_lon)
                    back_cart_y = np.cos(lat) * np.sin(back_lon)
                    back_cart_z = -cart_z  # Flip Z for back camera
                    
                    if back_cart_z >= 0:  # Visible in back camera
                        # Stereographic projection
                        if back_cart_z < 0.99:
                            proj_x = back_cart_x / (1 + back_cart_z)
                            proj_y = back_cart_y / (1 + back_cart_z)
                            
                            # Convert to fisheye coordinates
                            r = np.sqrt(proj_x**2 + proj_y**2)
                            if r <= 1.0:
                                theta = np.arctan2(proj_y, proj_x)
                                
                                # Map to fisheye image coordinates
                                fish_x = fisheye_center_x + r * fisheye_radius * np.cos(theta)
                                fish_y = fisheye_center_y + r * fisheye_radius * np.sin(theta)
                                
                                if 0 <= fish_x < fisheye_width and 0 <= fish_y < fisheye_height:
                                    self.back_map_x[y, x] = fish_x
                                    self.back_map_y[y, x] = fish_y
        
        self.maps_initialized = True
        print("Fisheye to equirectangular maps initialized")
    
    def convert_frames(self, front_frame, back_frame):
        """Convert dual fisheye frames to equirectangular"""
        if front_frame is None or back_frame is None:
            return None
            
        if not self.maps_initialized:
            h, w = front_frame.shape[:2]
            self.initialize_maps(w, h)
        
        # Create output frame
        equirect = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)
        
        # Remap front fisheye
        front_remapped = cv2.remap(front_frame, self.front_map_x, self.front_map_y, 
                                  cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
        
        # Remap back fisheye  
        back_remapped = cv2.remap(back_frame, self.back_map_x, self.back_map_y,
                                 cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
        
        # Combine front and back (simple blending where both exist)
        mask_front = (self.front_map_x > 0) & (self.front_map_y > 0)
        mask_back = (self.back_map_x > 0) & (self.back_map_y > 0)
        
        equirect[mask_front] = front_remapped[mask_front]
        equirect[mask_back & ~mask_front] = back_remapped[mask_back & ~mask_front]
        
        # Blend overlapping regions
        overlap = mask_front & mask_back
        if np.any(overlap):
            equirect[overlap] = (front_remapped[overlap].astype(np.float32) * 0.5 + 
                               back_remapped[overlap].astype(np.float32) * 0.5).astype(np.uint8)
        
        return equirect

class Insta360VuerProcessor:
    """Main processor that receives UDP video and converts for Vuer"""
    
    def __init__(self):
        self.assembler = VideoPacketAssembler()
        self.converter = DualFisheyeConverter()
        self.front_decoder = cv2.VideoCapture()
        self.back_decoder = cv2.VideoCapture()
        
        # Frame buffers for dual fisheye
        self.front_frame = None
        self.back_frame = None
        self.frame_lock = threading.Lock()
        
        # UDP output socket
        self.output_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.output_addr = (OUTPUT_ADDRESS, OUTPUT_UDP_PORT)
        
        # Performance monitoring
        self.last_stats_time = time.time()
        self.frames_processed = 0
        
    def decode_h264_frame(self, h264_data, decoder):
        """Decode H.264 data to frame using OpenCV"""
        try:
            # Write H.264 data to temporary file (not ideal but works with OpenCV)
            # In production, consider using FFmpeg Python bindings for better performance
            temp_file = f"/tmp/temp_video_{threading.get_ident()}.h264"
            with open(temp_file, 'wb') as f:
                f.write(h264_data)
            
            # Open with OpenCV
            cap = cv2.VideoCapture(temp_file)
            ret, frame = cap.read()
            cap.release()
            
            # Clean up temp file
            import os
            try:
                os.remove(temp_file)
            except:
                pass
                
            return frame if ret else None
            
        except Exception as e:
            print(f"Error decoding H.264: {e}")
            return None
    
    def process_video_packet(self, packet):
        """Process a complete video packet"""
        stream_index = packet['stream_index']
        h264_data = packet['data']
        
        # Decode H.264 to frame
        if stream_index == 0:  # Front fisheye
            frame = self.decode_h264_frame(h264_data, self.front_decoder)
            if frame is not None:
                with self.frame_lock:
                    self.front_frame = frame
        elif stream_index == 1:  # Back fisheye
            frame = self.decode_h264_frame(h264_data, self.back_decoder)
            if frame is not None:
                with self.frame_lock:
                    self.back_frame = frame
        
        # Try to create equirectangular frame if we have both
        with self.frame_lock:
            if self.front_frame is not None and self.back_frame is not None:
                equirect = self.converter.convert_frames(self.front_frame, self.back_frame)
                if equirect is not None:
                    self.send_equirectangular_frame(equirect)
                    self.frames_processed += 1
    
    def send_equirectangular_frame(self, frame):
        """Send equirectangular frame via UDP for Vuer"""
        try:
            # Encode frame as JPEG for transmission
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # Convert to base64 for easy transmission
            jpg_data = buffer.tobytes()
            b64_data = base64.b64encode(jpg_data).decode('utf-8')
            
            # Create packet with metadata
            packet = {
                'type': 'equirectangular_frame',
                'timestamp': time.time(),
                'width': frame.shape[1],
                'height': frame.shape[0],
                'data': b64_data
            }
            
            # Send as JSON
            import json
            json_data = json.dumps(packet).encode('utf-8')
            
            # Fragment if needed
            if len(json_data) > MAX_PACKET_SIZE:
                # Send in chunks
                chunks = [json_data[i:i+MAX_PACKET_SIZE] for i in range(0, len(json_data), MAX_PACKET_SIZE)]
                for i, chunk in enumerate(chunks):
                    header = struct.pack('!III', len(chunks), i, len(chunk))
                    self.output_socket.sendto(header + chunk, self.output_addr)
            else:
                # Send single packet
                header = struct.pack('!III', 1, 0, len(json_data))
                self.output_socket.sendto(header + json_data, self.output_addr)
                
        except Exception as e:
            print(f"Error sending equirectangular frame: {e}")
    
    def stats_thread(self):
        """Print performance statistics"""
        while True:
            time.sleep(10)
            now = time.time()
            elapsed = now - self.last_stats_time
            fps = self.frames_processed / elapsed if elapsed > 0 else 0
            
            print(f"📊 360° Processing: {fps:.1f} FPS, {self.frames_processed} frames processed")
            
            self.frames_processed = 0
            self.last_stats_time = now
    
    def run(self):
        """Main processing loop"""
        print(f"🎥 Starting Insta360 to Vuer processor...")
        print(f"📡 Listening for dual fisheye on UDP port {UDP_LISTEN_PORT}")
        print(f"📤 Sending equirectangular to {OUTPUT_ADDRESS}:{OUTPUT_UDP_PORT}")
        
        # Start stats thread
        stats_thread = threading.Thread(target=self.stats_thread, daemon=True)
        stats_thread.start()
        
        # Setup UDP listener
        listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listen_socket.bind(('0.0.0.0', UDP_LISTEN_PORT))
        listen_socket.settimeout(1.0)
        
        print("🚀 Processor ready - waiting for dual fisheye video data...")
        
        while True:
            try:
                data, addr = listen_socket.recvfrom(MAX_PACKET_SIZE + 100)
                
                # Parse packet header
                header_size = struct.calcsize('!IIIIIQ')  # packet_id, stream_index, total_size, chunk_index, total_chunks, timestamp
                if len(data) < header_size:
                    continue
                    
                header = struct.unpack('!IIIIIQ', data[:header_size])
                packet_id, stream_index, total_size, chunk_index, total_chunks, timestamp = header
                payload = data[header_size:]
                
                # Assemble packet
                complete_packet = self.assembler.add_chunk(
                    packet_id, chunk_index, total_chunks, payload, timestamp, stream_index
                )
                
                if complete_packet:
                    self.process_video_packet(complete_packet)
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error processing UDP data: {e}")

def main():
    parser = argparse.ArgumentParser(description='Insta360 X3 Dual Fisheye to Equirectangular Converter')
    parser.add_argument('--input-port', type=int, default=UDP_LISTEN_PORT,
                       help=f'UDP port to listen for dual fisheye video (default: {UDP_LISTEN_PORT})')
    parser.add_argument('--output-port', type=int, default=OUTPUT_UDP_PORT,
                       help=f'UDP port to send equirectangular video (default: {OUTPUT_UDP_PORT})')
    parser.add_argument('--output-address', default=OUTPUT_ADDRESS,
                       help=f'Address to send equirectangular video (default: {OUTPUT_ADDRESS})')
    parser.add_argument('--width', type=int, default=EQUIRECT_WIDTH,
                       help=f'Output equirectangular width (default: {EQUIRECT_WIDTH})')
    parser.add_argument('--height', type=int, default=EQUIRECT_HEIGHT,
                       help=f'Output equirectangular height (default: {EQUIRECT_HEIGHT})')
    
    args = parser.parse_args()
    
    global UDP_LISTEN_PORT, OUTPUT_UDP_PORT, OUTPUT_ADDRESS, EQUIRECT_WIDTH, EQUIRECT_HEIGHT
    UDP_LISTEN_PORT = args.input_port
    OUTPUT_UDP_PORT = args.output_port
    OUTPUT_ADDRESS = args.output_address
    EQUIRECT_WIDTH = args.width
    EQUIRECT_HEIGHT = args.height
    
    processor = Insta360VuerProcessor()
    
    try:
        processor.run()
    except KeyboardInterrupt:
        print("\n🛑 Converter stopped by user")
    except Exception as e:
        print(f"❌ Converter error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()