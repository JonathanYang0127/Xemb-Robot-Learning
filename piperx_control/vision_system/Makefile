# Makefile for Insta360 X3 RTSP Streamer

CXX = g++
CXXFLAGS = -std=c++11 -Wall -O2
INCLUDES = -I./CameraSDK-20250418_145834-2.0.2-Linux/include
LIBS = -L./CameraSDK-20250418_145834-2.0.2-Linux/lib -lCameraSDK -lpthread
GSTREAMER_FLAGS = $(shell pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0)
GSTREAMER_LIBS = $(shell pkg-config --libs gstreamer-1.0 gstreamer-app-1.0)

# Targets
all: insta360_rtsp_streamer insta360_simple_streamer

insta360_rtsp_streamer: insta360_rtsp_streamer.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(GSTREAMER_FLAGS) -o $@ $< $(LIBS) $(GSTREAMER_LIBS)

insta360_simple_streamer: insta360_simple_streamer.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(LIBS)

clean:
	rm -f insta360_rtsp_streamer insta360_simple_streamer

.PHONY: all clean 