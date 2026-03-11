# Makefile pour le test caméra standalone
# Usage : make -f makefile_camera && ./test_camera [gstreamer|v4l2]

MAKEDIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
DEV := $(abspath $(MAKEDIR)../../Dev)

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra \
            -I$(DEV)/includes \
            -I$(DEV)/lib/include \
            $(shell pkg-config --cflags opencv4)
LDFLAGS := $(shell pkg-config --libs opencv4)

# Librairie LIDAR
LIDAR_LIB := $(DEV)/lib/bin/libsl_lidar_sdk.a

TARGET       := test_camera
SRCS         := $(MAKEDIR)test_camera.cpp $(DEV)/src/CameraProcessing.cpp

TARGET_ARUCO := test_camera_aruco
SRCS_ARUCO   := $(MAKEDIR)test_camera_aruco.cpp $(DEV)/src/CameraProcessing.cpp

.PHONY: all clean run run-v4l2 aruco run-aruco run-aruco-v4l2

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LIDAR_LIB) $(LDFLAGS) -o $@

aruco: $(TARGET_ARUCO)

$(TARGET_ARUCO): $(SRCS_ARUCO)
	$(CXX) $(CXXFLAGS) $(SRCS_ARUCO) $(LDFLAGS) -o $@

run: $(TARGET)
	./$(TARGET) gstreamer

run-v4l2: $(TARGET)
	./$(TARGET) v4l2

run-aruco: $(TARGET_ARUCO)
	./$(TARGET_ARUCO)

run-aruco-v4l2: $(TARGET_ARUCO)
	./$(TARGET_ARUCO) v4l2

clean:
	rm -f $(TARGET) $(TARGET_ARUCO)
