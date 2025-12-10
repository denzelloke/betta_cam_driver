cmake_minimum_required(VERSION 3.10)
project(h264_camera_driver)

# libssh Package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSSH REQUIRED libssh)

# Include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${LIBSSH_INCLUDE_DIRS}
    /usr/local/cuda/include
    /usr/include/libdrm
    /usr/include/aarch64-linux-gnu
    /usr/src/jetson_multimedia_api/include
    /usr/src/jetson_multimedia_api/include/libjpeg-8b
    /usr/src/jetson_multimedia_api/samples/common/algorithm/cuda
    /usr/src/jetson_multimedia_api/samples/common/algorithm/trt
    /usr/src/jetson_multimedia_api/argus/samples/utils
)

# Link directories
link_directories(
    /usr/local/cuda/lib64
    /usr/lib/aarch64-linux-gnu
    /usr/lib/aarch64-linux-gnu/tegra
)

# Compile Nvidia Files
file(GLOB NVFILES /usr/src/jetson_multimedia_api/samples/common/classes/*.cpp)
file(GLOB NV_THREAD_FILES /usr/src/jetson_multimedia_api/argus/sample/utils/Thread.cpp)

set(PROJECT_LIBRARIES
    pthread
    v4l2
    EGL
    GLESv2
    X11
    nvbufsurface
    nvbufsurftransform
    nvjpeg
    nvosd
    drm
    cuda
    cudart
    nvinfer
    nvparsers
    # nveglstream_camconsumer idk why i needed to delete this but ima just do it for now...
    nvargus_socketclient
    ${LIBSSH_LIBRARIES}
    ssh
)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

compile_my_library(lib_h264_nvidia
    ${NVFILES}
    ${NV_THREAD_FILES})
set(PROJECT_LIBRARIES ${PROJECT_LIBRARIES} lib_h264_nvidia)

compile_my_library(lib_h264_driver
    src/core/CameraFeedWrapper.cpp
    src/core/ClaheEnhancement.cpp
    src/core/H264Encoder.cpp
    src/core/RectifyImageYUV.cpp
    src/core/SSHHelper.cpp)
set(PROJECT_LIBRARIES ${PROJECT_LIBRARIES} lib_h264_driver)

compile_my_exec(imx_477_node src/imx/imx_477_main.cpp)

compile_my_exec(external_jpeg_node src/jpeg/mjpeg_main.cpp)

compile_my_exec(external_h264_node
    src/core/H264Decoder.cpp
    src/h264stream/h264stream_main.cpp)

install_my_config_and_launch_files()
