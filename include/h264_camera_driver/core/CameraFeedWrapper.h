// Copyright by BeeX [2024]
#ifndef CAMERA_FEED_WRAPPER_H
#define CAMERA_FEED_WRAPPER_H

// Helper files
#include <h264_camera_driver/core/ClaheEnhancement.h>
#include <h264_camera_driver/core/H264Encoder.h>
#include <h264_camera_driver/core/RectifyImageYUV.h>
#include <h264_camera_driver/core/SSHHelper.h>
#include <bx_msgs/RosBindings.hpp>

// Standard Library
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

// Argus and NV libs
#include <Argus/Argus.h>
#include <Argus/Ext/DeFog.h>
#include <Argus/Ext/FaceDetect.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <NvJpegEncoder.h>
#include <Value.h>

// NvUtils (New API)
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

// Threading
#include <mutex>

using namespace Argus;
using namespace EGLStream;

#define IMAGE_WIDTH_2K   2'560
#define IMAGE_HEIGHT_2K  1'440
#define MAX_JPEG_SIZE_2K (IMAGE_WIDTH_2K * IMAGE_HEIGHT_2K * 3 / 2)

#define IMAGE_WIDTH_720  1'280
#define IMAGE_HEIGHT_720 720

#define VEHICLE_INFO_PATH "/home/aib/.ikan-bilis/VehicleInfo.yaml"
#define MAIN_HULL_FIELD   "Main_Hull_ID"
#define VEHICLE_IP        "192.168.1.1"
#define VEHICLE_USERNAME  "aib"
#define VEHICLE_PWD       "bb"

#define VEHICLE_ODOMETRY_TOPIC "/ikan/nav/world_ned_msl"

class CameraFeedWrapper {

public:
    CameraFeedWrapper(
            const std::string frame_id,
            const std::string ros_topic_root_name,
            std::string calibration_file_path,
            const bool use_hw_jpeg_encoder = true);
    ~CameraFeedWrapper();


    virtual void captureAndPublishOnce(Srv_CamGetSnapshot_Response *res_ptr) = 0;

protected:
    // MIGRATION: Changed from int32_t fd to NvBufSurface*
    NvBufSurface *dma_buf_2K;
    float camera_depth = 0.0F;

    bool publishH264Feed(const NV::IImageNativeBuffer *iNativeBuffer, Srv_CamGetSnapshot_Response *res_ptr);
    bool publishH264Feed(Srv_CamGetSnapshot_Response *res_ptr, uint64_t timestamp_ms);
    bool publishH264Feed(cv::Mat &in_yuv_img, Srv_CamGetSnapshot_Response *res_ptr);

    void loadCalibrationFile(std::string &calibration_file_path);
    std::string isExistsCalibration(std::string &calibration_file_path);

    std::string mainHullId;
    std::string UpdatedCalibrationPath;

private:
    bool spinH264FeedOnce(Srv_CamGetSnapshot_Response *res_ptr);
    std::recursive_mutex calibration_mutex;

    uint64_t ipcam_timestamp_ms;

    // Camera interface settings
    std::mutex feed_mutex;

    // MIGRATION: Changed from int32_t fd to NvBufSurface*
    NvBufSurface *dma_buf_HD;

    // Helper Class
    cv::Mat img_YUV_2K, img_YUV_FHD;
    SafeSharedPtr<H264Encoder> h264_clahe_2k_encoder, h264_raw_hd_encoder, h264_clahe_hd_encoder;
    SafeSharedPtr<NvJPEGEncoder> jpeg_encoder;

    ClaheEnhancement claheEnhance;

    SafeSharedPtr<RectifyImageYUV> rectifyImageYUVPtr;

    bool use_clahe_feed = false;
    ros::Time feed_timestamp;

    // Subscriber
    DECLARE_ROS_SUBSCRIBER(sub_camera_depth, Msg_VehicleState)

    // Publisher
    Msg_ImageH264Feed h264_msg;
    DECLARE_ROS_PUBLISHER(pub_clahe_2k_h264, Msg_ImageH264Feed)
    DECLARE_ROS_PUBLISHER(pub_raw_hd_h264, Msg_ImageH264Feed)
    DECLARE_ROS_PUBLISHER(pub_clahe_hd_h264, Msg_ImageH264Feed)

    Msg_CompressedImage clahe_224_jpeg_msg;
    DECLARE_ROS_PUBLISHER(pub_clahe_224_jpeg, Msg_CompressedImage)

    // ROS service - Get Raw Image snapshot
    DECLARE_ROS_SERVICE_SERVER(srv_get_snapshot_server, Srv_CamGetSnapshot)

    // -----------------------------------------------------
    // Initialisation functions
    // -----------------------------------------------------
    // MIGRATION: Returns NvBufSurface* instead of fd
    NvBufSurface *initialiseDmaBuffer(const int32_t img_width, const int32_t img_height);
    void initialiseRosBindings(const std::string &frame_id, const std::string &ros_topic_root_name);


    // -----------------------------------------------------
    // NvBuffer Helper functions
    // -----------------------------------------------------
    // MIGRATION: Takes NvBufSurface*
    bool nvBuffer2CvMat(cv::Mat &out_yuv_image, NvBufSurface *src_surf);
    bool CvMat2NvBuffer(const cv::Mat &yuv_image, NvBufSurface *dst_surf);
    bool resizeFrame(
            NvBufSurface *srcSurf,
            NvBufSurface *dstSurf,
            const int32_t srcWidth,
            const int32_t srcHeight,
            const int32_t dstWidth,
            const int32_t dstHeight);

    cv::Mat jpeg_yuv_image, jpeg_bgr_image;
    void NvBuffer2Jpeg(NvBufSurface *src_surf, std::vector<uint8_t> &jpeg_buffer);


    // -----------------------------------------------------
    // ROS Callbacks
    // -----------------------------------------------------
    void depthCallback(const Msg_VehicleState::ConstPtr &msg);
    bool getImgSrvCallback(Srv_CamGetSnapshot_Request &req, Srv_CamGetSnapshot_Response &res);

    cv::Mat image_yuv_2K, rectified_yuv_2K;
    bool initialised = false;
    std::string lastCalibrationPath_;
};

#endif  // CAMERA_FEED_WRAPPER_H