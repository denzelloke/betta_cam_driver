// Copyright by BeeX [2024]
#ifndef H264_ROS_H
#define H264_ROS_H

#include <h264_camera_driver/core/CameraFeedWrapper.h>
#include <h264_camera_driver/core/H264Decoder.h>

#define CALIBRATIONS_PATH "/home/aibx/.xavier/calibrations/"
#define CAMERA_FPS        12  // Hz

class H264StreamDecoder : public CameraFeedWrapper {
public:
    H264StreamDecoder(
            const std::string source_h264_ros_topic,
            const std::string source_h264_config_topic,
            const std::string frame_id,
            const std::string ros_topic_root_name,
            const std::string calibration_file_name);

    void captureAndPublishOnce(Srv_CamGetSnapshot_Response *res_ptr) override;
    void capture_n_pub();

private:
    bool should_continue_capture;
    H264Decoder h264_decoder;

    void capture_decode_publish_fn();

    DECLARE_ROS_SUBSCRIBER(sub_image, Msg_ImageH264Feed);
    void source_h264_cb(Msg_ImageH264Feed_ConstPtr msg);

    DECLARE_ROS_SUBSCRIBER(sub_config, Msg_FullDepthCamConfig);
    void source_h264_config_cb(Msg_FullDepthCamConfig_ConstPtr msg);

    int mode;
    std::string calibrationMode;
    uint64_t ipcam_timestamp_ms;
    std::string serial_id;
};

#endif  // H264_ROS_H
