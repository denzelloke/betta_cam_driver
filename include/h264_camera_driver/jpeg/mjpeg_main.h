// Copyright by BeeX [2024]
#ifndef MJPEG_H
#define MJPEG_H

#include <h264_camera_driver/core/CameraFeedWrapper.h>

#define CAMERA_FPS 15  // Hz

class MJPEGDecoder : public CameraFeedWrapper {
public:
    MJPEGDecoder(
            const std::string source_ros_topic,
            const std::string frame_id,
            const std::string ros_topic_root_name,
            const std::string calibration_file_name);

    void captureAndPublishOnce(Srv_CamGetSnapshot_Response *res_ptr) override;
    void capture_n_pub();

private:
    cv::Mat raw_frame;

    DECLARE_ROS_SUBSCRIBER(sub_image, Msg_CompressedImage);
    void source_image_cb(Msg_CompressedImage_ConstPtr msg);
};

#endif  // MJPEG_H
