// Copyright by BeeX [2024]
#include <h264_camera_driver/jpeg/mjpeg_main.h>

MJPEGDecoder::MJPEGDecoder(
        const std::string source_ros_topic,
        const std::string frame_id,
        const std::string ros_topic_root_name,
        const std::string calibration_file_name)
        : CameraFeedWrapper(frame_id, ros_topic_root_name, calibration_file_name, false) {

    INIT_ROS_SUBSCRIBER(sub_image, source_ros_topic, 1, &MJPEGDecoder::source_image_cb);
}

void MJPEGDecoder::captureAndPublishOnce(Srv_CamGetSnapshot_Response *res_ptr) {
    // Publish the rectified image
    CameraFeedWrapper::publishH264Feed(raw_frame, res_ptr);
}

void MJPEGDecoder::capture_n_pub() {

    LOG_INFO("Starting Capture & publish session...\r\n");

    RateLimiter rate_limiter(30);

    while (IS_ROS_NODE_OK()) {
        ROS_SPIN_ONCE();
    }

    LOG_INFO("Closing session...");
}

void MJPEGDecoder::source_image_cb(Msg_CompressedImage_ConstPtr msg) {

    raw_frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::resize(raw_frame, raw_frame, cv::Size(IMAGE_WIDTH_2K, IMAGE_HEIGHT_2K));
    cv::cvtColor(raw_frame, raw_frame, cv::COLOR_BGR2YUV_I420);
    MJPEGDecoder::captureAndPublishOnce(nullptr);
}


DECLARE_ROS_NODE_HANDLE
int main(int argc, char *argv[]) {
    INIT_ROS_NODE("external_camera_driver", 0, "external_camera_driver/alive")

    std::string source_ros_topic, frame_id, topic_name, calibration_file_name;

    GET_ROS_PARAM("~source_ros_topic", source_ros_topic, "/ikan/front_cam/image_color/compressed");
    GET_ROS_PARAM("~frame_id", frame_id, "front_cam");
    GET_ROS_PARAM("~topic_name", topic_name, "/ikan/front_cam");
    GET_ROS_PARAM("~calibration_file_name", calibration_file_name, "calibration_imx185.yaml");

    const std::string calibration_path =
            ros::package::getPath("h264_camera_driver") + "/config/" + calibration_file_name;

    printf("Waiting for ROS Master to come back up\n");
    while (!IS_ROS_NODE_OK()) {
        printf(".");
        sleep(1);
    }
    printf("OK\n\nStarting Camera Driver.\n");

    MJPEGDecoder cam_driver(source_ros_topic, frame_id, topic_name, calibration_path);
    cam_driver.capture_n_pub();

    return 0;
}
