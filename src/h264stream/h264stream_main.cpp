// Copyright by BeeX [2024]
#include <h264_camera_driver/h264stream/h264stream_main.h>

H264StreamDecoder::H264StreamDecoder(
        const std::string source_h264_ros_topic,
        const std::string source_h264_config_topic,
        const std::string frame_id,
        const std::string ros_topic_root_name,
        const std::string calibration_file_name)
        : CameraFeedWrapper(frame_id, ros_topic_root_name, calibration_file_name),
          should_continue_capture(true),
          h264_decoder() {

    INIT_ROS_SUBSCRIBER(sub_image, source_h264_ros_topic, 8, &H264StreamDecoder::source_h264_cb);
    INIT_ROS_SUBSCRIBER(sub_config, source_h264_config_topic, 8, &H264StreamDecoder::source_h264_config_cb);
}

void H264StreamDecoder::captureAndPublishOnce(Srv_CamGetSnapshot_Response *res_ptr) {
    // Publish the rectified image
    CameraFeedWrapper::publishH264Feed(res_ptr, ipcam_timestamp_ms);
}

void H264StreamDecoder::capture_decode_publish_fn() {

    RateLimiter rate_limiter(CAMERA_FPS);

    while (should_continue_capture) {
        rate_limiter.sleep();
        if (h264_decoder.decoder_get_frame(CameraFeedWrapper::dma_buf_2K)) {
            captureAndPublishOnce(nullptr);
        }
    }
}

void H264StreamDecoder::capture_n_pub() {

    LOG_INFO("Starting Capture & publish session...\r\n");

    RateLimiter rate_limiter(2 * CAMERA_FPS);
    std::thread thread_capture(&H264StreamDecoder::capture_decode_publish_fn, this);

    while (IS_ROS_NODE_OK()) {
        rate_limiter.sleep();
        ROS_SPIN_ONCE();
    }

    should_continue_capture = false;
    thread_capture.join();

    LOG_INFO("Closing session...");
}

void H264StreamDecoder::source_h264_cb(Msg_ImageH264Feed_ConstPtr msg) {
    h264_decoder.decoder_put_packet(msg);
    ipcam_timestamp_ms = msg->unix_time_ms;
}

void H264StreamDecoder::source_h264_config_cb(Msg_FullDepthCamConfig_ConstPtr msg) {
    mode      = msg->conf_mode_outdoor;
    serial_id = msg->camera_serial_num;
    if (mode == 0) {
        calibrationMode = "calibration_seaWater.yaml";
    } else if (mode == 1) {
        calibrationMode = "calibration_air.yaml";
    }
    std::string updatedPath;
    updatedPath = CALIBRATIONS_PATH + serial_id + "_" + calibrationMode;
    updatedPath = isExistsCalibration(updatedPath);
    loadCalibrationFile(updatedPath);
}

DECLARE_ROS_NODE_HANDLE
int main(int argc, char *argv[]) {
    INIT_ROS_NODE("external_camera_driver", 0, "external_camera_driver/alive")

    std::string source_ros_topic, source_config_topic, frame_id, topic_name, calibration_file_name;

    GET_ROS_PARAM("~source_ros_topic", source_ros_topic, "/ikan/ip_cam/raw_h264");
    GET_ROS_PARAM("~source_config_topic", source_config_topic, "/ikan/ip_cam/camera_config");
    GET_ROS_PARAM("~frame_id", frame_id, "ip_cam");
    GET_ROS_PARAM("~topic_name", topic_name, "/ikan/ip_cam");
    GET_ROS_PARAM("~calibration_file_name", calibration_file_name, "calibration_seaWater.yaml");

    const std::string calibration_path = CALIBRATIONS_PATH + calibration_file_name;

    printf("Waiting for ROS Master to come back up\n");
    while (!IS_ROS_NODE_OK()) {
        printf(".");
        sleep(1);
    }
    printf("OK\n\nStarting Camera Driver.\n");

    H264StreamDecoder cam_driver(source_ros_topic, source_config_topic, frame_id, topic_name, calibration_path);
    cam_driver.capture_n_pub();

    return 0;
}
