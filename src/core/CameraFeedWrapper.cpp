// Copyright by BeeX [2024]
#include <h264_camera_driver/core/CameraFeedWrapper.h>


/*@brief: Jetson camera API design -> Objects do not have methods.
 *  All methods are provided by Interfaces.
 *  Driver runs 1 camera based on the port given, compresses the image
 *  using encodeFromFD which is hardware accelerated and finally publish the image
 *  reference: http://on-demand.gputechconf.com/gtc/2016/webinar/getting-started-jetpack-camera-api.pdf
 */
CameraFeedWrapper::CameraFeedWrapper(
        const std::string frame_id,
        const std::string ros_topic_root_name,
        std::string calibration_file_path,
        const bool use_hw_jpeg_encoder)
        : dma_buf_2K(initialiseDmaBuffer(IMAGE_WIDTH_2K, IMAGE_HEIGHT_2K)),
          dma_buf_HD(initialiseDmaBuffer(IMAGE_WIDTH_720, IMAGE_HEIGHT_720)),

          h264_clahe_2k_encoder(nullptr),
          h264_raw_hd_encoder(nullptr),
          h264_clahe_hd_encoder(nullptr),

          claheEnhance() {

    // Retrieve Main_Hull_ID & FullDepth ID
    try {
        SSHHelper ssh(VEHICLE_IP, VEHICLE_USERNAME, VEHICLE_PWD);
        mainHullId = ssh.getId(VEHICLE_INFO_PATH, MAIN_HULL_FIELD);
    } catch (const std::exception &e) {
        std::cout << "Failed to retrieve Main_Hull_ID" << std::endl;
    }

    // Update the Calibration File Path
    if (calibration_file_path.find("calibration_imx185") != std::string::npos) {
        size_t lastSlash      = calibration_file_path.find_last_of('/');
        calibration_file_path = calibration_file_path.insert(lastSlash + 1, mainHullId + "_");
    }

    // Verify the existence of calibration file
    UpdatedCalibrationPath = isExistsCalibration(calibration_file_path);

    // Load the calibration file
    loadCalibrationFile(UpdatedCalibrationPath);

    // Initialise ROS bindings
    initialiseRosBindings(frame_id, ros_topic_root_name);

    h264_msg.seq_id       = 0;
    h264_clahe_2k_encoder = make_safe_shared<H264Encoder>(IMAGE_WIDTH_2K, IMAGE_HEIGHT_2K, 30'000, 5);
    h264_raw_hd_encoder   = make_safe_shared<H264Encoder>(IMAGE_WIDTH_720, IMAGE_HEIGHT_720, 12'000, 10, true);
    h264_clahe_hd_encoder = make_safe_shared<H264Encoder>(IMAGE_WIDTH_720, IMAGE_HEIGHT_720, 12'000, 10, true);

    if (use_hw_jpeg_encoder) {
        // Initialise JPEG encoder for Raw thread
        jpeg_encoder = SafeSharedPtr<NvJPEGEncoder>(
                std::shared_ptr<NvJPEGEncoder>(NvJPEGEncoder::createJPEGEncoder("jpeg_encoder")));
    }
}

/* @brief: Destructor, cleanS up buffer, thread and all sessions before exiting program */
CameraFeedWrapper::~CameraFeedWrapper() {

    NvBufferDestroy(dma_buf_2K);
    NvBufferDestroy(dma_buf_HD);

    pub_clahe_2k_h264.shutdown();
    pub_raw_hd_h264.shutdown();
    pub_clahe_hd_h264.shutdown();

    pub_clahe_224_jpeg.shutdown();
}

bool CameraFeedWrapper::publishH264Feed(
        const NV::IImageNativeBuffer *iNativeBuffer,
        Srv_CamGetSnapshot_Response *res_ptr) {

    if (!iNativeBuffer) {
        return false;
    }

    std::lock_guard<std::mutex> lock(feed_mutex);

    iNativeBuffer->copyToNvBuffer(dma_buf_2K);

    return spinH264FeedOnce(res_ptr);
}

bool CameraFeedWrapper::publishH264Feed(Srv_CamGetSnapshot_Response *res_ptr, uint64_t timestamp_ms) {

    ipcam_timestamp_ms = timestamp_ms;
    std::lock_guard<std::mutex> lock(feed_mutex);
    return spinH264FeedOnce(res_ptr);
}

bool CameraFeedWrapper::publishH264Feed(cv::Mat &in_yuv_img, Srv_CamGetSnapshot_Response *res_ptr) {

    if (in_yuv_img.empty()) {
        ROS_ERROR("Empty YUV image for H264 encoding");
        SET_NODE_STATUS_CRITICAL_ERROR();
        return false;
    }

    std::lock_guard<std::mutex> lock(feed_mutex);

    CvMat2NvBuffer(in_yuv_img, dma_buf_2K);
    return spinH264FeedOnce(res_ptr);
}

/* @brief: Load the Calibration file and store in the camera message  */
void CameraFeedWrapper::loadCalibrationFile(std::string &calibration_file_path) {

    // Load camera matrices from the YAML file into a map
    YAML::Node config = YAML::LoadFile(calibration_file_path);

    // Assign distortion coefficients to both messages
    std::vector<double> dist_coeffs = config["distortion_coefficients"]["data"].as<std::vector<double>>();
    for (int idx = 0; idx < std::min(5, static_cast<int>(dist_coeffs.size())); ++idx) {
        h264_msg.D[idx] = dist_coeffs[idx];
    }

    // Assign camera matrix parameters (assuming fx, fy, cx, cy format) to both messages
    std::vector<double> camera_matrix = config["camera_matrix"]["data"].as<std::vector<double>>();
    if (camera_matrix.size() >= 6) {
        h264_msg.K[0] = camera_matrix[0];  // fx
        h264_msg.K[1] = camera_matrix[4];  // fy
        h264_msg.K[2] = camera_matrix[2];  // cx
        h264_msg.K[3] = camera_matrix[5];  // cy
    }
}

/* @brief: Checks the existence of calibration file  */
std::string CameraFeedWrapper::isExistsCalibration(std::string &calibration_file) {
    std::ifstream fin(calibration_file);
    if (!fin) {
        ROS_ERROR("Unable to find the existence of calibration file\n");
        LOG_INFO("Loading default calibration\n");
        std::string path = ROS_PACKAGE_GETPATH("h264_camera_driver") + "/config/";

        if (calibration_file.find("calibration_imx185") != std::string::npos) {
            calibration_file = path + "calibration_imx185.yaml";
        } else if (calibration_file.find("calibration_seaWater") != std::string::npos) {
            calibration_file = path + "calibration_seaWater.yaml";
        } else if (calibration_file.find("calibration_air") != std::string::npos) {
            calibration_file = path + "calibration_air.yaml";
        }
    }

    // Ensure that it gets executed only when the last is calibration changed
    if ((calibration_file.find("calibration_seaWater") != std::string::npos
         || calibration_file.find("calibration_air") != std::string::npos)
        && calibration_file != lastCalibrationPath_) {
        std::lock_guard<std::recursive_mutex> lock(calibration_mutex);
        rectifyImageYUVPtr   = make_safe_shared<RectifyImageYUV>(calibration_file);
        lastCalibrationPath_ = calibration_file;
    }

    return calibration_file;
}

bool CameraFeedWrapper::spinH264FeedOnce(Srv_CamGetSnapshot_Response *res_ptr) {

    std::lock_guard<std::recursive_mutex> lock(calibration_mutex);
    if (rectifyImageYUVPtr) {
        ROS_TIME feed_timestamp;
        feed_timestamp.sec    = ipcam_timestamp_ms / 1'000;
        feed_timestamp.nsec   = (ipcam_timestamp_ms % 1'000) * 1'000'000;  // Convert remaining ms to ns
        h264_msg.unix_time_ms = ipcam_timestamp_ms;
    } else {
        ROS_TIME feed_timestamp = ROS_TIME_NOW();
        h264_msg.unix_time_ms   = ROS_TIME_TO_MS(feed_timestamp);
    }

    const bool is_even_frame   = ((h264_msg.seq_id & 0x01) == 0);
    const bool is_fourth_frame = ((h264_msg.seq_id & 0x03) == 0);

    // If using the full-depth feed, process rectification.
    {
        std::lock_guard<std::recursive_mutex> lock(calibration_mutex);
        if (rectifyImageYUVPtr) {
            nvBuffer2CvMat(image_yuv_2K, dma_buf_2K);

            // Perform rectification
            if (!rectifyImageYUVPtr->rectifyImage(rectified_yuv_2K, image_yuv_2K)) {
                ROS_ERROR("Image rectification failed");
                SET_NODE_STATUS_CRITICAL_ERROR();
                return false;
            }

            // Convert back to NvBuffer
            CvMat2NvBuffer(rectified_yuv_2K, dma_buf_2K);
        }
    }

    // Handles the JPEG encoding for the RAW feed
    if (res_ptr != nullptr) {
        res_ptr->seq_id       = h264_msg.seq_id;
        res_ptr->unix_time_ms = h264_msg.unix_time_ms;
        NvBuffer2Jpeg(dma_buf_2K, res_ptr->raw_image_data);
    }

    // Publish the Raw HD H264 feed
    resizeFrame(dma_buf_2K, dma_buf_HD, IMAGE_WIDTH_2K, IMAGE_HEIGHT_2K, IMAGE_WIDTH_720, IMAGE_HEIGHT_720);
    if (h264_raw_hd_encoder->encodeFrame(h264_msg, dma_buf_HD)) {
        PUBLISH_ROS(pub_raw_hd_h264, h264_msg);
        SET_NODE_STATUS_OK();

    } else {
        ROS_ERROR("H264 encoding failed for RAW HD frame - No Encoder");
        SET_NODE_STATUS_CRITICAL_ERROR();
    }

    // Process the CLAHE as a 2K Frame
    nvBuffer2CvMat(img_YUV_2K, dma_buf_2K);
    claheEnhance.apply_clahe(img_YUV_2K);
    CvMat2NvBuffer(img_YUV_2K, dma_buf_2K);

    // Handles the JPEG encoding for the CLAHE feed
    if (res_ptr != nullptr) {
        NvBuffer2Jpeg(dma_buf_2K, res_ptr->processed_image_data);
    }   

    // Publish the CLAHE HD H264 feed
    resizeFrame(dma_buf_2K, dma_buf_HD, IMAGE_WIDTH_2K, IMAGE_HEIGHT_2K, IMAGE_WIDTH_720, IMAGE_HEIGHT_720);
    if (h264_clahe_hd_encoder->encodeFrame(h264_msg, dma_buf_HD)) {
        PUBLISH_ROS(pub_clahe_hd_h264, h264_msg);
        SET_NODE_STATUS_OK();
    } else {
        ROS_ERROR("H264 encoding failed for CLAHE HD frame - No Encoder");
        SET_NODE_STATUS_CRITICAL_ERROR();
    }

    // Publish the CLAHE 2K H264 feed
    if (is_even_frame) {
        if (h264_clahe_2k_encoder->encodeFrame(h264_msg, dma_buf_2K)) {
            PUBLISH_ROS(pub_clahe_2k_h264, h264_msg);
            SET_NODE_STATUS_OK();
        } else {
            ROS_ERROR("H264 encoding failed for CLAHE 2K frame - No Encoder");
            SET_NODE_STATUS_CRITICAL_ERROR();
        }

        // Publish the JPEG compressed image
        if (is_fourth_frame) {
            clahe_224_jpeg_msg.header.stamp = feed_timestamp;
            NvBuffer2Jpeg(dma_buf_HD, clahe_224_jpeg_msg.data);
            PUBLISH_ROS(pub_clahe_224_jpeg, clahe_224_jpeg_msg);
        }

        SET_NODE_STATUS_OK();
    }

    h264_msg.seq_id++;
    return true;
}



// -----------------------------------------------------
// Initialisation functions
// -----------------------------------------------------

int32_t CameraFeedWrapper::initialiseDmaBuffer(const int32_t img_width, const int32_t img_height) {

    // Create a DMA buffer
    NvBufferCreateParams params;
    memset(&params, 0, sizeof(params));
    params.width       = img_width;
    params.height      = img_height;
    params.layout      = NvBufferLayout_Pitch;
    params.colorFormat = NvBufferColorFormat_YUV420;
    params.payloadType = NvBufferPayload_SurfArray;
    params.nvbuf_tag   = NvBufferTag_NONE;

    int32_t dmabuf_fd = -1;
    if (NvBufferCreateEx(&dmabuf_fd, &params) != 0) {
        NvBufferDestroy(dmabuf_fd);
        raiseFatal("Failed to create DMA buffer of size: %dx%d!", img_width, img_height);
    }

    return dmabuf_fd;
}


/* @brief: Publishers initialization for the Imx 477 camera driver  */
void CameraFeedWrapper::initialiseRosBindings(const std::string &frame_id, const std::string &ros_topic_root_name) {
    // Topics initializations
    const std::string topic_clahe_224             = ros_topic_root_name + "/ml_clahe/compressed";
    const std::string topic_clahe_h264_2K         = ros_topic_root_name + "/clahe_h264_2K";
    const std::string topic_raw_h264_HD           = ros_topic_root_name + "/raw_h264_HD";
    const std::string topic_clahe_h264_HD         = ros_topic_root_name + "/clahe_h264_HD";
    const std::string topic_snapshot_service_name = ros_topic_root_name + "/get_snapshot";

    // Initialise the publishers
    INIT_ROS_PUBLISHER(pub_clahe_2k_h264, Msg_ImageH264Feed, topic_clahe_h264_2K, 10);
    INIT_ROS_PUBLISHER(pub_raw_hd_h264, Msg_ImageH264Feed, topic_raw_h264_HD, 10);
    INIT_ROS_PUBLISHER(pub_clahe_hd_h264, Msg_ImageH264Feed, topic_clahe_h264_HD, 10);

    INIT_ROS_PUBLISHER(pub_clahe_224_jpeg, Msg_CompressedImage, topic_clahe_224, 1);

    INIT_ROS_SUBSCRIBER(sub_camera_depth, VEHICLE_ODOMETRY_TOPIC, 1, &CameraFeedWrapper::depthCallback);

    // Initialise the services
    INIT_ROS_SERVICE_SERVER(
            srv_get_snapshot_server,
            topic_snapshot_service_name,
            &CameraFeedWrapper::getImgSrvCallback);

    // Declare the message headers
    clahe_224_jpeg_msg.header.frame_id = frame_id;
    clahe_224_jpeg_msg.format          = "jpeg";
}


// -----------------------------------------------------
// NvBuffer Helper functions
// -----------------------------------------------------

bool CameraFeedWrapper::nvBuffer2CvMat(cv::Mat &out_yuv_image, const int32_t src_dmabuf_fd) {

    // 1. Validate fd
    if (src_dmabuf_fd < 0) {
        LOG_INFO("Invalid DMA buffer fd: %d", src_dmabuf_fd);
        return false;
    }

    // 2. Get NvBufSurface from fd (NEW API - this is the key difference!)
    NvBufSurface *nvbuf_surf = nullptr;
    int ret                  = NvBufSurfaceFromFd(src_dmabuf_fd, (void **)(&nvbuf_surf));
    if (ret != 0 || nvbuf_surf == nullptr) {
        LOG_INFO("Failed to get NvBufSurface from fd %d", src_dmabuf_fd);
        return false;
    }

    // 3. Get dimensions from surface
    const int32_t img_w                        = nvbuf_surf->surfaceList[0].width;
    const int32_t img_h                        = nvbuf_surf->surfaceList[0].height;
    const uint32_t num_planes                  = nvbuf_surf->surfaceList[0].planeParams.num_planes;
    const NvBufSurfaceColorFormat color_format = nvbuf_surf->surfaceList[0].colorFormat;

    if (img_w == 0 || img_h == 0) {
        LOG_INFO("Invalid buffer dimensions: %dx%d", img_w, img_h);
        return false;
    }

    // 4. Create/resize output Mat for YUV (height * 1.5)
    const int32_t mat_height = (img_h * 3) / 2;
    if (out_yuv_image.empty() || out_yuv_image.rows != mat_height || out_yuv_image.cols != img_w) {
        out_yuv_image.create(mat_height, img_w, CV_8UC1);
    }

    uint8_t *yuv_data = out_yuv_image.data;
    size_t offset     = 0;

    // 5. Handle based on format (NV12 = 2 planes, YUV420 = 3 planes)
    if (num_planes == 2) {
        // NV12 format: Y plane + interleaved UV plane

        // Copy Y plane (plane 0)
        ret = NvBufSurfaceMap(nvbuf_surf, 0, 0, NVBUF_MAP_READ);
        if (ret != 0) {
            LOG_INFO("Failed to map Y plane");
            return false;
        }
        NvBufSurfaceSyncForCpu(nvbuf_surf, 0, 0);  // IMPORTANT: sync before CPU access!

        const uint32_t y_pitch = nvbuf_surf->surfaceList[0].planeParams.pitch[0];
        uint8_t *y_src         = (uint8_t *)nvbuf_surf->surfaceList[0].mappedAddr.addr[0];

        if (y_src == nullptr) {
            LOG_INFO("Y plane mapped address is null");
            NvBufSurfaceUnMap(nvbuf_surf, 0, 0);
            return false;
        }

        for (int32_t row = 0; row < img_h; row++) {
            memcpy(yuv_data + offset, y_src + row * y_pitch, img_w);
            offset += img_w;
        }
        NvBufSurfaceUnMap(nvbuf_surf, 0, 0);

        // Copy UV plane (plane 1) - interleaved, half height
        ret = NvBufSurfaceMap(nvbuf_surf, 0, 1, NVBUF_MAP_READ);
        if (ret != 0) {
            LOG_INFO("Failed to map UV plane");
            return false;
        }
        NvBufSurfaceSyncForCpu(nvbuf_surf, 0, 1);

        const uint32_t uv_pitch = nvbuf_surf->surfaceList[0].planeParams.pitch[1];
        const int32_t uv_height = img_h / 2;
        uint8_t *uv_src         = (uint8_t *)nvbuf_surf->surfaceList[0].mappedAddr.addr[1];

        if (uv_src == nullptr) {
            LOG_INFO("UV plane mapped address is null");
            NvBufSurfaceUnMap(nvbuf_surf, 0, 1);
            return false;
        }

        for (int32_t row = 0; row < uv_height; row++) {
            memcpy(yuv_data + offset, uv_src + row * uv_pitch, img_w);
            offset += img_w;
        }
        NvBufSurfaceUnMap(nvbuf_surf, 0, 1);

    } else if (num_planes == 3) {
        // YUV420 planar format: Y, U, V separate planes
        static const int plane_divs[] = {1, 2, 2};

        for (int plane_idx = 0; plane_idx < 3; plane_idx++) {
            ret = NvBufSurfaceMap(nvbuf_surf, 0, plane_idx, NVBUF_MAP_READ);
            if (ret != 0) {
                LOG_INFO("Failed to map plane %d", plane_idx);
                return false;
            }
            NvBufSurfaceSyncForCpu(nvbuf_surf, 0, plane_idx);

            const int32_t plane_h      = img_h / plane_divs[plane_idx];
            const int32_t plane_w      = img_w / plane_divs[plane_idx];
            const uint32_t plane_pitch = nvbuf_surf->surfaceList[0].planeParams.pitch[plane_idx];
            uint8_t *src               = (uint8_t *)nvbuf_surf->surfaceList[0].mappedAddr.addr[plane_idx];

            if (src == nullptr) {
                LOG_INFO("Plane %d mapped address is null", plane_idx);
                NvBufSurfaceUnMap(nvbuf_surf, 0, plane_idx);
                return false;
            }

            for (int32_t row = 0; row < plane_h; row++) {
                memcpy(yuv_data + offset, src + row * plane_pitch, plane_w);
                offset += plane_w;
            }
            NvBufSurfaceUnMap(nvbuf_surf, 0, plane_idx);
        }
    } else {
        LOG_INFO("Unsupported plane count: %d", num_planes);
        return false;
    }

    return true;
}



/* @brief Copies YUV image data to an NVIDIA buffer using direct memory access (DMA)
 *        Processes the image in three planes (Y, U, and V), performing memory-mapped copies
 *        with proper stride handling for each plane. Ensures thread-safe buffer access and
 *        proper memory unmapping.
 * @param yuv_image: OpenCV Mat containing YUV420 format image data
 * @param dmabuf_fd: File descriptor for the target DMA buffer
 */
bool CameraFeedWrapper::CvMat2NvBuffer(const cv::Mat &yuv_image, const int32_t dst_dmabuf_fd) {

    if (yuv_image.empty() || dst_dmabuf_fd < 0) {
        return false;
    }

    // Get NvBufSurface from fd (NEW API)
    NvBufSurface *nvbuf_surf = nullptr;
    int ret                  = NvBufSurfaceFromFd(dst_dmabuf_fd, (void **)(&nvbuf_surf));
    if (ret != 0 || nvbuf_surf == nullptr) {
        LOG_INFO("CvMat2NvBuffer: Failed to get NvBufSurface from fd %d", dst_dmabuf_fd);
        return false;
    }

    const int32_t img_width   = yuv_image.cols;
    const int32_t img_height  = (yuv_image.rows * 2) / 3;
    const uint32_t num_planes = nvbuf_surf->surfaceList[0].planeParams.num_planes;

    uint8_t *yuv_data = yuv_image.data;
    size_t src_offset = 0;

    if (num_planes == 2) {
        // NV12 format: Y plane + interleaved UV plane

        // Write Y plane
        ret = NvBufSurfaceMap(nvbuf_surf, 0, 0, NVBUF_MAP_WRITE);
        if (ret != 0) {
            LOG_INFO("Failed to map Y plane for write");
            return false;
        }

        const uint32_t y_pitch = nvbuf_surf->surfaceList[0].planeParams.pitch[0];
        uint8_t *y_dst         = (uint8_t *)nvbuf_surf->surfaceList[0].mappedAddr.addr[0];

        for (int32_t row = 0; row < img_height; row++) {
            memcpy(y_dst + row * y_pitch, yuv_data + src_offset, img_width);
            src_offset += img_width;
        }

        NvBufSurfaceSyncForDevice(nvbuf_surf, 0, 0);
        NvBufSurfaceUnMap(nvbuf_surf, 0, 0);

        // Write UV plane
        ret = NvBufSurfaceMap(nvbuf_surf, 0, 1, NVBUF_MAP_WRITE);
        if (ret != 0) {
            LOG_INFO("Failed to map UV plane for write");
            return false;
        }

        const uint32_t uv_pitch = nvbuf_surf->surfaceList[0].planeParams.pitch[1];
        const int32_t uv_height = img_height / 2;
        uint8_t *uv_dst         = (uint8_t *)nvbuf_surf->surfaceList[0].mappedAddr.addr[1];

        for (int32_t row = 0; row < uv_height; row++) {
            memcpy(uv_dst + row * uv_pitch, yuv_data + src_offset, img_width);
            src_offset += img_width;
        }

        NvBufSurfaceSyncForDevice(nvbuf_surf, 0, 1);
        NvBufSurfaceUnMap(nvbuf_surf, 0, 1);

    } else if (num_planes == 3) {
        // YUV420 planar: Y, U, V separate
        static const int plane_divs[] = {1, 2, 2};

        for (int plane_idx = 0; plane_idx < 3; plane_idx++) {
            ret = NvBufSurfaceMap(nvbuf_surf, 0, plane_idx, NVBUF_MAP_WRITE);
            if (ret != 0) {
                LOG_INFO("Failed to map plane %d for write", plane_idx);
                return false;
            }

            const int32_t plane_width  = img_width / plane_divs[plane_idx];
            const int32_t plane_height = img_height / plane_divs[plane_idx];
            const uint32_t plane_pitch = nvbuf_surf->surfaceList[0].planeParams.pitch[plane_idx];
            uint8_t *dst               = (uint8_t *)nvbuf_surf->surfaceList[0].mappedAddr.addr[plane_idx];

            for (int row = 0; row < plane_height; row++) {
                memcpy(dst + row * plane_pitch, yuv_data + src_offset, plane_width);
                src_offset += plane_width;
            }

            NvBufSurfaceSyncForDevice(nvbuf_surf, 0, plane_idx);
            NvBufSurfaceUnMap(nvbuf_surf, 0, plane_idx);
        }
    } else {
        LOG_INFO("CvMat2NvBuffer: Unsupported plane count: %d", num_planes);
        return false;
    }

    return true;
}

bool CameraFeedWrapper::resizeFrame(
        const int32_t srcDmaBuf,
        const int32_t dstDmaBuf,
        const int32_t srcWidth,
        const int32_t srcHeight,
        const int32_t dstWidth,
        const int32_t dstHeight) {

    NvBufferTransformParams transform_resize_params = {0};
    transform_resize_params.src_rect.top            = 0;
    transform_resize_params.src_rect.left           = 0;
    transform_resize_params.src_rect.width          = srcWidth;
    transform_resize_params.src_rect.height         = srcHeight;
    transform_resize_params.dst_rect.top            = 0;
    transform_resize_params.dst_rect.left           = 0;
    transform_resize_params.dst_rect.width          = dstWidth;
    transform_resize_params.dst_rect.height         = dstHeight;
    transform_resize_params.transform_flag          = NVBUFFER_TRANSFORM_FILTER;
    transform_resize_params.transform_flip          = NvBufferTransform_None;
    transform_resize_params.transform_filter        = NvBufferTransform_Filter_Nicest;

    return (NvBufferTransform(srcDmaBuf, dstDmaBuf, &transform_resize_params) == 0);
}

void CameraFeedWrapper::NvBuffer2Jpeg(const int src_dmabuf_fd, std::vector<uint8_t> &jpeg_buffer) {

    // Get buffer parameters
    NvBufferParams dst_params;

    if (!jpeg_encoder || NvBufferGetParams(src_dmabuf_fd, &dst_params) < 0) {
        nvBuffer2CvMat(jpeg_yuv_image, src_dmabuf_fd);
        cv::cvtColor(jpeg_yuv_image, jpeg_bgr_image, cv::COLOR_YUV2BGR_I420);

        jpeg_buffer.clear();
        cv::imencode(".jpeg", jpeg_bgr_image, jpeg_buffer);
        return;
    }

    const int32_t img_h       = dst_params.height[0];
    const int32_t img_w       = dst_params.width[0];
    unsigned long out_bufSize = img_h * img_w * 3 / 2;

    jpeg_buffer.resize(out_bufSize);
    uint8_t *buffer_ptr = jpeg_buffer.data();

    jpeg_encoder->encodeFromFd(src_dmabuf_fd, JCS_YCbCr, &buffer_ptr, out_bufSize, 80);
    jpeg_buffer.resize(out_bufSize);
    return;
}



// -----------------------------------------------------
// ROS Callbacks
// -----------------------------------------------------


void CameraFeedWrapper::depthCallback(const Msg_VehicleState::ConstPtr &msg) {
    camera_depth = std::max(0.0, msg->pose_ned.position.z);
}

/* @brief Service callback to snapshot the Image feed */
bool CameraFeedWrapper::getImgSrvCallback(Srv_CamGetSnapshot_Request &req, Srv_CamGetSnapshot_Response &res) {

    Srv_CamGetSnapshot_Response *res_ptr = &res;
    captureAndPublishOnce(res_ptr);

    return true;
}
