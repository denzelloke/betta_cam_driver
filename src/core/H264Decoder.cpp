#include <h264_camera_driver/core/H264Decoder.h>


#define TEST_ERROR(condition, message)                                                                                 \
    if (condition) {                                                                                                   \
        LOG_ERROR(message);                                                                                            \
    }

#define DEC_CHUNK_SIZE 4'000'000

H264Decoder::H264Decoder() : has_first_frame_(false) { nvmpi_create_decoder(); }

H264Decoder::~H264Decoder() { nvmpi_decoder_close(); }

bool H264Decoder::decoder_put_packet(const Msg_ImageH264Feed_ConstPtr &msg) {
    if (msg->data.size() < 5) {
        return false;
    }

    uint8_t *buffer_data   = const_cast<uint8_t *>(msg->data.data());
    const uint8_t nal_type = buffer_data[4] & 0x1F;
    const bool is_keyframe = (nal_type != 2 && nal_type != 7);

    if (!has_first_frame_) {
        printf("Waiting for first I frame\n");

        if (is_keyframe) {
            return false;
        }

        has_first_frame_ = true;
    }

    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer *nvBuffer = nullptr;

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, sizeof(planes));

    v4l2_buf.m.planes = planes;

    if (buffer_index_ < static_cast<int32_t>(dec_->output_plane.getNumBuffers())) {
        nvBuffer          = dec_->output_plane.getNthBuffer(buffer_index_);
        v4l2_buf.index    = buffer_index_;
        v4l2_buf.m.planes = planes;

    }
    // 	[out] &nvBUffer - a pointer to a pointer to the NvBuffer object associated with the dequeued buffer
    else if (dec_->output_plane.dqBuffer(v4l2_buf, &nvBuffer, NULL, -1) < 0) {
        printf("Error DQing buffer at output plane\n");
        return false;
    }

    // -- safety net cuase it keeps seg faulting --
    if (!nvBuffer) {
        printf("Error: Retrieved nvBuffer is NULL\n");
        return false;
    }

    if (nvBuffer->planes[0].data == nullptr) {
        printf("Error: nvBuffer->planes[0].data is NULL! (Buffer Index: %d)\n", buffer_index_);
        // If we can't write to the buffer, we can't decode. Return false to skip.
        return false;
    }

    if (msg->data.size() > DEC_CHUNK_SIZE) {
        printf("Error: Incoming packet size (%lu) exceeds decoder buffer size (%d)\n",
               msg->data.size(),
               DEC_CHUNK_SIZE);
        return false;
    }
    // -- end of safety net --

    memcpy(nvBuffer->planes[0].data, buffer_data, msg->data.size());
    nvBuffer->planes[0].bytesused = msg->data.size();

    v4l2_buf.timestamp.tv_sec      = msg->unix_time_ms / 1'000;
    v4l2_buf.timestamp.tv_usec     = msg->unix_time_ms % 1'000;
    v4l2_buf.m.planes[0].bytesused = nvBuffer->planes[0].bytesused;
    v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;

    if (dec_->output_plane.qBuffer(v4l2_buf, NULL) < 0) {
        printf("Error Qing buffer at output plane\n");
        return false;
    }

    buffer_index_++;
    return true;
}

bool H264Decoder::decoder_get_frame(NvBufSurface *out_surf) {
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    if (frame_pools_.empty()) {
        return false;
    }

    const int picture_index = frame_pools_.front();
    frame_pools_.pop();

    NvBufSurfTransform_Error error =
            NvBufSurfTransform(intermediate_surf_vec_[picture_index], out_surf, &transform_params_);

    if (error != NvBufSurfTransformError_Success) {
        LOG_ERROR("NvBufSurfTransform FAILED IN DECODER_GET_FRAME\npicture index = %d\n", picture_index);
        return false;
    }

    return true;
}

/**
 * @details
 * [detects resolution of incoming h264 stream]
 * getFormat, getCrop --> coded_height_, coded_width_
 *
 * [cleanup old resources]
 * deinitPlane and NvBufSurfaceDestroy(surf) loop
 *
 * [re-config decoder]
 * setCapturePlaneFormat - reconfigures the format (YUV420 HD/2K) in which the h264stream will be
 * decoded and output
 *
 * [allocate new memory]
 * NvBufSurfaceAllocateParams - define what type of buffers the decoder needs
 * dec->capture_plane.getNumBuffers() - calculates how many intermediate buffers the decoder needs
 * NvBufSurfaceAllocate loop - allocates those buffers
 * intermediate_surf_vec_.push_back(new_surf) - populate intermediate_surf_vec_ with the intermediate
 * buffers holding frames
 *
 * [queue buffers]
 * qBuffer - assigns each frame to a new surface by shoving the FD from each new surface into the V4L2 driver
 *
 * [setup transformation]
 * prepares for decoder_get_frame which will do the actual transformation of the intermediate buffers
 */
void H264Decoder::respondToResolutionEvent() {
    printf("RESPOND TO RESOLUTION EVENT CALLED!\n");
    struct v4l2_format v4l2Format;
    int32_t ret = dec_->capture_plane.getFormat(v4l2Format);
    TEST_ERROR(ret < 0, "Error: Could not get format from decoder capture plane");

    struct v4l2_crop v4l2Crop;
    ret = dec_->capture_plane.getCrop(v4l2Crop);
    TEST_ERROR(ret < 0, "Error: Could not get crop from decoder capture plane");

    coded_width_  = v4l2Crop.c.width;
    coded_height_ = v4l2Crop.c.height;

    dec_->capture_plane.deinitPlane();

    // cleanup old surfaces using NvBufSurfaceDestroy
    for (NvBufSurface *surf_to_destroy : intermediate_surf_vec_) {
        if (surf_to_destroy) {
            NvBufSurfaceDestroy(surf_to_destroy);
        }
    }
    intermediate_surf_vec_.clear();

    ret = dec_->setCapturePlaneFormat(
            v4l2Format.fmt.pix_mp.pixelformat,
            v4l2Format.fmt.pix_mp.width,
            v4l2Format.fmt.pix_mp.height);
    TEST_ERROR(ret < 0, "Error in setting decoder capture plane format");

    int32_t minimumDecoderCaptureBuffers;
    dec_->getMinimumCapturePlaneBuffers(minimumDecoderCaptureBuffers);
    TEST_ERROR(ret < 0, "Error while getting value of minimum capture plane buffers");


    // If Standard, Decoder colorspace ITU-R BT.601 with standard range luma (16-235)
    // Otherwise, Decoder colorspace ITU-R BT.601 with extended range luma (0-255)
    NvBufSurfaceAllocateParams allocParams;
    memset(&allocParams, 0, sizeof(allocParams));

    allocParams.params.width   = coded_width_;
    allocParams.params.height  = coded_height_;
    allocParams.params.layout  = NVBUF_LAYOUT_BLOCK_LINEAR;
    allocParams.params.memType = NVBUF_MEM_SURFACE_ARRAY;
    allocParams.memtag         = NvBufSurfaceTag_VIDEO_DEC;

    const bool isQuantDefault = (v4l2Format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT);
    switch (v4l2Format.fmt.pix_mp.colorspace) {
    case V4L2_COLORSPACE_SMPTE170M:
        allocParams.params.colorFormat = isQuantDefault ? NVBUF_COLOR_FORMAT_NV12 : NVBUF_COLOR_FORMAT_NV12_ER;
        break;

    case V4L2_COLORSPACE_REC709:
        allocParams.params.colorFormat = isQuantDefault ? NVBUF_COLOR_FORMAT_NV12_709 : NVBUF_COLOR_FORMAT_NV12_709_ER;
        break;

    case V4L2_COLORSPACE_BT2020:
        //"Decoder colorspace ITU-R BT.2020";
        allocParams.params.colorFormat = NVBUF_COLOR_FORMAT_NV12_2020;
        break;

    default:
        allocParams.params.colorFormat = isQuantDefault ? NVBUF_COLOR_FORMAT_NV12 : NVBUF_COLOR_FORMAT_NV12_ER;
        break;
    }

    const int32_t numberCaptureBuffers = 5 + minimumDecoderCaptureBuffers;
    dec_->capture_plane.reqbufs(V4L2_MEMORY_DMABUF, numberCaptureBuffers);
    TEST_ERROR(ret < 0, "Error in decoder capture plane streamon");

    dec_->capture_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in decoder capture plane streamon");


    for (uint32_t idx = 0; idx < dec_->capture_plane.getNumBuffers(); idx++) {
        NvBufSurface *new_surf = nullptr;
        int success            = NvBufSurfaceAllocate(&new_surf, 1, &allocParams);
        TEST_ERROR(success < 0, "Failed to create NvBufSurface");

        intermediate_surf_vec_.push_back(new_surf);

        // queue buffer to decoder (V4L2)
        // decoder (v4l2 driver) still needs integer fd to know where to write
        // so we extract the FD from new_surf
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.index    = idx;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory   = V4L2_MEMORY_DMABUF;

        // extract fd from new_surf and shove it into the decoder
        v4l2_buf.m.planes[0].m.fd = (int)new_surf->surfaceList[0].bufferDesc;

        ret = dec_->capture_plane.qBuffer(v4l2_buf, NULL);
        TEST_ERROR(ret < 0, "Error Qing buffer at output plane");
    }

    // Update the transform parameters
    memset(&transform_params_, 0, sizeof(transform_params_));

    // create source and destination rectangles
    src_rect_params_.top    = 0;
    src_rect_params_.left   = 0;
    src_rect_params_.width  = coded_width_;
    src_rect_params_.height = coded_height_;

    dst_rect_params_.top    = 0;
    dst_rect_params_.left   = 0;
    dst_rect_params_.width  = coded_width_;
    dst_rect_params_.height = coded_height_;

    // assign transform_params_ rectangle pointers
    transform_params_.src_rect = &src_rect_params_;
    transform_params_.dst_rect = &dst_rect_params_;

    // set transform_params flags
    transform_params_.transform_flag   = NVBUFSURF_TRANSFORM_FILTER;
    transform_params_.transform_flip   = NvBufSurfTransform_None;
    transform_params_.transform_filter = NvBufSurfTransformInter_Algo3;
}


void H264Decoder::dec_capture_loop_fcn() {

    // initialise
    struct v4l2_event v4l2Event;
    NvBuffer *dec_buffer;
    RateLimiter rate_limiter(15);
    bool got_res_event_ = false;

    while (!eos_ && !(dec_->isInError())) {

        // polls dqEvent for v4l2 driver events
        int32_t ret = dec_->dqEvent(v4l2Event, got_res_event_ ? 0 : 500);
        if (ret == 0 && v4l2Event.type == V4L2_EVENT_RESOLUTION_CHANGE) {
            // allocate new intermediate buffers for this resolution stream
            respondToResolutionEvent();
            got_res_event_ = true;
        }

        // if yet to allocate new intermediate buffers, loop and wait.
        // do not start processing frames until we know the resolution
        if (!got_res_event_) {
            continue;
        }

        // process frames
        while (!eos_) {
            rate_limiter.sleep();

            struct v4l2_buffer v4l2_buf;
            struct v4l2_plane planes[MAX_PLANES];
            v4l2_buf.m.planes = planes;

            // dequeue - assign intermediate buffer to decoder's capture plane function
            // fills v4l2_buf with metadetails of the captured frames - like index/size/flags,
            // dec_buffer q useless. if im not wrong its just legacy, but dec_ needs it soooooo
            if (dec_->capture_plane.dqBuffer(v4l2_buf, &dec_buffer, NULL, 0) != 0) {
                printf("DQBuffer failed\n");  // probably cos no frame is ready yet (normal)
                usleep(10'000);
                break;
            }

            // retrieve surface corresponding to this index (intermediate_surf_vec_ is a vector of surfaces)
            NvBufSurface *intermediate_surface_ = intermediate_surf_vec_[v4l2_buf.index];

            // extract FD from this
            int intermediate_fd = (int)intermediate_surface_->surfaceList[0].bufferDesc;

            // update the NvBuffer wrapper helper?? honestly dk
            dec_buffer->planes[0].fd = intermediate_fd;

            // handoff to application
            // queues v4l2_buf.index. if more than 5 frames in frame_pool_,
            // dequeue the old ones (drop frames) to prevent lag
            {
                std::unique_lock<std::mutex> lock(buffer_mutex_);
                frame_pools_.push(v4l2_buf.index);
                while (frame_pools_.size() >= 5) {
                    frame_pools_.pop();
                }
            }

            // frame is done processing, so this particular v4l2_buf.m.planes[0].m.fd can be overwritten
            v4l2_buf.m.planes[0].m.fd = intermediate_fd;

            // overwrite v4l2_buf
            if (dec_->capture_plane.qBuffer(v4l2_buf, NULL) != 0) {
                ERROR_MSG("Error while queueing buffer at decoder capture plane");
            }
        }
    }
}

void H264Decoder::nvmpi_create_decoder() {

    dec_ = NvVideoDecoder::createVideoDecoder("dec0");
    TEST_ERROR(!dec_, "Could not create decoder");

    int32_t ret = dec_->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0);
    TEST_ERROR(ret < 0, "Could not subscribe to V4L2_EVENT_RESOLUTION_CHANGE");

    ret = dec_->setOutputPlaneFormat(V4L2_PIX_FMT_H264, DEC_CHUNK_SIZE);
    TEST_ERROR(ret < 0, "Could not set output plane format");

    ret = dec_->setFrameInputMode(0);
    TEST_ERROR(ret < 0, "Error in decoder setFrameInputMode for NALU");

    // MIGRATION NOTES:
    // V4L2_MEMORY_USERPTR --> V4L2_MEMORY_MMAP
    // rationale:
    // V4L2_MEMORY_USERPTR -> application allocates memory buffers (using new / malloc) and pass the ptrs to the encoder
    // V4L2_MEMORY_MMAP -> encoder driver allocates the buffers itself within hardware.
    // you js ask for a pointer to map it to copy ur data into
    ret = dec_->output_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true, false);
    // ret = dec_->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 10, false, true);
    TEST_ERROR(ret < 0, "Error while setting up output plane");

    dec_->output_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in output plane stream on");

    eos_          = false;
    buffer_index_ = 0;

    while (!frame_pools_.empty()) {
        frame_pools_.pop();
    }

    for (NvBufSurface *surf_to_destroy : intermediate_surf_vec_) {
        if (surf_to_destroy) {
            NvBufSurfaceDestroy(surf_to_destroy);
        }
    }
    intermediate_surf_vec_.clear();

    if (dec_capture_loop_ == nullptr) {
        dec_capture_loop_ = new std::thread(&H264Decoder::dec_capture_loop_fcn, this);
    }
}

void H264Decoder::nvmpi_decoder_close() {

    {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        eos_ = true;
    }

    dec_->capture_plane.setStreamStatus(false);

    if (dec_capture_loop_) {
        dec_capture_loop_->join();
        delete dec_capture_loop_;
        dec_capture_loop_ = nullptr;
    }

    for (NvBufSurface *surf_to_destroy : intermediate_surf_vec_) {
        if (surf_to_destroy) {
            NvBufSurfaceDestroy(surf_to_destroy);
        }
    }
    intermediate_surf_vec_.clear();

    delete dec_;
    dec_ = nullptr;
}
