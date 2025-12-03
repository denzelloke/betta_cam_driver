// Copyright by BeeX [2024]
#include <h264_camera_driver/core/H264Decoder.h>

#define TEST_ERROR(condition, message)                                                                                 \
    if (condition) {                                                                                                   \
        LOG_ERROR(message);                                                                                            \
    }

#define DEC_CHUNK_SIZE 4'000'000

H264Decoder::H264Decoder() : has_first_frame(false) { nvmpi_create_decoder(); }

H264Decoder::~H264Decoder() { nvmpi_decoder_close(); }

bool H264Decoder::decoder_put_packet(const Msg_ImageH264Feed_ConstPtr &msg) {
    if (msg->data.size() < 5) {
        return false;
    }

    uint8_t *buffer_data   = const_cast<uint8_t *>(msg->data.data());
    const uint8_t nal_type = buffer_data[4] & 0x1F;

    // Allow IDR (5), SPS (7), or PPS (8) to start the sequence
    const bool is_idr = (nal_type == 5);
    const bool is_sps = (nal_type == 7);
    const bool is_pps = (nal_type == 8);

    if (!has_first_frame) {
        if (is_idr || is_sps || is_pps) {
            printf("Found keyframe/config (Type %d)! Starting decode...\n", nal_type);
            has_first_frame = true;
        } else {
            static int log_counter = 0;
            if (log_counter++ % 30 == 0) {
                printf("nal_type: %d Waiting for first I frame (IDR/SPS/PPS)...\n", nal_type);
            }
            return false;
        }
    }

    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer *nvBuffer;

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, sizeof(planes));

    v4l2_buf.m.planes = planes;

    if (buffer_index < static_cast<int32_t>(dec->output_plane.getNumBuffers())) {
        nvBuffer          = dec->output_plane.getNthBuffer(buffer_index);
        v4l2_buf.index    = buffer_index;
        v4l2_buf.m.planes = planes;

    } else if (dec->output_plane.dqBuffer(v4l2_buf, &nvBuffer, NULL, -1) < 0) {
        printf("Error DQing buffer at output plane\n");
        return false;
    }

    memcpy(nvBuffer->planes[0].data, buffer_data, msg->data.size());
    nvBuffer->planes[0].bytesused = msg->data.size();

    v4l2_buf.timestamp.tv_sec      = msg->unix_time_ms / 1'000;
    v4l2_buf.timestamp.tv_usec     = msg->unix_time_ms % 1'000;
    v4l2_buf.m.planes[0].bytesused = nvBuffer->planes[0].bytesused;
    v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;

    if (dec->output_plane.qBuffer(v4l2_buf, NULL) < 0) {
        printf("Error Qing buffer at output plane\n");
        return false;
    }

    buffer_index++;
    return true;
}

// MIGRATION: Takes NvBufSurface* as destination
bool H264Decoder::decoder_get_frame(NvBufSurface *out_dma_buf_2K) {

    std::unique_lock<std::mutex> lock(buffer_mutex);
    if (frame_pools.empty()) {
        return false;
    }

    const int picture_index = frame_pools.front();
    frame_pools.pop();

    // Ensure index is valid within our surfaces bounds
    if (picture_index >= (int)output_surfaces.size()) {
        LOG_ERROR("Frame pool index out of bounds");
        return false;
    }

    // MIGRATION: Use NvBufSurfTransform instead of NvBufferTransform
    // Use the stored source surface from output_surfaces
    int32_t ret = NvBufSurfTransform(output_surfaces[picture_index], out_dma_buf_2K, &transform_params);

    // FIX: Return the buffer to the decoder NOW, after we are done reading/transforming it.
    // This allows the decoder to reuse this buffer for future frames.
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, sizeof(planes));

    v4l2_buf.index            = picture_index;
    v4l2_buf.m.planes         = planes;
    v4l2_buf.type             = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_buf.memory           = V4L2_MEMORY_DMABUF;
    v4l2_buf.m.planes[0].m.fd = (int)output_surfaces[picture_index]->surfaceList[0].bufferDesc;

    if (dec->capture_plane.qBuffer(v4l2_buf, NULL) < 0) {
        LOG_ERROR("Error returning buffer to decoder in decoder_get_frame");
    }

    TEST_ERROR(ret != NvBufSurfTransformError_Success, "Transform failed");

    return true;
}

void H264Decoder::respondToResolutionEvent() {
    // FIX: Lock the mutex to prevent race with decoder_get_frame
    std::lock_guard<std::mutex> lock(buffer_mutex);

    struct v4l2_format v4l2Format;
    int32_t ret = dec->capture_plane.getFormat(v4l2Format);
    TEST_ERROR(ret < 0, "Error: Could not get format from decoder capture plane");

    struct v4l2_crop v4l2Crop;
    ret = dec->capture_plane.getCrop(v4l2Crop);
    TEST_ERROR(ret < 0, "Error: Could not get crop from decoder capture plane");

    coded_width  = v4l2Crop.c.width;
    coded_height = v4l2Crop.c.height;

    dec->capture_plane.deinitPlane();

    // FIX: Clear stale frames from the pool.
    // Previous indices refer to destroyed surfaces or old session state.
    while (!frame_pools.empty()) {
        frame_pools.pop();
    }

    // MIGRATION: Destroy existing surfaces
    for (auto *surf : output_surfaces) {
        NvBufSurfaceDestroy(surf);
    }
    output_surfaces.clear();

    ret = dec->setCapturePlaneFormat(
            v4l2Format.fmt.pix_mp.pixelformat,
            v4l2Format.fmt.pix_mp.width,
            v4l2Format.fmt.pix_mp.height);
    TEST_ERROR(ret < 0, "Error in setting decoder capture plane format");

    int32_t minimumDecoderCaptureBuffers;
    dec->getMinimumCapturePlaneBuffers(minimumDecoderCaptureBuffers);
    TEST_ERROR(ret < 0, "Error while getting value of minimum capture plane buffers");

    // MIGRATION: Map V4L2 format to NvBufSurfaceColorFormat
    NvBufSurfaceAllocateParams allocParams = {0};
    allocParams.params.width               = coded_width;
    allocParams.params.height              = coded_height;
    allocParams.params.layout              = NVBUF_LAYOUT_BLOCK_LINEAR;
    allocParams.params.memType             = NVBUF_MEM_SURFACE_ARRAY;
    allocParams.memtag                     = NvBufSurfaceTag_VIDEO_DEC;  // Use VIDEO_DEC tag if possible, else NONE

    const bool isQuantDefault = (v4l2Format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT);
    switch (v4l2Format.fmt.pix_mp.colorspace) {
    case V4L2_COLORSPACE_SMPTE170M:
        allocParams.params.colorFormat = isQuantDefault ? NVBUF_COLOR_FORMAT_NV12 : NVBUF_COLOR_FORMAT_NV12_ER;
        break;
    case V4L2_COLORSPACE_REC709:
        allocParams.params.colorFormat = isQuantDefault ? NVBUF_COLOR_FORMAT_NV12_709 : NVBUF_COLOR_FORMAT_NV12_709_ER;
        break;
    case V4L2_COLORSPACE_BT2020:
        allocParams.params.colorFormat = NVBUF_COLOR_FORMAT_NV12_2020;
        break;
    default:
        allocParams.params.colorFormat = isQuantDefault ? NVBUF_COLOR_FORMAT_NV12 : NVBUF_COLOR_FORMAT_NV12_ER;
        break;
    }

    const int32_t numberCaptureBuffers = 5 + minimumDecoderCaptureBuffers;
    dec->capture_plane.reqbufs(V4L2_MEMORY_DMABUF, numberCaptureBuffers);
    TEST_ERROR(ret < 0, "Error in decoder capture plane streamon");

    dec->capture_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in decoder capture plane streamon");

    for (uint32_t idx = 0; idx < dec->capture_plane.getNumBuffers(); idx++) {

        // MIGRATION: Allocate NvBufSurface
        NvBufSurface *new_surf = nullptr;
        ret                    = NvBufSurfaceAllocate(&new_surf, 1, &allocParams);
        TEST_ERROR(ret < 0, "Failed to create buffers");

        output_surfaces.push_back(new_surf);

        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.index    = idx;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory   = V4L2_MEMORY_DMABUF;

        // MIGRATION: Get FD from surface params
        if (new_surf) {
            v4l2_buf.m.planes[0].m.fd = (int)new_surf->surfaceList[0].bufferDesc;
        }

        ret = dec->capture_plane.qBuffer(v4l2_buf, NULL);
        TEST_ERROR(ret < 0, "Error Qing buffer at output plane");
    }

    // MIGRATION: Update the transform parameters
    memset(&transform_params, 0, sizeof(transform_params));

    // Store rects in class members to ensure pointers remain valid
    src_rect.top    = 0;
    src_rect.left   = 0;
    src_rect.width  = coded_width;
    src_rect.height = coded_height;

    dst_rect.top    = 0;
    dst_rect.left   = 0;
    dst_rect.width  = coded_width;
    dst_rect.height = coded_height;

    transform_params.transform_flag   = NVBUFSURF_TRANSFORM_FILTER;
    transform_params.transform_flip   = NvBufSurfTransform_None;
    transform_params.transform_filter = NvBufSurfTransformInter_Algo3;  // Smart
    transform_params.src_rect         = &src_rect;
    transform_params.dst_rect         = &dst_rect;
}

void H264Decoder::dec_capture_loop_fcn() {
    struct v4l2_event v4l2Event;
    NvBuffer *dec_buffer;

    RateLimiter rate_limiter(15);

    bool got_res_event = false;
    while (!eos && !(dec->isInError())) {

        // Non-blocking event check if we already have resolution, otherwise blocking (500ms)
        int32_t ret = dec->dqEvent(v4l2Event, got_res_event ? 0 : 500);
        if (ret == 0 && v4l2Event.type == V4L2_EVENT_RESOLUTION_CHANGE) {
            respondToResolutionEvent();
            got_res_event = true;
        }

        if (!got_res_event) {
            continue;
        }

        // FIX: Replaced inner loop with single pass to allow checking for events repeatedly
        // This ensures subsequent resolution changes or other events are not starved.

        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        v4l2_buf.m.planes = planes;

        // Poll for frame (10ms timeout)
        if (dec->capture_plane.dqBuffer(v4l2_buf, &dec_buffer, NULL, 10) == 0) {

            std::unique_lock<std::mutex> lock(buffer_mutex);

            // FIX: If the queue is full, we must drop the oldest frame AND return it to the decoder
            while (frame_pools.size() >= 5) {
                int dropped_idx = frame_pools.front();
                frame_pools.pop();

                // Return dropped buffer to decoder so it can keep working
                struct v4l2_buffer v4l2_buf_drop;
                struct v4l2_plane planes_drop[MAX_PLANES];
                memset(&v4l2_buf_drop, 0, sizeof(v4l2_buf_drop));
                memset(planes_drop, 0, sizeof(planes_drop));

                v4l2_buf_drop.index    = dropped_idx;
                v4l2_buf_drop.m.planes = planes_drop;
                v4l2_buf_drop.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                v4l2_buf_drop.memory   = V4L2_MEMORY_DMABUF;

                // Ensure the surface still exists (might have been cleared by resolution event logic)
                if (dropped_idx < (int)output_surfaces.size() && output_surfaces[dropped_idx]) {
                    v4l2_buf_drop.m.planes[0].m.fd = (int)output_surfaces[dropped_idx]->surfaceList[0].bufferDesc;
                    dec->capture_plane.qBuffer(v4l2_buf_drop, NULL);
                }
            }

            frame_pools.push(v4l2_buf.index);

            // FIX: DO NOT qBuffer the current frame here immediately.
            // We hold it until the consumer (decoder_get_frame) finishes with it.
        } else {
            // No frame ready, sleep briefly to avoid spinlock
            rate_limiter.sleep();
        }
    }
}

void H264Decoder::nvmpi_create_decoder() {

    dec = NvVideoDecoder::createVideoDecoder("dec0");
    TEST_ERROR(!dec, "Could not create decoder");

    int32_t ret = dec->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0);
    TEST_ERROR(ret < 0, "Could not subscribe to V4L2_EVENT_RESOLUTION_CHANGE");

    ret = dec->setOutputPlaneFormat(V4L2_PIX_FMT_H264, DEC_CHUNK_SIZE);
    TEST_ERROR(ret < 0, "Could not set output plane format");

    ret = dec->setFrameInputMode(0);
    TEST_ERROR(ret < 0, "Error in decoder setFrameInputMode for NALU");

    ret = dec->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 10, false, true);
    TEST_ERROR(ret < 0, "Error while setting up output plane");

    dec->output_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in output plane stream on");

    eos          = false;
    buffer_index = 0;

    while (!frame_pools.empty()) {
        frame_pools.pop();
    }

    // MIGRATION: Cleanup surfaces
    for (auto *surf : output_surfaces) {
        NvBufSurfaceDestroy(surf);
    }
    output_surfaces.clear();

    if (dec_capture_loop == nullptr) {
        dec_capture_loop = new std::thread(&H264Decoder::dec_capture_loop_fcn, this);
    }
}

void H264Decoder::nvmpi_decoder_close() {

    {
        std::unique_lock<std::mutex> lock(buffer_mutex);
        eos = true;
    }

    dec->capture_plane.setStreamStatus(false);

    if (dec_capture_loop) {
        dec_capture_loop->join();
        delete dec_capture_loop;
        dec_capture_loop = nullptr;
    }

    // MIGRATION: Cleanup surfaces
    for (auto *surf : output_surfaces) {
        NvBufSurfaceDestroy(surf);
    }
    output_surfaces.clear();

    delete dec;
    dec = nullptr;
}