// Copyright by BeeX [2024]
#include <h264_camera_driver/core/H264Encoder.h>
#include <iostream>


#define ENC_CHUNK_SIZE 2 * 1'024 * 1'024
#define TEST_ERROR(condition, message)                                                                                 \
    if (condition) {                                                                                                   \
        printf(message);                                                                                               \
    }


static bool encoder_capture_plane_dq_callback(
        struct v4l2_buffer *v4l2_buf,
        NvBuffer *buffer,
        NvBuffer *shared_buffer,
        void *arg) {

    printf("[ENCODER CALLBACK] Called!\n");

    if (v4l2_buf == NULL) {
        printf("[ENCODER CALLBACK] v4l2_buf is NULL!\n");
        return false;
    }

    const int32_t bytesused = buffer->planes[0].bytesused;
    printf("[ENCODER CALLBACK] bytesused=%d\n", bytesused);

    if (bytesused == 0) {
        printf("[ENCODER CALLBACK] Zero bytes produced!\n");
        return false;
    }

    H264EncoderNvmpictx *ctx = static_cast<H264EncoderNvmpictx *>(arg);
    if (ctx->packets_buf_size < bytesused) {
        printf("[ENCODER CALLBACK] Resizing packet buffers to %d\n", bytesused);
        ctx->packets_buf_size = bytesused;
        for (int index = 0; index < ENC_MAX_BUFFERS; index++) {
            delete[] ctx->packets[index];
            ctx->packets[index]      = new uint8_t[ctx->packets_buf_size];
            ctx->packets_size[index] = 0;
        }
    }

    ctx->packets_size[ctx->out_h264_buffer_index] = bytesused;
    memcpy(ctx->packets[ctx->out_h264_buffer_index], buffer->planes[0].data, bytesused);

    {
        std::lock_guard<std::mutex> lock(ctx->buffer_mutex);
        ctx->packet_pools.push(ctx->out_h264_buffer_index);
        printf("[ENCODER CALLBACK] Pushed packet to queue, queue size=%zu\n", ctx->packet_pools.size());
        ctx->out_h264_buffer_index = (ctx->out_h264_buffer_index + 1) % ENC_MAX_BUFFERS;

        // Pop the oldest buffer if the queue size exceeds the maximum
        while (ctx->packet_pools.size() > ENC_MAX_BUFFERS) {
            ctx->packet_pools.pop();
        }
    }

    return (ctx->enc->capture_plane.qBuffer(*v4l2_buf, NULL) == 0);
}

H264Encoder::H264Encoder(int32_t width, int32_t height, int32_t bitrate, int32_t fps, bool use_all_intra) {
    ctx                        = new H264EncoderNvmpictx();
    ctx->in_frame_buffer_index = 0;
    ctx->out_h264_buffer_index = 0;

    ctx->width  = width;
    ctx->height = height;
    ctx->fps_n  = fps;
    ctx->qmax   = 32;
    ctx->qmin   = 0;

    const bool is_more_than_FHD = (width > 1'920 || height > 1'080);
    ctx->packets_buf_size       = (is_more_than_FHD ? 4 : 1) * UINT16_MAX;

    for (int index = 0; index < ENC_MAX_BUFFERS; index++) {
        ctx->packets[index]      = new uint8_t[ctx->packets_buf_size];
        ctx->packets_size[index] = 0;
    }


    ctx->enc = NvVideoEncoder::createVideoEncoder("enc0");
    TEST_ERROR(!ctx->enc, "Could not create encoder");

    int ret = ctx->enc->setCapturePlaneFormat(V4L2_PIX_FMT_H264, ctx->width, ctx->height, ctx->packets_buf_size);
    TEST_ERROR(ret < 0, "Could not set output plane format");

    ret = ctx->enc->setOutputPlaneFormat(V4L2_PIX_FMT_NV12M, ctx->width, ctx->height);
    TEST_ERROR(ret < 0, "Could not set output plane format");

    ret = ctx->enc->setBitrate(bitrate);
    TEST_ERROR(ret < 0, "Could not set encoder bitrate");

    ret = ctx->enc->setHWPresetType(V4L2_ENC_HW_PRESET_MEDIUM);
    TEST_ERROR(ret < 0, "Could not set encoder HW Preset Type");

    ret = ctx->enc->setProfile(
            V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE);  // V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE
    TEST_ERROR(ret < 0, "Could not set encoder profile");

    ret = ctx->enc->setNumBFrames(0);
    TEST_ERROR(ret < 0, "Could not set number of B Frames");

    if (use_all_intra) {
        ret = ctx->enc->setIFrameInterval(1);
        TEST_ERROR(ret < 0, "Could not set I-Frame interval");

        ret = ctx->enc->setAlliFramesEncode(true);
        TEST_ERROR(ret < 0, "Could not set all I frames encode");
    } else {

        ret = ctx->enc->setIFrameInterval(ctx->fps_n);
        TEST_ERROR(ret < 0, "Could not set encoder I-Frame interval");
    }

    ret = ctx->enc->setLevel(V4L2_MPEG_VIDEO_H264_LEVEL_5_1);
    TEST_ERROR(ret < 0, "Could not set encoder level");

    ret = ctx->enc->setRateControlMode(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
    TEST_ERROR(ret < 0, "Could not set encoder rate control mode");

    ret = ctx->enc->setIDRInterval(ctx->fps_n);
    TEST_ERROR(ret < 0, "Could not set encoder IDR interval");

    ctx->enc->setQpRange(ctx->qmin, ctx->qmax, ctx->qmin, ctx->qmax, ctx->qmin, ctx->qmax);

    ret = ctx->enc->setInsertSpsPpsAtIdrEnabled(true);
    TEST_ERROR(ret < 0, "Could not set insertSPSPPSAtIDR");

    ret = ctx->enc->setFrameRate(ctx->fps_n, 1);
    TEST_ERROR(ret < 0, "Could not set framerate");

    // MIGRATION: V4L2_MEMORY_MMAP with map=true to allow CPU memcpy
    ret = ctx->enc->output_plane.setupPlane(V4L2_MEMORY_MMAP, ENC_MAX_BUFFERS, true, true);
    TEST_ERROR(ret < 0, "Could not setup output plane");

    ret = ctx->enc->capture_plane.setupPlane(V4L2_MEMORY_MMAP, ENC_MAX_BUFFERS, true, false);
    TEST_ERROR(ret < 0, "Could not setup capture plane");

    ret = ctx->enc->subscribeEvent(V4L2_EVENT_EOS, 0, 0);
    TEST_ERROR(ret < 0, "Could not subscribe EOS event");

    ret = ctx->enc->output_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in output plane streamon");

    ret = ctx->enc->capture_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in capture plane streamon");

    ctx->enc->capture_plane.setDQThreadCallback(encoder_capture_plane_dq_callback);
    ctx->enc->capture_plane.startDQThread(ctx);

    for (uint32_t idx = 0; idx < ctx->enc->capture_plane.getNumBuffers(); idx++) {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index    = idx;
        v4l2_buf.m.planes = planes;

        ret = ctx->enc->capture_plane.qBuffer(v4l2_buf, NULL);
        TEST_ERROR(ret < 0, "Error while queueing buffer at capture plane");
    }
}

H264Encoder::~H264Encoder() {
    ctx->enc->capture_plane.stopDQThread();
    ctx->enc->capture_plane.waitForDQThread(2'000);

    for (int index = 0; index < ENC_MAX_BUFFERS; index++) {
        delete[] ctx->packets[index];
    }

    delete ctx->enc;
    delete ctx;
}

bool H264Encoder::encodeFrame(Msg_ImageH264Feed &msg, NvBufSurface *src_surf) {
    if (!nvmpi_encoder_put_frame(src_surf)) {
        return false;
    }

    if (!nvmpi_encoder_get_packet(msg)) {
        // Return TRUE to indicate no error, just no data ready yet (buffering/latency)
        msg.data.clear();
        return true;
    }

    msg.width  = ctx->width;
    msg.height = ctx->height;

    return true;
}

bool H264Encoder::nvmpi_encoder_put_frame(NvBufSurface *src_surf) {
    printf("[ENCODER PUT] Start, in_buffer_index=%d\n", ctx->in_frame_buffer_index);

    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer *nvBuffer;

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, sizeof(planes));
    v4l2_buf.m.planes = planes;

    if (ctx->enc->isInError()) {
        printf("[ENCODER PUT] ERROR: Encoder is in error state!\n");
        return false;
    }

    if (ctx->in_frame_buffer_index < ctx->enc->output_plane.getNumBuffers()) {
        nvBuffer       = ctx->enc->output_plane.getNthBuffer(ctx->in_frame_buffer_index);
        v4l2_buf.index = ctx->in_frame_buffer_index;
        printf("[ENCODER PUT] Using buffer index %d (initial phase)\n", v4l2_buf.index);
    } else if (ctx->enc->output_plane.dqBuffer(v4l2_buf, &nvBuffer, NULL, -1) < 0) {
        printf("[ENCODER PUT] ERROR: Failed to dqBuffer from output plane!\n");
        return false;
    } else {
        printf("[ENCODER PUT] Using buffer index %d (recycled)\n", v4l2_buf.index);
    }

    uint64_t timestamp_us = (uint64_t)ctx->in_frame_buffer_index * 1'000'000 / ctx->fps_n;
    v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
    v4l2_buf.timestamp.tv_sec  = timestamp_us / 1'000'000;
    v4l2_buf.timestamp.tv_usec = timestamp_us % 1'000'000;
    ctx->in_frame_buffer_index++;

    // NV12 Size Calculation (2 Planes)
    const int32_t plane_y_bytes  = ctx->width * ctx->height;
    const int32_t plane_uv_bytes = ctx->width * (ctx->height / 2);

    nvBuffer->planes[0].bytesused = plane_y_bytes;
    nvBuffer->planes[1].bytesused = plane_uv_bytes;

    printf("[ENCODER PUT] Set bytesused: plane0=%d, plane1=%d\n",
           nvBuffer->planes[0].bytesused,
           nvBuffer->planes[1].bytesused);

    for (int i = 0; i < MAX_PLANES; i++) {
        v4l2_buf.m.planes[i].bytesused = nvBuffer->planes[i].bytesused;
    }

    printf("[ENCODER PUT] Mapping source surface...\n");
    if (NvBufSurfaceMap(src_surf, 0, -1, NVBUF_MAP_READ) != 0) {
        printf("[ENCODER PUT] ERROR: Failed to map source surface!\n");
        return false;
    }
    NvBufSurfaceSyncForCpu(src_surf, 0, -1);

    printf("[ENCODER PUT] Copying YUV data (NV12)...\n");
    // COPY LOGIC FOR NV12 (2 PLANES ONLY)
    for (int i = 0; i < 2; i++) {
        const int32_t plane_h = (i == 0) ? ctx->height : (ctx->height / 2);
        const int32_t plane_w = ctx->width;
        uint32_t src_pitch    = src_surf->surfaceList[0].planeParams.pitch[i];
        uint32_t dst_pitch    = nvBuffer->planes[i].fmt.stride;
        uint8_t *src          = (uint8_t *)src_surf->surfaceList[0].mappedAddr.addr[i];
        uint8_t *dst          = (uint8_t *)nvBuffer->planes[i].data;

        printf("[ENCODER PUT] Plane %d: h=%d, w=%d, src_pitch=%u, dst_pitch=%u\n",
               i,
               plane_h,
               plane_w,
               src_pitch,
               dst_pitch);

        if (src_pitch == dst_pitch && src_pitch == (uint32_t)plane_w) {
            memcpy(dst, src, plane_w * plane_h);
        } else {
            for (int32_t row = 0; row < plane_h; row++) {
                memcpy(dst + row * dst_pitch, src + row * src_pitch, plane_w);
            }
        }
    }
    NvBufSurfaceUnMap(src_surf, 0, -1);

    printf("[ENCODER PUT] Queueing buffer to encoder...\n");
    int ret = ctx->enc->output_plane.qBuffer(v4l2_buf, NULL);
    if (ret < 0) {
        printf("[ENCODER PUT] ERROR: Failed to qBuffer! ret=%d\n", ret);
        return false;
    }

    printf("[ENCODER PUT] Success!\n");
    return true;
}


bool H264Encoder::nvmpi_encoder_get_packet(Msg_ImageH264Feed &msg) {

    std::lock_guard<std::mutex> lock(ctx->buffer_mutex);
    if (ctx->packet_pools.empty()) {
        return false;
    }

    const int32_t packet_index = ctx->packet_pools.front();
    ctx->packet_pools.pop();

    uint32_t size = ctx->packets_size[packet_index];
    if (size == 0) {
        return false;
    }

    msg.data.clear();
    msg.data.assign(ctx->packets[packet_index], ctx->packets[packet_index] + size);

    ctx->packets_size[packet_index] = 0;  // mark as read
    return true;
}