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


    if (v4l2_buf == NULL) {
        return false;
    }

    const int32_t bytesused = buffer->planes[0].bytesused;
    if (bytesused == 0) {
        return false;
    }

    H264EncoderNvmpictx *ctx = static_cast<H264EncoderNvmpictx *>(arg);
    if (ctx->packets_buf_size < bytesused) {
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

    ret = ctx->enc->setOutputPlaneFormat(
            V4L2_PIX_FMT_NV12M,
            ctx->width,
            ctx->height);  // YUV420->NV12: V4L2_PIX_FMT_YUV420M
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

    // MIGRATION NOTES:
    // V4L2_MEMORY_USERPTR --> V4L2_MEMORY_MMAP
    // rationale:
    // V4L2_MEMORY_USERPTR -> application allocates memory buffers (using new / malloc) and pass the ptrs to the encoder
    // V4L2_MEMORY_MMAP -> encoder driver allocates the buffers itself within hardware.
    // you js ask for a pointer to map it to copy ur data into
    // ret = ctx->enc->output_plane.setupPlane(V4L2_MEMORY_MMAP, ENC_MAX_BUFFERS, true, false);
    ret = ctx->enc->output_plane.setupPlane(V4L2_MEMORY_USERPTR, ENC_MAX_BUFFERS, false, true);

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

bool H264Encoder::encodeFrame(Msg_ImageH264Feed &msg, NvBufSurface *surf) {
    if (!nvmpi_encoder_put_frame(surf)) {
        printf("\n\n111111111111111111111111111\n\n");
        // return true;
    }

    if (!nvmpi_encoder_get_packet(msg)) {
        printf("\n\n2222222222222222222222222222\n\n");
        return false;
    }

    msg.width  = ctx->width;
    msg.height = ctx->height;

    return true;
}

bool H264Encoder::nvmpi_encoder_put_frame(NvBufSurface *surf) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer *nvBuffer;

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, sizeof(planes));

    v4l2_buf.m.planes = planes;

    if (ctx->enc->isInError()) {
        return false;
    }

    // Copy the dma buffer to the nvbuffer
    if (ctx->in_frame_buffer_index < ctx->enc->output_plane.getNumBuffers()) {
        nvBuffer       = ctx->enc->output_plane.getNthBuffer(ctx->in_frame_buffer_index);
        v4l2_buf.index = ctx->in_frame_buffer_index;

    } else if (ctx->enc->output_plane.dqBuffer(v4l2_buf, &nvBuffer, NULL, -1) < 0) {
        return false;
    }
    ctx->in_frame_buffer_index++;

    v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
    v4l2_buf.timestamp.tv_usec = (ctx->in_frame_buffer_index * 1'000'000) % (ctx->fps_n * 1'000'000);
    v4l2_buf.timestamp.tv_sec  = ctx->in_frame_buffer_index / ctx->fps_n;

    // Fill plane sizes for NV12
    // Plane 0 (Y): 1 byte per pixel
    // Plane 1 (UV): 2 bytes per 2 pixels (interleaved) -> effectively 1 byte per pixel-pair width-wise,
    //               but full width in bytes. Total size is Y_Size / 2.
    const int32_t plane_y_size  = ctx->width * ctx->height;
    const int32_t plane_uv_size = plane_y_size / 2;

    // Fill NvBuffer (NV12 has only 2 planes)
    nvBuffer->planes[0].bytesused = plane_y_size;
    nvBuffer->planes[1].bytesused = plane_uv_size;

    NvBufSurfaceParams &surf_params = surf->surfaceList[0];

    // Loop for 2 planes (Y and UV)
    for (int channel_idx = 0; channel_idx < 2; channel_idx++) {

        // map dma memory into surf-> surfaceList->mappedAddr->addr
        if (NvBufSurfaceMap(surf, 0, channel_idx, NVBUF_MAP_READ) != 0) {
            return false;
        }

        // extract mapped memory and cast to src
        void *src_data = surf_params.mappedAddr.addr[channel_idx];

        // sync before read operation to ensure cpu sees the latest data from hardware
        if (NvBufSurfaceSyncForCpu(surf, 0, channel_idx) != 0) {
            return false;
        }

        // NV12 Plane Dimensions
        // Plane 0 (Y):  Height = H,     Width (bytes) = W
        // Plane 1 (UV): Height = H / 2, Width (bytes) = W  <-- Important: Width is NOT divided by 2
        const int32_t plane_h = (channel_idx == 0) ? ctx->height : ctx->height / 2;
        const int32_t plane_w = ctx->width;

        const int32_t plane_pitch = surf_params.planeParams.pitch[channel_idx];

        uint8_t *src = static_cast<uint8_t *>(src_data);
        uint8_t *dst = static_cast<uint8_t *>(nvBuffer->planes[channel_idx].data);

        // Safety check for NULL buffer mapping
        if (!dst) {
            printf("Error: Encoder input buffer plane %d is NULL\n", channel_idx);
            NvBufSurfaceUnMap(surf, 0, channel_idx);
            return false;
        }

        for (int32_t row = 0; row < plane_h; row++) {
            memcpy(dst + row * plane_w, src + row * plane_pitch, plane_w);
        }


        // TURN ON IN CASE OF GREEN-NESS
        if (channel_idx == 0) {  // Y plane
            printf("[ENCODER] Y plane: ");
            printf("First 16 Y: ");
            for (int i = 0; i < 16; i++) {
                printf("%d ", src[i]);
            }
            printf("\n");
        } else if (channel_idx == 1) {  // UV plane
            printf("[ENCODER] UV plane: ");
            printf("First 16 UV: ");
            for (int i = 0; i < 16; i++) {
                printf("%d ", src[i]);
            }
            printf(" (sum of first 100: ");
            int sum = 0;
            for (int i = 0; i < 100; i++) {
                sum += src[i];
            }
            printf("%d, avg: %d)\n\n", sum, sum / 100);
        }

        // unmap memory
        NvBufSurfaceUnMap(surf, 0, channel_idx);
    }

    int32_t ret = ctx->enc->output_plane.qBuffer(v4l2_buf, NULL);
    TEST_ERROR(ret < 0, "Error while queueing buffer at output plane");

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
    if (size == 0) {  // Old packet, but 0-0 skip!
        return false;
    }

    msg.data.clear();
    msg.data.assign(ctx->packets[packet_index], ctx->packets[packet_index] + size);

    ctx->packets_size[packet_index] = 0;  // mark as read
    return true;
}