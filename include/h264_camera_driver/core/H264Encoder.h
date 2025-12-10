// Copyright by BeeX [2024]
#ifndef H264_ENCODER_H
#define H264_ENCODER_H

#include <bx_msgs/Memory.hpp>
#include <bx_msgs/RateLimiter.hpp>
#include <bx_msgs/RosBindings.hpp>

#include <NvBuffer.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

#include "NvVideoEncoder.h"

#include <queue>

#define ENC_MAX_BUFFERS 2

struct H264EncoderNvmpictx {
    NvVideoEncoder *enc;
    int32_t in_frame_buffer_index;
    int32_t out_h264_buffer_index;

    std::queue<int32_t> packet_pools;
    uint32_t width;
    uint32_t height;

    uint32_t fps_n;

    uint32_t qmax;
    uint32_t qmin;

    bool insert_sps_pps_at_idr;
    uint32_t packets_buf_size;
    uint8_t *packets[ENC_MAX_BUFFERS];
    uint32_t packets_size[ENC_MAX_BUFFERS];

    std::mutex buffer_mutex;
};

class H264Encoder {
public:
    explicit H264Encoder(int32_t width, int32_t height, int32_t bitrate, int32_t fps, bool use_all_intra = false);
    ~H264Encoder();

    bool encodeFrame(Msg_ImageH264Feed &msg, NvBufSurface *surf);

private:
    H264EncoderNvmpictx *ctx;

    bool nvmpi_encoder_put_frame(NvBufSurface *surf);
    bool nvmpi_encoder_get_packet(Msg_ImageH264Feed &msg);
};

#endif  // H264_ENCODER_H