// Copyright by BeeX [2024]
#ifndef H264_DECODER_H
#define H264_DECODER_H

#include <bx_msgs/Memory.hpp>
#include <bx_msgs/RateLimiter.hpp>
#include <bx_msgs/RosBindings.hpp>

#include <NvBuffer.h>
// #include <nvbuf_utils.h>
#include <h264_camera_driver/compat/nvbuf_compat.h>

#include "NvVideoDecoder.h"


class H264Decoder {
public:
    explicit H264Decoder();
    ~H264Decoder();

    bool decoder_put_packet(const Msg_ImageH264Feed_ConstPtr &msg);
    bool decoder_get_frame(const int32_t out_dma_buf_2K);

private:
    bool has_first_frame = false;
    bool eos             = false;
    bool got_res_event   = false;

    NvVideoDecoder *dec = nullptr;

    int32_t buffer_index         = 0;
    uint32_t coded_width         = 0;
    uint32_t coded_height        = 0;
    int32_t numberCaptureBuffers = 0;

    std::thread *dec_capture_loop = nullptr;
    std::mutex buffer_mutex;

    std::queue<int32_t> frame_pools;
    SafeVector<int32_t> dmaBufferFileDescriptors;
    NvBufferTransformParams transform_params;

    void respondToResolutionEvent();
    void dec_capture_loop_fcn();
    void nvmpi_create_decoder();
    void nvmpi_decoder_close();
};


#endif  // H264_DECODER_H
