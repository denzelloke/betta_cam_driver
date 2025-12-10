// Copyright by BeeX [2024]
#ifndef H264_DECODER_H
#define H264_DECODER_H

#include <bx_msgs/Memory.hpp>
#include <bx_msgs/RateLimiter.hpp>
#include <bx_msgs/RosBindings.hpp>

#include <NvBuffer.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

#include "NvVideoDecoder.h"


class H264Decoder {
public:
    explicit H264Decoder();
    ~H264Decoder();

    bool decoder_put_packet(const Msg_ImageH264Feed_ConstPtr &msg);
    bool decoder_get_frame(NvBufSurface *out_surf);

private:
    bool has_first_frame_ = false;
    bool eos_             = false;
    bool got_res_event_   = false;

    NvVideoDecoder *dec_ = nullptr;

    int32_t buffer_index_         = 0;
    uint32_t coded_width_         = 0;
    uint32_t coded_height_        = 0;
    int32_t numberCaptureBuffers_ = 0;

    std::thread *dec_capture_loop_ = nullptr;
    std::mutex buffer_mutex_;

    std::queue<int32_t> frame_pools_;
    SafeVector<NvBufSurface *> intermediate_surf_vec_;
    NvBufSurfTransformParams transform_params_;

    NvBufSurfTransformRect src_rect_params_;
    NvBufSurfTransformRect dst_rect_params_;

    void respondToResolutionEvent();
    void dec_capture_loop_fcn();
    void nvmpi_create_decoder();
    void nvmpi_decoder_close();
};


#endif  // H264_DECODER_H
