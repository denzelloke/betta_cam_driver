/*
g++ -o rtsp_decoder_with_opencv rtsp_decoder_with_opencv.cpp \
    -I/usr/include/opencv4 \
    -lavformat -lavcodec -lavutil -lswscale \
    -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio \
    -pthread -ldl && ./rtsp_decoder_with_opencv



*/

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>  // Required for av_image_get_buffer_size and av_image_fill_arrays
#include <libswscale/swscale.h>
}

#include <iostream>
#include <opencv2/opencv.hpp>

void decode_stream(const char *rtsp_url) {
    AVFormatContext *format_context = nullptr;
    AVCodecContext *codec_context   = nullptr;
    AVCodec *codec                  = nullptr;
    int video_stream_index          = -1;

    // Register all formats and codecs (this is needed for older FFmpeg versions)
    av_register_all();
    avformat_network_init();

    // Open the RTSP stream
    if (avformat_open_input(&format_context, rtsp_url, nullptr, nullptr) != 0) {
        std::cerr << "Could not open RTSP stream: " << rtsp_url << std::endl;
        return;
    }

    // Retrieve stream information
    if (avformat_find_stream_info(format_context, nullptr) < 0) {
        std::cerr << "Could not find stream information." << std::endl;
        avformat_close_input(&format_context);
        return;
    }

    // Find the best video stream
    video_stream_index = av_find_best_stream(format_context, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
    if (video_stream_index < 0) {
        std::cerr << "Could not find a video stream." << std::endl;
        avformat_close_input(&format_context);
        return;
    }

    // Get codec context for the video stream
    codec_context = avcodec_alloc_context3(codec);
    if (!codec_context) {
        std::cerr << "Could not allocate codec context." << std::endl;
        avformat_close_input(&format_context);
        return;
    }

    if (avcodec_parameters_to_context(codec_context, format_context->streams[video_stream_index]->codecpar) < 0) {
        std::cerr << "Could not copy codec parameters to context." << std::endl;
        avcodec_free_context(&codec_context);
        avformat_close_input(&format_context);
        return;
    }

    // Find the decoder for the video stream
    codec = avcodec_find_decoder(codec_context->codec_id);
    if (!codec) {
        std::cerr << "Codec not found." << std::endl;
        avcodec_free_context(&codec_context);
        avformat_close_input(&format_context);
        return;
    }

    // Open codec
    if (avcodec_open2(codec_context, codec, nullptr) < 0) {
        std::cerr << "Could not open codec." << std::endl;
        avcodec_free_context(&codec_context);
        avformat_close_input(&format_context);
        return;
    }

    // Prepare for reading frames
    AVPacket packet;
    AVFrame *frame     = av_frame_alloc();
    AVFrame *rgb_frame = av_frame_alloc();
    if (!frame || !rgb_frame) {
        std::cerr << "Could not allocate frame." << std::endl;
        avcodec_free_context(&codec_context);
        avformat_close_input(&format_context);
        return;
    }

    // Set up the conversion context
    SwsContext *sws_ctx = sws_getContext(
            codec_context->width,
            codec_context->height,
            codec_context->pix_fmt,
            codec_context->width,
            codec_context->height,
            AV_PIX_FMT_BGR24,
            SWS_BILINEAR,
            nullptr,
            nullptr,
            nullptr);
    if (!sws_ctx) {
        std::cerr << "Could not initialize the conversion context." << std::endl;
        av_frame_free(&frame);
        av_frame_free(&rgb_frame);
        avcodec_free_context(&codec_context);
        avformat_close_input(&format_context);
        return;
    }

    // Allocate buffer for the RGB frame
    int buffer_size = av_image_get_buffer_size(AV_PIX_FMT_BGR24, codec_context->width, codec_context->height, 1);
    uint8_t *buffer = (uint8_t *)av_malloc(buffer_size);
    if (!buffer) {
        std::cerr << "Could not allocate buffer for RGB frame." << std::endl;
        sws_freeContext(sws_ctx);
        av_frame_free(&frame);
        av_frame_free(&rgb_frame);
        avcodec_free_context(&codec_context);
        avformat_close_input(&format_context);
        return;
    }

    av_image_fill_arrays(
            rgb_frame->data,
            rgb_frame->linesize,
            buffer,
            AV_PIX_FMT_BGR24,
            codec_context->width,
            codec_context->height,
            1);


    cv::namedWindow("DecodedFrame", cv::WINDOW_NORMAL);

    // Read frames from the stream
    bool should_continue = true;
    while (should_continue && av_read_frame(format_context, &packet) >= 0) {
        if (packet.stream_index != video_stream_index) {
            av_packet_unref(&packet);
        }

        // Decode video frame
        int ret = avcodec_send_packet(codec_context, &packet);
        if (ret < 0) {
            std::cerr << "Error sending packet to codec context." << std::endl;
            break;
        }

        while (should_continue && ret >= 0) {
            ret = avcodec_receive_frame(codec_context, frame);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break;
            } else if (ret < 0) {
                std::cerr << "Error receiving frame from codec context." << std::endl;
                break;
            }

            // Convert the frame to RGB
            sws_scale(
                    sws_ctx,
                    frame->data,
                    frame->linesize,
                    0,
                    codec_context->height,
                    rgb_frame->data,
                    rgb_frame->linesize);

            // Create an OpenCV Mat from the RGB frame
            cv::Mat img(codec_context->height, codec_context->width, CV_8UC3, rgb_frame->data[0]);

            // Display the image
            cv::imshow("DecodedFrame", img);

            // Wait for a key press to close the window (optional)
            if (cv::waitKey(1) >= 0) {
                should_continue = false;
            }
        }
    }

    // Cleanup
    av_free(buffer);
    sws_freeContext(sws_ctx);
    av_frame_free(&frame);
    av_frame_free(&rgb_frame);
    avcodec_free_context(&codec_context);
    avformat_close_input(&format_context);
    cv::destroyAllWindows();
}

int main() {
    const char *rtsp_url = "rtsp://192.168.10.160:8554/0";  // Replace with your RTSP stream URL
    decode_stream(rtsp_url);
    return 0;
}
