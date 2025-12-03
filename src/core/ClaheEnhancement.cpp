// Copyright by BeeX [2024]
#include <h264_camera_driver/core/ClaheEnhancement.h>

/* @brief Constructor for CLAHE enhancement class
 *        Initializes OpenCV's CLAHE (Contrast Limited Adaptive Histogram Equalization)
 *        object with a clip limit of 2.0 for controlling contrast enhancement limits
 */
ClaheEnhancement::ClaheEnhancement() { clahe_K = cv::createCLAHE(4.0, cv::Size(8, 8)); }

/* @brief Applies CLAHE enhancement and color correction to YUV image */
void ClaheEnhancement::apply_clahe(cv::Mat &out_yuv_image) {
    if (out_yuv_image.empty()) {
        return;
    }

    const int32_t height  = out_yuv_image.rows * 2 / 3;  // Original Y plane height
    const int32_t width   = out_yuv_image.cols;
    const int32_t y_size  = width * height;
    const int32_t uv_size = y_size / 4;

    if (out_yuv_image.total() < (y_size * 3 / 2)) {
        return;
    }

    // Handles UV plane color correction
    // See https://ant2sky.blogspot.com/2020/04/python-opencv-nv21-yuv420-rgb.html
    int32_t sum_u   = 0;
    uint8_t *u_data = out_yuv_image.data + y_size;
    for (int32_t ch_idx = 0; ch_idx < uv_size; ch_idx++) {
        sum_u += *u_data;
        u_data++;
    }
    const int32_t mean_u  = sum_u / uv_size;
    const int32_t delta_u = 128 - mean_u;

    int32_t sum_v   = 0;
    uint8_t *v_data = out_yuv_image.data + y_size + uv_size;
    for (int32_t ch_idx = 0; ch_idx < uv_size; ch_idx++) {
        sum_v += *v_data;
        v_data++;
    }
    const int32_t mean_v  = sum_v / uv_size;
    const int32_t delta_v = 128 - mean_v;

    u_data = out_yuv_image.data + y_size;
    for (int32_t ch_idx = 0; ch_idx < uv_size; ch_idx++) {
        *u_data = cv::saturate_cast<uchar>(*u_data + delta_u);
        u_data++;
    }

    v_data = out_yuv_image.data + y_size + uv_size;
    for (int32_t ch_idx = 0; ch_idx < uv_size; ch_idx++) {
        *v_data = cv::saturate_cast<uchar>(*v_data + delta_v);
        v_data++;
    }


    // Apply CLAHE to Y plane
    cv::Mat y_plane(height, width, CV_8UC1, out_yuv_image.data);
    clahe_K->apply(y_plane, y_plane);
    // cv::equalizeHist(y_plane, y_plane);


    // Handles Y plane color correction
    int32_t sum_y   = 0;
    uint8_t *y_data = out_yuv_image.data;
    for (int32_t idx = 0; idx < y_size; ++idx) {
        sum_y += *(y_data++);
    }
    const int32_t mean_y  = sum_y / y_size;
    const int32_t delta_y = 130 - mean_y;

    y_data = out_yuv_image.data;
    for (int32_t idx = 0; idx < y_size; ++idx) {
        *y_data = cv::saturate_cast<uchar>(*y_data + delta_y);
        y_data++;
    }

    cv::medianBlur(y_plane, y_plane, 3);


    // printf("Shift: %d %d %d | Mean: %d %d %d\n", delta_y, delta_u, delta_v, mean_y, mean_u, mean_v);
}