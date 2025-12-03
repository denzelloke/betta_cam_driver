// Copyright by BeeX [2024]
#ifndef RECTIFY_IMAGE_YUV_H
#define RECTIFY_IMAGE_YUV_H

#include <yaml-cpp/yaml.h>
#include <bx_msgs/Memory.hpp>
#include <bx_msgs/RosBindings.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

class RectifyImageYUV {
public:
    explicit RectifyImageYUV(const std::string &calibration_file_path);

    bool rectifyImage(cv::Mat &out_dst_rectified_image, const cv::Mat &yuv_image_src);

private:
    const SafeVector<int32_t> lookup_array;

    cv::Mat readMatrix(const YAML::Node &node) const;
    SafeVector<int32_t> createLookupTable(const std::string &calibration_file_path) const;
};
#endif  // RECTIFY_IMAGE_YUV_H