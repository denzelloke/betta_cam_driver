// Copyright by BeeX [2024]
#ifndef CLAHE_ENHANCEMENT_H
#define CLAHE_ENHANCEMENT_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

class ClaheEnhancement {
public:
    ClaheEnhancement();
    void apply_clahe(cv::Mat &out_yuv_image);

private:
    cv::Ptr<cv::CLAHE> clahe_K;
};

#endif  // CLAHE_ENHANCEMENT_H