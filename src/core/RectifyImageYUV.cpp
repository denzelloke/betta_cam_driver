// Copyright by BeeX [2024]
#include <h264_camera_driver/core/RectifyImageYUV.h>

RectifyImageYUV::RectifyImageYUV(const std::string &calibration_file_path)
        : lookup_array(createLookupTable(calibration_file_path)) { }

bool RectifyImageYUV::rectifyImage(cv::Mat &out_dst_rectified_image, const cv::Mat &yuv_image_src) {

    const int32_t lookup_array_size = lookup_array.size();
    if (lookup_array_size == 0) {
        out_dst_rectified_image = yuv_image_src;
        return true;
    }

    if (lookup_array_size != yuv_image_src.total() || yuv_image_src.type() != CV_8UC1) {
        LOG_ERROR(
                "Invalid input image dimensions or format. Expecting %d pixels, got %d pixels.\n",
                lookup_array_size,
                yuv_image_src.total());
        return false;
    }

    if (out_dst_rectified_image.type() != CV_8UC1 || out_dst_rectified_image.rows != yuv_image_src.rows
        || out_dst_rectified_image.cols != yuv_image_src.cols) {

        out_dst_rectified_image.create(yuv_image_src.rows, yuv_image_src.cols, CV_8UC1);
    }

    const int32_t *mappingPtr = lookup_array.data();
    const uint8_t *srcPtr     = yuv_image_src.data;
    uint8_t *dstPtr           = out_dst_rectified_image.data;

    for (int idx = 0; idx < lookup_array_size; ++idx) {
       int mapIdx = mappingPtr[idx];
       dstPtr[idx] = (mapIdx >= 0)
                  ? srcPtr[mapIdx]
                  : 0;
    }

    return true;
}

// Helper function to read matrix from YAML node
cv::Mat RectifyImageYUV::readMatrix(const YAML::Node &node) const {
    std::vector<float> data = node["data"].as<std::vector<float>>();
    const int32_t rows      = node["rows"].as<int32_t>();
    const int32_t cols      = node["cols"].as<int32_t>();
    return cv::Mat(rows, cols, CV_32F, data.data()).clone();
};


SafeVector<int32_t> RectifyImageYUV::createLookupTable(const std::string &calibration_file_path) const {

    SafeVector<int32_t> res_lookup_array;

    try {
        YAML::Node config = YAML::LoadFile(calibration_file_path);

        if (config.IsNull()) {
            LOG_ERROR("Failed to load the YAML file.\n");
            return res_lookup_array;
        }

        LOG_INFO("Loading camera calibration from %s\n", calibration_file_path.c_str());

        // Reading matrices
        const int32_t imageWidth           = config["image_width"].as<int32_t>();
        const int32_t imageHeight          = config["image_height"].as<int32_t>();
        const cv::Mat camera_matrix        = readMatrix(config["camera_matrix"]);
        const cv::Mat dist_coeffs          = readMatrix(config["distortion_coefficients"]);
        const cv::Mat rectification_matrix = readMatrix(config["rectification_matrix"]);
        const cv::Mat projection_matrix    = readMatrix(config["projection_matrix"]);

        cv::Size imageSize(imageWidth, imageHeight);

        cv::Mat mapping_x, mapping_y;
        cv::initUndistortRectifyMap(
                camera_matrix,
                dist_coeffs,
                rectification_matrix,
                projection_matrix.rowRange(0, 3).colRange(0, 3),
                imageSize,
                CV_32FC1,
                mapping_x,
                mapping_y);

        const int32_t plane_uv_width = imageWidth / 2;
        const int32_t plane_y_size   = imageWidth * imageHeight;
        const int32_t plane_uv_size  = (imageWidth / 2) * (imageHeight / 2);

        res_lookup_array.assign(plane_y_size + 2 * plane_uv_size, -1);

        for (int32_t dst_y = 0; dst_y < mapping_x.rows; ++dst_y) {
            const bool is_even_row = ((dst_y % 2) == 0);

            for (int32_t dst_x = 0; dst_x < mapping_x.cols; ++dst_x) {
                const int32_t src_x = static_cast<int32_t>(std::round(mapping_x.at<float>(dst_y, dst_x)));
                if (src_x < 0 && imageWidth <= src_x) {
                    continue;
                }

                // Y plane
                const int32_t src_y = static_cast<int32_t>(std::round(mapping_y.at<float>(dst_y, dst_x)));
                if (src_y < 0 && imageHeight <= src_y) {
                    continue;
                }
                const int32_t planeY_dst_index     = dst_y * imageWidth + dst_x;
                const int32_t planeY_src_index     = src_y * imageWidth + src_x;
                res_lookup_array[planeY_dst_index] = planeY_src_index;

                // Check if the current pixel is in an even row and column
                const bool is_even_col = ((dst_x % 2) == 0);
                if (!is_even_row || !is_even_col) {
                    continue;
                }

                // U plane
                const int32_t planeU_dst_index     = (dst_y / 2) * plane_uv_width + (dst_x / 2) + plane_y_size;
                const int32_t planeU_src_index     = (src_y / 2) * plane_uv_width + (src_x / 2) + plane_y_size;
                res_lookup_array[planeU_dst_index] = planeU_src_index;

                // V plane
                const int32_t planeV_dst_index     = planeU_dst_index + plane_uv_size;
                const int32_t planeV_src_index     = planeU_src_index + plane_uv_size;
                res_lookup_array[planeV_dst_index] = planeV_src_index;
            }
        }

    } catch (const YAML::Exception &e) {
        LOG_ERROR("Failed to load calibration matrices and dimensions.");
        LOG_ERROR("Exception caught: %s", e.what());

    } catch (const std::exception &e) {
        LOG_ERROR("Failed to load calibration matrices and dimensions.");
        LOG_ERROR("Standard exception caught: %s", e.what());
    }

    return res_lookup_array;
}