// Copyright by BeeX [2024]
#ifndef IMX_477_H_
#define IMX_477_H_

#include <h264_camera_driver/core/CameraFeedWrapper.h>

#define CAMERA_FPS 12  // Hz

class Imx477Driver : public CameraFeedWrapper {
public:
    Imx477Driver(
            const int cam_port_,
            const bool flip_image_,
            const std::string frame_id,
            const std::string ros_topic_root_name,
            const std::string calibration_file_name);
    ~Imx477Driver();


    void captureAndPublishOnce(Srv_CamGetSnapshot_Response *res_ptr) override;

    void capture_n_pub();

private:
    // Configurable Parameters
    const int cam_port;
    const bool flip_image;

    // Additional member variables
    const uint64_t CAPTURE_TIMEOUT_NS = 1'000'000'000;  // 1sec in nanoseconds


    // Separate resources for thread
    std::vector<CameraDevice *> _cameraDevices;

    UniqueObj<CameraProvider> _cameraProvider;
    UniqueObj<CaptureSession> _captureSession;
    UniqueObj<OutputStream> _stream;
    UniqueObj<OutputStreamSettings> _streamSettings;
    UniqueObj<FrameConsumer> _frameConsumer;
    UniqueObj<Request> _request;

    ICameraProvider *_iCameraProvider;
    ICaptureSession *_iCaptureSession;
    IEGLOutputStreamSettings *_iEglStreamSettings;
    IFrameConsumer *_iframeConsumer;
    ISourceSettings *_iSourceSettings;
    IAutoControlSettings *_iAutoControlSettings;

    float colour_correction_last_depth_band_update = -1;
    void updateColourCorrection();
};

#endif  // IMX_477_H_
