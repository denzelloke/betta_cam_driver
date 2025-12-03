// Copyright by BeeX [2024]
#include <h264_camera_driver/imx/imx_477_main.h>
using namespace Argus;
using namespace EGLStream;

/*@brief: Jetson camera API design -> Objects do not have methods.
 *  All methods are provided by Interfaces.
 *  Driver runs 1 camera based on the port given, compresses the image
 *  using encodeFromFD which is hardware accelerated and finally publish the image
 *  reference: http://on-demand.gputechconf.com/gtc/2016/webinar/getting-started-jetpack-camera-api.pdf
 */
Imx477Driver::Imx477Driver(
        const int cam_port_,
        const bool flip_image_,
        const std::string frame_id,
        const std::string ros_topic_root_name,
        std::string calibration_file_name)
        : CameraFeedWrapper(frame_id, ros_topic_root_name, calibration_file_name),
          cam_port(cam_port_),
          flip_image(flip_image_) {

    LOG_INFO("Starting Camera Driver.");
    printf("Camera FPS      : %d\n", CAMERA_FPS);
    printf("Camera Port     : %d\n", cam_port);
    printf("Flip Image      : %c\n", flip_image ? 'Y' : 'N');
    printf("Frame ID name   : %s\n", frame_id.c_str());

    // Jetson Related
    // Camera Provider provides entry point to libargus runtime
    // It provides methods for querying the cameras in the system and for creating camera devices
    _cameraProvider  = UniqueObj<CameraProvider>(CameraProvider::create());
    _iCameraProvider = interface_cast<ICameraProvider>(_cameraProvider);
    LOG_INFO("Created camera provider...");
    if (!_iCameraProvider) {
        LOG_INFO("Failed to Create cameraProvider.");
        return;
    }

    // CameraDevice Object representing a single camera device.
    _iCameraProvider->getCameraDevices(&_cameraDevices);
    LOG_INFO("%ld camera available...", _cameraDevices.size());
    if (_cameraDevices.size() == 0) {
        raiseFatal("No camera available");
        return;
    }

    // Get ICameraProperties interface
    // ICameraProperties *_iCameraProperties = interface_cast<ICameraProperties>(_cameraDevices[cam_port]);

    // Create capture session
    _captureSession  = UniqueObj<CaptureSession>(_iCameraProvider->createCaptureSession(_cameraDevices[cam_port]));
    _iCaptureSession = interface_cast<ICaptureSession>(_captureSession);
    LOG_INFO("Created iCaptureSession interface...");
    if (!_iCaptureSession) {
        raiseFatal("Failed to get iCaptureSession interface (%d)", cam_port);
        return;
    }

    // Container for settings used to configure/create an OutputStream.
    _streamSettings = UniqueObj<OutputStreamSettings>(_iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
    _iEglStreamSettings = interface_cast<IEGLOutputStreamSettings>(_streamSettings);
    LOG_INFO("Created iEGLOutputStreamSetting interface...");
    if (!_iEglStreamSettings) {
        raiseFatal("Failed to get iEGLOutputStreamSetting interface");
        return;
    }
    _iEglStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);

    // Set the stream resolution
    const Size2D<uint32_t> CAPTURE_SIZE(IMAGE_WIDTH_2K, IMAGE_HEIGHT_2K);
    _iEglStreamSettings->setResolution(CAPTURE_SIZE);
    _iEglStreamSettings->setMetadataEnable(true);

    // the application is expected to connect the consumer to this stream before the first
    // request is made otherwise the request will fail.
    _stream = UniqueObj<OutputStream>(_iCaptureSession->createOutputStream(_streamSettings.get()));
    IEGLOutputStream *_iEglOutputStream = interface_cast<IEGLOutputStream>(_stream.get());
    LOG_INFO("Got iEGLOutputStream...");
    if (!_iEglOutputStream) {
        raiseFatal("Failed to get iEGLOutputStream");
        return;
    }

    _frameConsumer  = UniqueObj<FrameConsumer>(FrameConsumer::create(_stream.get()));
    _iframeConsumer = interface_cast<IFrameConsumer>(_frameConsumer.get());
    LOG_INFO("Got iframeConsumer...");
    if (!_iframeConsumer) {
        raiseFatal("Failed to get iframeConsumer");
        return;
    }

    // Container for all settings used by a single capture request.
    _request            = UniqueObj<Request>(_iCaptureSession->createRequest());
    IRequest *_iRequest = interface_cast<IRequest>(_request);

    LOG_INFO("Got request interface...");
    if (!_iRequest) {
        raiseFatal("Failed to get request interface");
        return;
    }
    _iRequest->enableOutputStream(_stream.get());

    // Set to get new frame when buffer is empty
    _iCaptureSession->repeat(_request.get());

    // Sets the range of operation of the camera
    _iSourceSettings = interface_cast<ISourceSettings>(_iRequest->getSourceSettings());
    _iSourceSettings->setFrameDurationRange(Argus::Range<uint64_t>(1e9 / CAMERA_FPS));

    // Sets the control setting of the camera e.g. exposure, white balance...
    _iAutoControlSettings            = interface_cast<IAutoControlSettings>(_iRequest->getAutoControlSettings());
    Argus::Size2D<uint32_t> ccm_size = _iAutoControlSettings->getColorCorrectionMatrixSize();

    _iAutoControlSettings->setAwbLock(true);

    // Set the AE Anti-banding mode to 50Hz
    // autoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_OFF);
    _iAutoControlSettings->setAeAntibandingMode(Argus::AE_ANTIBANDING_MODE_OFF);

    // _iAutoControlSettings->setAeLock(true);
    // _iAutoControlSettings->setAwbMode(AWB_MODE_OFF);

    // _iAutoControlSettings->setColorSaturationEnable(false);
    // _iAutoControlSettings->setColorSaturation(0.8);


    // Denoising Enhancement
    LOG_INFO("Adding Denoise...");
    IDenoiseSettings *_iDenoiseSetting = interface_cast<IDenoiseSettings>(_request);
    if (!_iDenoiseSetting) {
        raiseFatal("Failed to get denoise interface");
        return;
    }
    _iDenoiseSetting->setDenoiseMode(Argus::DENOISE_MODE_HIGH_QUALITY);
    _iDenoiseSetting->setDenoiseStrength(0.75);

    // Edge Enhancement
    LOG_INFO("added edge enhancing...");
    IEdgeEnhanceSettings *_iEdgeEnhanceSetting = interface_cast<IEdgeEnhanceSettings>(_request);
    if (!_iEdgeEnhanceSetting) {
        raiseFatal("Failed to get edge enhancing interface");
        return;
    }
    _iEdgeEnhanceSetting->setEdgeEnhanceMode(Argus::EDGE_ENHANCE_MODE_HIGH_QUALITY);
    _iEdgeEnhanceSetting->setEdgeEnhanceStrength(0.9);

    // Print the camera exposure time
    LOG_INFO(
            "Exposure time: %.2f - %.2f ms\n",
            _iSourceSettings->getExposureTimeRange().min() / 1e6,
            _iSourceSettings->getExposureTimeRange().max() / 1e6);
}

/* @brief: Destructor, cleanS up buffer, thread and all sessions before exiting program */
Imx477Driver::~Imx477Driver() {

    _iCaptureSession->stopRepeat();
    _iCaptureSession->waitForIdle();
    _stream.reset();

    LOG_INFO("Closing Camera Driver.");
}

/* @brief: Main capture and publishing loop, where all the functions are called and run in loop for camera feed */

void Imx477Driver::captureAndPublishOnce(Srv_CamGetSnapshot_Response *res_ptr) {
    // From repeat mode - acquisition
    UniqueObj<Frame> frame(_iframeConsumer->acquireFrame());
    IFrame *iFrame = interface_cast<IFrame>(frame);

    if (iFrame) {
        NV::IImageNativeBuffer *iNativeBuffer = interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
        if (iNativeBuffer) {
            CameraFeedWrapper::publishH264Feed(iNativeBuffer, res_ptr);
        }
    }
}

void Imx477Driver::capture_n_pub() {
    LOG_INFO("Starting Capture & publish session..\r\n");
    RateLimiter rate_limiter(CAMERA_FPS);

    while (IS_ROS_NODE_OK()) {
        rate_limiter.sleep();
        ROS_SPIN_ONCE();

        // Frame acquisition altered  - Gets from repeat mode
        UniqueObj<Frame> frame(_iframeConsumer->acquireFrame());
        IFrame *iFrame = interface_cast<IFrame>(frame);

        if (iFrame) {
            NV::IImageNativeBuffer *iNativeBuffer = interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
            if (iNativeBuffer) {
                CameraFeedWrapper::publishH264Feed(iNativeBuffer, nullptr);
            }
        }
        // Update color correction
        // updateColourCorrection();
    }
    LOG_INFO("Closing session...");
}

/* @brief: Update the color correction settings */
void Imx477Driver::updateColourCorrection() {
    if (fabs(colour_correction_last_depth_band_update - CameraFeedWrapper::camera_depth) < 0.5) {
        return;
    }

    if (colour_correction_last_depth_band_update < -0.8) {
        colour_correction_last_depth_band_update = CameraFeedWrapper::camera_depth;
        printf("AWB call status: %d\n", _iAutoControlSettings->setAwbLock(true));

    } else {
        printf("Depth Band Changed: %.2fm -> %.2fm\n",
               colour_correction_last_depth_band_update,
               CameraFeedWrapper::camera_depth);

        colour_correction_last_depth_band_update = -1;
        printf("AWB call status: %d\n", _iAutoControlSettings->setAwbLock(false));
    }

    printf("AWB lock: %d\n", _iAutoControlSettings->getAwbLock());
}


DECLARE_ROS_NODE_HANDLE
int main(int argc, char *argv[]) {
    // ros::init(argc, argv, "ros_li_imx_477_cam_driver");
    INIT_ROS_NODE("imx_477_driver", 0, "imx_477_driver/alive")


    bool flip_image;
    int32_t cam_port;
    std::string frame_id, topic_name, calibration_file_name;

    GET_ROS_PARAM("~cam_port", cam_port, 0);
    GET_ROS_PARAM("~flip_image", flip_image, true);
    GET_ROS_PARAM("~frame_id", frame_id, "front_cam");
    GET_ROS_PARAM("~topic_name", topic_name, "/ikan/front_cam");
    GET_ROS_PARAM("~calibration_file_name", calibration_file_name, "calibration_imx185.yaml");

    std::string calibration_path = "/home/aibx/.xavier/calibrations/" + calibration_file_name;

    printf("Waiting for ROS Master to come back up\n");
    while (!IS_ROS_NODE_OK()) {
        printf(".");
        sleep(1);
    }
    printf("OK\n\nStarting Camera Driver.\n");

    Imx477Driver cam_driver(cam_port, flip_image, frame_id, topic_name, calibration_path);
    cam_driver.capture_n_pub();

    return 0;
}
