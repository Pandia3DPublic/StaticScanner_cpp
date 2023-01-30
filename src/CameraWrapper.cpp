#include "CameraWrapper.h"

// AzureKinect logging, see https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/group___logging.html
#define K4A_ENABLE_LOG_TO_STDOUT 1
#define K4A_LOG_LEVEL 'w' // c(ritical), e(rror), w(arning), i(nformation), t(race)

#include "util.h"
#include "open3d/3rdparty/turbojpeg/turbojpeg.h"
#include "PhoxiUtil.h"
using namespace std;
using namespace cv;

rs2::context CameraWrapper::IntelContext;
pho::api::PhoXiFactory CameraWrapper::PhoxiFactory;
std::vector<pho::api::PhoXiDeviceInformation> CameraWrapper::PhoxiDeviceList;
Zivid::Application CameraWrapper::ZividApp;
std::vector<Zivid::Camera> CameraWrapper::ZividCameraList;
NxLibItem CameraWrapper::EnsensoRoot;
NxLibItem CameraWrapper::EnsensoCameraList;

CameraWrapper::CameraWrapper(CameraTypes camtype)
{
    CameraType = camtype;
}


bool CameraWrapper::connect(int CameraNumber, string DataCamPath_)
{
    PandiaTimer timeout;
    int t_max = 5;
    if (CameraType == AzureKinect)
    {
        //connect to Azure Kinect
        while (!Connected && timeout.seconds() < t_max)
        {
            try
            {
                //CameraNumber is given in the main program to indicate which of multiple cameras must be opened.
                KinectDevice = KinectDevice.open(CameraNumber); //throws exception if there is no device
                SerialNumber = KinectDevice.get_serialnum();
                cout << "Azure Kinect device with id " << SerialNumber << " opened!" << endl;
                Connected = true;
            }
            catch (k4a::error e)
            {
                cerr << e.what() << endl;
                cout << "Retry Connecting Kinect Camera" << endl;
                this_thread::sleep_for(1000ms);
            }
        }
        if (!Connected)
        {
            return false;
        }
        //todo check if this config is the best
        KinectConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        KinectConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;  //note this takes processing power since some stupid mjpeg format is native.
        // KinectConfig.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;    //native color format, needs to be decompressed with turbojpeg for example
        KinectConfig.color_resolution = K4A_COLOR_RESOLUTION_1536P; //res is 2048x1536, this is the smallest 4:3 resolution which has max overlap with depth
        KinectConfig.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;    // binned res is 320x288
        // config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;     //res is 640x576 for K4A_DEPTH_MODE_NFOV_UNBINNED
        KinectConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
        KinectConfig.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
        KinectConfig.synchronized_images_only = true;

        KinectDevice.start_cameras(&KinectConfig);
        KinectDevice.start_imu();
        KinectCalibration = KinectDevice.get_calibration(KinectConfig.depth_mode, KinectConfig.color_resolution);
        KinectTransform = k4a::transformation(KinectCalibration); //this takes some time
        //get basic ldt intrinsic
        //TODO at the moment we are switching between depth to color and vice versa and undistort or no undistort. get a grip on this!
        // auto colorCalib = KinectCalibration.depth_camera_calibration;
        auto &ColorCalib = KinectCalibration.color_camera_calibration;
        auto &ColorParams = ColorCalib.intrinsics.parameters.param;
        FullCalibrationImageHeight = ColorCalib.resolution_height;
        FullCalibrationImageWidth = ColorCalib.resolution_width;
        ColorDistortionCoefficients = {ColorParams.k1, ColorParams.k2, ColorParams.p1, ColorParams.p2, ColorParams.k3, ColorParams.k4, ColorParams.k5, ColorParams.k6};
        ColorLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(ColorCalib.resolution_width, ColorCalib.resolution_height,
                                                                   ColorParams.fx, ColorParams.fy, ColorParams.cx, ColorParams.cy);
        CalibrationIntrinsic = ColorLDTIntrinsic;
        CalibrationCamDistortionCoeffs = ColorDistortionCoefficients;

        auto &DepthCalib = KinectCalibration.depth_camera_calibration;
        auto &DepthParams = DepthCalib.intrinsics.parameters.param;
        DepthDistortionCoefficients = {DepthParams.k1, DepthParams.k2, DepthParams.p1, DepthParams.p2, DepthParams.k3, DepthParams.k4, DepthParams.k5, DepthParams.k6};
        DepthLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(DepthCalib.resolution_width, DepthCalib.resolution_height,
                                                                   DepthParams.fx, DepthParams.fy, DepthParams.cx, DepthParams.cy);

        // Kinect undistortion look up table init
        PinholeDepth = create_pinhole_from_xy_range(&KinectCalibration, K4A_CALIBRATION_TYPE_DEPTH);
        DepthLDTIntrinsic.SetIntrinsics(PinholeDepth.width, PinholeDepth.height, PinholeDepth.fx, PinholeDepth.fy, PinholeDepth.px, PinholeDepth.py);
        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                         PinholeDepth.width,
                         PinholeDepth.height,
                         PinholeDepth.width * (int)sizeof(coordinate_t),
                         &LUTDepth);
        create_undistortion_lut(&KinectCalibration, K4A_CALIBRATION_TYPE_DEPTH, &PinholeDepth, LUTDepth, interpolation_type_depth);

        return true;
    }

    if (CameraType == IntelCamera)
    {
        rs2::log_to_console(rs2_log_severity::RS2_LOG_SEVERITY_ERROR);
        // rs2::log_to_console(rs2_log_severity::RS2_LOG_SEVERITY_WARN);
        while (!Connected && timeout.seconds() < t_max)
        {
            try
            {
                IntelDevice = IntelContext.query_devices()[CameraNumber];
                SerialNumber = IntelDevice.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                //set sensor and its start options
                for (auto s : IntelDevice.query_sensors())
                {
                    if (std::string(s.get_info(RS2_CAMERA_INFO_NAME)) == "RGB Camera")
                    {
                        IntelRGBSensor = s;
                        setIntelSensorOption(IntelRGBSensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, true);
                    }
                    if (std::string(s.get_info(RS2_CAMERA_INFO_NAME)) == "Stereo Module")
                    {
                        IntelDepthSensor = s;
                        setIntelSensorOption(IntelDepthSensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, true);
                        setIntelSensorOption(IntelDepthSensor, RS2_OPTION_EMITTER_ENABLED, true);
                    }
                }
                int width = 1280;
                int height = 720;
                IntelRGBProfile = getIntelStreamProfile(IntelRGBSensor, width, height, 30, RS2_FORMAT_RGB8, "Color");
                IntelDepthProfiles.push_back(getIntelStreamProfile(IntelDepthSensor, width, height, 30, RS2_FORMAT_Z16, "Depth"));
                IntelDepthProfiles.push_back(getIntelStreamProfile(IntelDepthSensor, width, height, 30, RS2_FORMAT_Y8, "Infrared 1")); //one should denote left ir cam
                cout << "Intel device with id " << SerialNumber << " connected!" << endl;
                Connected = true;
            }
            catch (rs2::error &e)
            {
                cerr << e.what() << endl;
                cout << "Retry Connecting Intel Camera" << endl;
                this_thread::sleep_for(1000ms);
                //todo maybe some stop commandos
            }
        }
        if (!Connected)
        {
            return false;
        }
        Align_to_depth = make_shared<rs2::align>(RS2_STREAM_DEPTH);
        Align_to_color = make_shared<rs2::align>(RS2_STREAM_COLOR);
        //set DepthLDTIntrinsic
        auto DepthIntr = IntelDepthProfiles[0].as<rs2::video_stream_profile>().get_intrinsics();
        DepthLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(DepthIntr.width, DepthIntr.height, DepthIntr.fx, DepthIntr.fy, DepthIntr.ppx, DepthIntr.ppy);
        auto ColorIntr = IntelRGBProfile.as<rs2::video_stream_profile>().get_intrinsics();
        ColorLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(ColorIntr.width, ColorIntr.height, ColorIntr.fx, ColorIntr.fy, ColorIntr.ppx, ColorIntr.ppy);
        auto InfraredIntr = IntelDepthProfiles[1].as<rs2::video_stream_profile>().get_intrinsics();
        CalibrationIntrinsic = open3d::camera::PinholeCameraIntrinsic(InfraredIntr.width, InfraredIntr.height, InfraredIntr.fx, InfraredIntr.fy, InfraredIntr.ppx, InfraredIntr.ppy);
        FullCalibrationImageWidth = InfraredIntr.width;
        FullCalibrationImageHeight = InfraredIntr.height;
        //TODO
        if (ColorIntr.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY && DepthIntr.model == RS2_DISTORTION_BROWN_CONRADY) 
        {
            // we need the unmodified brown conrady distortion coefficients for opencv, therefore undo the inversion
            // however this yields worse calibration errors than original parameters somehow
            // maybe realsense color distortion is actually pincushion?? k1 > 0 == Barell distorion, k1 < 0 == pincushion.
            // see https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4934233/ and http://www.aensiweb.net/AENSIWEB/anas/anas/2017/March/86-90.pdf
            auto cc = ColorIntr.coeffs;
            auto dc = DepthIntr.coeffs;
            // float b1 = cc[0];
            // float b2 = cc[1];
            // float b3 = cc[4];
            // float k1 = -b1;
            // float k2 = 3*k1*k1 - b2;
            // float k3 = -12*k1*k1*k1 + 8*k1*k2 - b3;
            // ColorDistortionCoefficients = {k1, k2, cc[2], cc[3], k3};
            // ColorDistortionCoefficients = {0,0,0,0,0};
            ColorDistortionCoefficients = {cc[0], cc[1], cc[2], cc[3], cc[4]};
            DepthDistortionCoefficients = {dc[0], dc[1], dc[2], dc[3], dc[4]};
            CalibrationCamDistortionCoeffs = {0,0,0,0,0}; //Intel has no distortion values for infrared
        }
        else if (ColorIntr.model == RS2_DISTORTION_BROWN_CONRADY && DepthIntr.model == RS2_DISTORTION_BROWN_CONRADY)
        {
            auto cc = ColorIntr.coeffs;
            auto dc = DepthIntr.coeffs;
            ColorDistortionCoefficients = {cc[0], cc[1], cc[2], cc[3], cc[4]};
            DepthDistortionCoefficients = {dc[0], dc[1], dc[2], dc[3], dc[4]};
        }
        else 
        {
            ColorDistortionCoefficients = {0, 0, 0, 0, 0, 0, 0, 0};
            DepthDistortionCoefficients = {0, 0, 0, 0, 0, 0, 0, 0};
            cout << "Warning: Realsense color and depth distortion model currently not supported!" << endl;
        }
        return true;
    }

    if (CameraType == DataCamera)
    {
        if (DataCamPath_.length() == 0)
        {
            cout << "Warning : You need to supply a path if the camera type is set to DataCam! \n";
        }
        else
        {
            DataCamPath = DataCamPath_;
            cout << "Harddrive intrinsic path is " << DataCamPath + "intrinsic.txt" << endl;
            if (fileExists(DataCamPath + "intrinsic.txt"))
            {
                setFromIntrinsicFile(DataCamPath + "intrinsic.txt", DepthLDTIntrinsic);
                return true;
            }
            else
            {
                cout << "Warning no intrinsic file found. Data is not valid! \n";
            }
        }
    }

    if (CameraType == PhoxiScanner)
    {
        //Try to connect device opened in PhoXi Control, if any
        std::cout << "Trying to connect to " << PhoxiDeviceList.at(CameraNumber).HWIdentification << std::endl;
        PhoxiDevice = PhoxiFactory.CreateAndConnect(PhoxiDeviceList.at(CameraNumber).HWIdentification, 5000); //5s timeout
        if (PhoxiDevice && PhoxiDevice->isConnected())
        {
            std::cout << "successfully connected" << std::endl;
            SerialNumber = PhoxiDeviceList.at(CameraNumber).HWIdentification;
            Connected = true;
        }
        else
        {
            std::cout << "Can not connect to device" << std::endl;
            return false;
        }
        if (!Connected)
        {
            return false;
        }
        if (PhoxiDevice->Profiles.isEnabled())
        {
            std::vector<pho::api::PhoXiProfileDescriptor> ProfilesList = PhoxiDevice->Profiles;
            if (!PhoxiDevice->Profiles.isLastOperationSuccessful())
            {
                std::cout << "Can not get profile list: " << PhoxiDevice->Profiles.GetLastErrorMessage() << std::endl;
                return false;
            }
            // Available profiles: DEFAULT FAST_SCANNING MARKER_SPACE_WITH_ROI NOISE_FILTERING_FOR_SHINY_MATERIAL STRONG_AMBIENT_LIGHT
            std::string NewActiveProfile = "DEFAULT";
            for (const pho::api::PhoXiProfileDescriptor &Profile : ProfilesList)
            {
                if (Profile.Name == NewActiveProfile)
                {
                    PhoxiDevice->ActiveProfile = NewActiveProfile;
                    if (!PhoxiDevice->ActiveProfile.isLastOperationSuccessful())
                    {
                        std::cout << "Can not set active profile: " << PhoxiDevice->ActiveProfile.GetLastErrorMessage() << std::endl;
                        return false;
                    }
                    break;
                }
            }
        }
        if (PhoxiDevice->CapturingSettings.isEnabled())
        {
            pho::api::PhoXiCapturingSettings NewCapturingSettings = PhoxiDevice->CapturingSettings;
            if (!PhoxiDevice->CapturingSettings.isLastOperationSuccessful())
            {
                std::cout << "Can not get capturing settings: " << PhoxiDevice->CapturingSettings.GetLastErrorMessage() << std::endl;
                return false;
            }
            NewCapturingSettings.CodingQuality = pho::api::PhoXiCodingQuality::Ultra;
            PhoxiDevice->CapturingSettings = NewCapturingSettings;
            if (!PhoxiDevice->CapturingSettings.isLastOperationSuccessful())
            {
                std::cout << "Can not set new capturing settings: " << PhoxiDevice->CapturingSettings.GetLastErrorMessage() << std::endl;
                return false;
            }
        }

        auto& Calibration = PhoxiDevice->CalibrationSettings;
        PrintCalibrationSettings(Calibration);
        auto DepthIntr = Calibration->CameraMatrix;
        DepthLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(PhoxiDevice->Resolution->Width, PhoxiDevice->Resolution->Height,
                                                                   DepthIntr.At(0, 0), DepthIntr.At(1, 1), DepthIntr.At(0, 2), DepthIntr.At(1, 2));
        ColorLDTIntrinsic = DepthLDTIntrinsic; //todo
        CalibrationIntrinsic = DepthLDTIntrinsic; //todo
        FullCalibrationImageWidth = PhoxiDevice->Resolution->Width;
        FullCalibrationImageHeight = PhoxiDevice->Resolution->Height;
        // DepthDistortionCoefficients = std::vector<float>(Calibration->DistortionCoefficients.begin(), Calibration->DistortionCoefficients.end());
        // DepthDistortionCoefficients.resize(5); //only first 5 as others are zero
        // ColorDistortionCoefficients = DepthDistortionCoefficients;
        // CalibrationCamDistortionCoeffs = DepthDistortionCoefficients;
        ColorDistortionCoefficients = {0, 0, 0, 0, 0};
        DepthDistortionCoefficients = {0, 0, 0, 0, 0};
        CalibrationCamDistortionCoeffs = {0, 0, 0, 0, 0};

        PhoxiDevice->StartAcquisition();
        if (!PhoxiDevice->isAcquiring())
        {
            std::cout << "Your device could not start acquisition!" << std::endl;
            return false;
        }
        PhoxiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
        return true;
    }
    if (CameraType == ZividCamera)
    {
        try
        {
            cout << "Trying to connect to " << ZividCameraList.at(CameraNumber).info() << endl;
            ZividCam = ZividCameraList.at(CameraNumber).connect();
            if (ZividCam.state().isConnected())
            {
                cout << "Successfully connected!" << endl;
                Connected = true;
                SerialNumber = ZividCam.info().serialNumber().value();
            }
            else
            {
                cout << "Failed to connect to device..." << endl;
                return false;
            }

            const auto suggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
                Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
                Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{std::chrono::milliseconds{1200}}};

            std::cout << "Running Capture Assistant with parameters:\n" << suggestSettingsParameters << std::endl;
            ZividSettings = Zivid::CaptureAssistant::suggestSettings(ZividCam, suggestSettingsParameters);

            std::cout << "Settings suggested by Capture Assistant:" << std::endl;
            std::cout << ZividSettings.acquisitions() << std::endl;

            std::cout << "Manually configuring processing settings (Capture Assistant only suggests acquisition settings)" << std::endl;
            const auto BasicProcessing =
                Zivid::Settings::Processing{Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
                                            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                                            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{1.5}};
            ZividSettings.set(BasicProcessing);

            // use stripe engine for reflective objects, this overwrites BasicProcessing settings
            bool useStripeEngine = false;
            if (useStripeEngine)
            {
                cout << "USING STRIPE ENGINE" << endl;
                ZividSettings.set(Zivid::Settings::Experimental::Engine::stripe);
                const auto AdvancedProcessing = Zivid::Settings::Processing{
                    Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                    Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{1.5},
                    Zivid::Settings::Processing::Filters::Noise::Removal::Enabled::yes,
                    Zivid::Settings::Processing::Filters::Noise::Removal::Threshold{10.0}, //7
                    Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
                    Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{7.0}, //5
                    Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
                    Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
                    Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{0.4},
                    Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
                    Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold{0.5},
                    Zivid::Settings::Processing::Color::Balance::Red{1.0},
                    Zivid::Settings::Processing::Color::Balance::Green{1.0},
                    Zivid::Settings::Processing::Color::Balance::Blue{1.0},
                    Zivid::Settings::Processing::Color::Gamma{1.0},
                    Zivid::Settings::Processing::Color::Experimental::ToneMapping::Enabled::hdrOnly};
                ZividSettings.set(AdvancedProcessing);
            }
            // cout << "All Zivid Settings used: \n" << ZividSettings << endl;

            auto intrinsics = Zivid::Experimental::Calibration::intrinsics(ZividCam);
            cout << intrinsics << endl;
            auto resolution = Zivid::Experimental::SettingsInfo::resolution(ZividCam.info(), ZividSettings);
            auto intr = intrinsics.cameraMatrix();
            auto dc = intrinsics.distortion();
            DepthLDTIntrinsic = open3d::camera::PinholeCameraIntrinsic(resolution.width(), resolution.height(), intr.fx().value(), intr.fy().value(), intr.cx().value(), intr.cy().value());
            ColorLDTIntrinsic = DepthLDTIntrinsic;
            CalibrationIntrinsic = DepthLDTIntrinsic;
            FullCalibrationImageWidth = resolution.width();
            FullCalibrationImageHeight = resolution.height();
            // ColorDistortionCoefficients = {(float)dc.k1().value(), (float)dc.k2().value(), (float)dc.p1().value(), (float)dc.p2().value(), (float)dc.k3().value()};
            ColorDistortionCoefficients = {0,0,0,0,0}; // todo this yields more exact gravity vector with zivid's pcd (which should be undistorted)
                                                        //however we calibrate with rgb images.
                                                        //rgb images are clearly distorted, depth seems too but is hard to see.
            DepthDistortionCoefficients = ColorDistortionCoefficients;
            CalibrationCamDistortionCoeffs = ColorDistortionCoefficients;

            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in connect: " << Zivid::toString(e) << std::endl;
            return false;
        }
    }
    if (CameraType == EnsensoCamera)
    {
        try
        {
            EnsensoCam = EnsensoCameraList[CameraNumber];
            if (EnsensoCam.exists() && EnsensoCam["Status"]["Available"] == true)
            {
                SerialNumber = EnsensoCam[itmSerialNumber].asString();
                cout << "Opening Camera " << SerialNumber << endl;
                NxLibCommand open(cmdOpen); // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
                open.parameters()[itmCameras] = SerialNumber; // Set parameters for the open command
                open.execute();
                cout << "Successfully connected!" << endl;
                Connected = true;
            }
            else
            {
                cout << "Connect failed as device is busy...\n";
                return false;
            }

            // cam settings
            EnsensoSettings = EnsensoCam[itmParameters][itmCapture];
            EnsensoSettings[itmAutoExposure] = true;
            EnsensoSettings[itmAutoGain] = false;
            // EnsensoSettings[itmExposure] = 0.001;
            // EnsensoSettings[itmBinning] = 1;
            // EnsensoSettings[itmPixelClock] = 36;
            EnsensoSettings[itmTriggerMode] = valSoftware; // valContinious
            // EnsensoSettings[itmTriggerDelay] = 60;
            EnsensoSettings[itmProjector] = true;

            // Capture one image to get frame info
            NxLibCommand(cmdCapture, EnsensoCam.name()).execute();      // Without parameters, most commands just operate on all open cameras
            NxLibCommand(cmdRectifyImages, EnsensoCam.name()).execute();
            NxLibCommand(cmdComputeDisparityMap, EnsensoCam.name()).execute();
            NxLibCommand(cmdComputePointMap, EnsensoCam.name()).execute();

            // width, height. ensenso s10 pointmap is 364x272, raw and rectified imgs are 1456x1088
            int pointMapWidth, pointMapHeight;
            EnsensoCam[itmImages][itmRectified].getBinaryDataInfo(&FullCalibrationImageWidth, &FullCalibrationImageHeight, 0, 0, 0, 0);
            EnsensoCam[itmImages][itmPointMap].getBinaryDataInfo(&pointMapWidth, &pointMapHeight, 0, 0, 0, 0);

            // get intrinsics from node tree
            int32_t* treePtr;
            std::string calibrationPath = "/Cameras/BySerialNo/" + SerialNumber + "/Calibration/";
            auto calibStr = nxLibGetJson(treePtr, calibrationPath.c_str(), 0, 15, 0);
            Json::Value calibJson = StringToJson(calibStr);
            auto intrMat = calibJson["Dynamic"]["Stereo"]["Left"]["Camera"];
            // auto dc = calibJson["Monocular"]["Left"]["Distortion"]; //unclear so don't use them

            CalibrationIntrinsic = open3d::camera::PinholeCameraIntrinsic(FullCalibrationImageWidth, FullCalibrationImageHeight, 
                                    intrMat[0][0].asDouble(), intrMat[1][1].asDouble(), intrMat[2][0].asDouble(), intrMat[2][1].asDouble());
            DepthLDTIntrinsic = getScaledIntrinsic(CalibrationIntrinsic, pointMapWidth, pointMapHeight);
            ColorLDTIntrinsic = DepthLDTIntrinsic;
            CalibrationCamDistortionCoeffs = {0, 0, 0, 0, 0}; // we use rectified images (undistorted), as distortion coeffs are unclear
            ColorDistortionCoefficients = CalibrationCamDistortionCoeffs;
            DepthDistortionCoefficients = ColorDistortionCoefficients;

            return true;
        }
        catch (NxLibException& ex)
        {
            cout << "Error: Connect failed " << ex.getItemPath() << " " << ex.getErrorText() << endl;
        }
    }
    return false;
}

// stop streaming, usb reset camera, start streaming after reset if wanted
// called when receving bad depth data
bool CameraWrapper::resetConnection(bool streamAfterReset /* = true*/)
{
    bool resetted = false;
    if (CameraType == IntelCamera)
    {
        PandiaTimer timeout;
        int t_max = 15;
        while (!resetted && timeout.seconds() < t_max)
        {
            try
            {
                cout << "Resetting camera " << SerialNumber << endl;
                stopStreaming();
                // intel realsense sdk doensn't have a reliable method to check if a camera is ready again after a hardware_reset
                // we should therefore wait a painful 10s before attemting to start the sensors again
                // see https://github.com/IntelRealSense/librealsense/issues/9287#issuecomment-872606987
                IntelDevice.hardware_reset(); //reset usb connection
                int n_wait = 0;
                while (n_wait++ < 10)
                {
                    cout << "waiting for usb reconnect (" << n_wait << "s / 10s)" << endl;
                    std::this_thread::sleep_for(1s);
                }
                rs2::device_hub IntelHub(IntelContext);
                while (!resetted && timeout.seconds() < t_max)
                {
                    cout << "searching device..." << endl;
                    rs2::device device = IntelHub.wait_for_device(); //Note that device hub will get any device, if you have more than 1 connected it could return the other device
                    if (std::string(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) == SerialNumber)
                    {
                        cout << "found device!" << endl;
                        // hardware reset should invalidate device handles but sensors apparantly still work after????
                        // device and sensors are all shared_ptr so setting them again probably does nothing as reference count 
                        // is always > 0 while CameraWrapper instance is alive

                        IntelDevice = device;
                        resetted = true;
                        break;
                    }
                }
                if (streamAfterReset)
                {
                    startStreaming();
                }
                cout << "Done resetting camera " << SerialNumber << endl;
                break;
            }
            catch (rs2::error &e)
            {
                cerr << e.what() << endl;
                cout << "Reset connection failed!" << endl;
                this_thread::sleep_for(1000ms);
            }
        }
    }
    else
    {
        cout << "Info: Reset not implemented for this cam type." << endl;
    }
    return resetted;
}

//returns true if new frames were recorded and pushed back
bool CameraWrapper::record(int n, int n_wait, bool checkDepthImage /* = true*/)
{
    //1. Wait for auto-exposure
    //2. Record images
    int nFramesOld = 0;
    if (CameraType == AzureKinect)
    {
        nFramesOld = KinectCaptures.size();
        k4a::capture warmupCapture;
        for (int i = 0; i < n_wait; i++)
        {
            KinectDevice.get_capture(&warmupCapture); //dropping several frames for auto-exposure
        }
        if (KinectConfig.color_format == K4A_IMAGE_FORMAT_COLOR_BGRA32)
        {
            for (int i = 0; i < n; i++)
            {
                shared_ptr<k4a::capture> listCapture = make_shared<k4a::capture>();
                KinectDevice.get_capture(listCapture.get());
                KinectCaptures.push_back(listCapture);
            }
        }
        else if (KinectConfig.color_format == K4A_IMAGE_FORMAT_COLOR_MJPG)
        {
            for (int i = 0; i < n; i++)
            {
                shared_ptr<k4a::capture> listCapture = make_shared<k4a::capture>();
                KinectDevice.get_capture(listCapture.get());
                k4a::image color = listCapture->get_color_image();
                int width = color.get_width_pixels();
                int height = color.get_height_pixels();
                k4a::image colorDecompressed = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * (int)sizeof(uint32_t));
                tjhandle tjHandle;
                tjHandle = tjInitDecompress();
                // note: decompress takes around 22ms
                // azure kinect sdk also uses turbojpeg to decode to bgra, so doing this ourselves only adds some more control
                auto res = tjDecompress2(tjHandle, color.get_buffer(),
                                         static_cast<unsigned long>(color.get_size()),
                                         colorDecompressed.get_buffer(), width, 0 /* pitch */, height,
                                         TJPF_BGRA, TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);
                if (res == 0)
                {
                    listCapture->set_color_image(colorDecompressed);
                    KinectCaptures.push_back(listCapture);
                }
                else
                {
                    cout << "WARNING: Failed to decompress k4a color image.. discarding capture, recording one more" << endl;
                    n++;
                }
                tjDestroy(tjHandle);
            }
        }
        else
        {
            cout << "WARNING: Unsupported K4A color image format in record" << endl;
        }
        return KinectCaptures.size() > nFramesOld ? true : false;
    }

    if (CameraType == IntelCamera)
    {
        startStreaming();
        nFramesOld = rgbdImages.size();
        int nReset = 0; //number of resets
        int nResetMax = 1;
        for (int i = 0; i < n + nReset; i++)
        {
            try
            {
                PandiaTimer framesetTimer;
                bool validFrameset = false;
                rs2::frameset fs;
                while (!validFrameset && framesetTimer.seconds() < 5)
                {
                    fs = IntelSyncer.wait_for_frames(); // does not copy, only adds a reference
                    validFrameset = IsIntelFramesetValid(fs);
                    // if (!validFrameset)
                    //     cout << "Warning: Frameset of camera " << SerialNumber << " is invalid. Recording another..." << endl;
                }
                if (!validFrameset)
                    cout << "Warning: Could not get a valid frameset for camera " << SerialNumber << endl;
                //############ processing ###################
                //Note: Processsing must happen here due to stupid intel pipeline memory management
                //need to take color here since align to depth changes color
                auto color_frame_full = fs.get_color_frame();
                auto cvColor = FrametoMat(color_frame_full);
                auto infrared_frame = fs.get_infrared_frame(1); //left infrared image

                FRCalibrationImages.push_back(FrametoMat(infrared_frame).clone());
                auto alignedFS = Align_to_depth->process(fs);
                auto aligned_color_frame = alignedFS.get_color_frame();
                auto depth_frame = alignedFS.get_depth_frame();

                //############## apply filters, order is important ####################
                depth_frame = Thr_filter.process(depth_frame);
                depth_frame = Depth_to_disparity.process(depth_frame);
                depth_frame = Spat_filter.process(depth_frame); //spatial filter does hole filling, which we should not do!
                depth_frame = Disparity_to_depth.process(depth_frame);
                auto cvDepth = FrametoMatMillimeters(depth_frame);
                //############### convert to open3d ############################
                auto cvColorAligned = FrametoMat(aligned_color_frame);
                std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
                rgbdImg->color_.Prepare(cvColorAligned.cols, cvColorAligned.rows, 3, 1);
                rgbdImg->depth_.Prepare(cvDepth.cols, cvDepth.rows, 1, 2);
                if (cvColor.isContinuous() && cvDepth.isContinuous())
                {
                    memcpy(rgbdImg->color_.data_.data(), cvColorAligned.data, cvColorAligned.total() * cvColorAligned.elemSize());
                    memcpy(rgbdImg->depth_.data_.data(), cvDepth.data, cvDepth.total() * cvDepth.elemSize());
                }
                else
                {
                    cout << "Warning opencv images are not continous! \n";
                }
                rgbdImg->depth_ = *rgbdImg->depth_.ConvertDepthToFloatImage();
                //################ check depth image ############################
                if (checkDepthImage)
                {
                    if (IsDepthImageValid(rgbdImg->depth_))
                    {
                        rgbdImages.push_back(rgbdImg);
                    }
                    else
                    {
                        // open3d::visualization::DrawGeometries({make_shared<open3d::geometry::Image>(rgbdImg->depth_)});
                        cout << "Warning: Depth frame of camera " << SerialNumber << " is invalid!" << endl;
                        if (nReset < nResetMax)
                        {
                            cout << "Resetting camera then recording again..." << endl;
                            resetConnection(true);
                            nReset++;
                        }
                    }
                }
                else
                {
                    rgbdImages.push_back(rgbdImg);
                }
            }
            catch (const rs2::error& e)
            {
                std::cerr << e.what() << '\n';
                cout << "Error: Recording or processing frames " << i << " of cam " << SerialNumber << " failed." << endl;
                if (nReset < nResetMax)
                {
                    cout << "Resetting camera then recording again..." << endl;
                    resetConnection(true);
                    nReset++;
                }
            }
        }
        stopStreaming();
        return rgbdImages.size() > nFramesOld ? true : false;
    }

    if (CameraType == DataCamera)
    {
        nFramesOld = rgbdImages.size();
        for (int i = 0; i < n; i++)
        {
            //1.build file path
            //2.read file from filepath
            string filePathColor = DataCamPath + "color/" + getPicNumberString(i) + ".png";
            string filePathDepth = DataCamPath + "depth/" + getPicNumberString(i) + ".png";
            std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
            open3d::io::ReadImage(filePathColor, rgbdImg->color_);
            open3d::io::ReadImage(filePathDepth, rgbdImg->depth_);
            rgbdImages.push_back(rgbdImg);
        }
        return rgbdImages.size() > nFramesOld ? true : false;
    }

    if (CameraType == PhoxiScanner)
    {
        nFramesOld = rgbdImages.size();
        PhoxiDevice->ClearBuffer();
        for (int i = 0; i < n; ++i)
        {
            try
            {
                std::cout << "Triggering the " << i << "-th frame" << std::endl;
                int FrameID = PhoxiDevice->TriggerFrame();
                if (FrameID < 0)
                {
                    // If negative number is returned trigger was unsuccessful
                    std::cout << "Trigger was unsuccessful!" << std::endl;
                    continue;
                }
                else
                {
                    std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
                }

                std::cout << "Waiting for frame " << i << std::endl;
                pho::api::PFrame Frame = PhoxiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
                if (!Frame || Frame->Empty())
                {
                    std::cout << "Failed to retrieve the frame!" << std::endl;
                    continue;
                }
                printFrameInfo(Frame);
                printFrameData(Frame);

                //images to opencv
                cv::Mat cvGrayRaw;
                cv::Mat cvDepth;
                Frame->Texture.ConvertTo(cvGrayRaw); //distorted
                Frame->DepthMap.ConvertTo(cvDepth);  //distorted
                double minVal;
                double maxVal;
                minMaxLoc(cvGrayRaw, &minVal, &maxVal);
                cv::Mat cvGray = cv::Mat(cvGrayRaw.rows, cvGrayRaw.cols, CV_8UC1);
                for (int i = 0; i < cvGrayRaw.rows; i++)
                {
                    for (int j = 0; j < cvGrayRaw.cols; j++)
                    {
                        float tmp = cvGrayRaw.at<float>(i, j) / maxVal * 255;
                        cvGray.at<uint8_t>(i, j) = (uint8_t)tmp;
                    }
                }
                cv::equalizeHist(cvGray, cvGray);
                cv::Mat cvColor = cv::Mat(cvGrayRaw.rows, cvGrayRaw.cols, CV_8UC3);
                cv::cvtColor(cvGray, cvColor, COLOR_GRAY2RGB);
                FRCalibrationImages.push_back(cvColor.clone());

                // phoxi pcd
                cv::Mat cvPcd;
                cv::Mat cvNormals;
                Frame->PointCloud.ConvertTo(cvPcd); //undistorted
                Frame->NormalMap.ConvertTo(cvNormals);
                auto PhoxiPcd = make_shared<open3d::geometry::PointCloud>();
                for (int i = 0; i < cvPcd.rows; i++)
                {
                    for (int j = 0; j < cvPcd.cols; j++)
                    {
                        Eigen::Vector3d p = Eigen::Vector3d(cvPcd.at<cv::Point3f>(i, j).x, cvPcd.at<cv::Point3f>(i, j).y, cvPcd.at<cv::Point3f>(i, j).z);
                        p /= 1000;
                        Eigen::Vector3d n = Eigen::Vector3d(cvNormals.at<cv::Point3f>(i, j).x, cvNormals.at<cv::Point3f>(i, j).y, cvNormals.at<cv::Point3f>(i, j).z);
                        n.normalize();
                        Eigen::Vector3d rgb = Eigen::Vector3d(cvColor.at<cv::Vec3b>(i, j)(0), cvColor.at<cv::Vec3b>(i, j)(1), cvColor.at<cv::Vec3b>(i, j)(2));
                        rgb /= 255;
                        if (p != Eigen::Vector3d::Zero() && n != Eigen::Vector3d::Zero())
                        {
                            PhoxiPcd->points_.push_back(p);
                            PhoxiPcd->normals_.push_back(n);
                            PhoxiPcd->colors_.push_back(rgb);
                        }
                    }
                }
                // open3d rgbd image
                std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
                rgbdImg->color_.Prepare(cvColor.cols, cvColor.rows, cvColor.channels(), cvColor.elemSize() / cvColor.channels());
                rgbdImg->depth_.Prepare(cvDepth.cols, cvDepth.rows, cvDepth.channels(), cvDepth.elemSize() / cvDepth.channels());
                if (cvColor.isContinuous() && cvDepth.isContinuous())
                {
                    memcpy(rgbdImg->color_.data_.data(), cvColor.data, cvColor.total() * cvColor.elemSize());
                    memcpy(rgbdImg->depth_.data_.data(), cvDepth.data, cvDepth.total() * cvDepth.elemSize());
                }
                else
                {
                    cout << "Warning opencv images are not continous! \n";
                }
                rgbdImg->depth_ = *rgbdImg->depth_.ConvertDepthToFloatImage();

                // lastly push back pcd and rgbdImg
                Pcds.push_back(PhoxiPcd);
                rgbdImages.push_back(rgbdImg);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error in record: " << e.what() << std::endl;
            }
        }
        // PhoxiDevice->StopAcquisition();
        return rgbdImages.size() > nFramesOld ? true : false;
    }
    if (CameraType == ZividCamera)
    {
        nFramesOld = rgbdImages.size();
        for (int i = 0; i < n; ++i)
        {
            try
            {
                auto frame = ZividCam.capture(ZividSettings);
                auto pcd = frame.pointCloud();
                if (pcd.isEmpty())
                {
                    cout << "Warning: Zivid Pcd is empty" << endl;
                    continue;
                }
                auto rgba = pcd.copyImageRGBA();
                // The cast for colors.data() is required because the cv::Mat constructor requires non-const void *.
                // It does not actually mutate the data, it only adds an OpenCV header to the matrix. We then protect
                // our own instance with const.
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
                auto cvRGBA = cv::Mat(rgba.height(), rgba.width(), CV_8UC4, const_cast<void *>(static_cast<const void *>(rgba.data()))); //wrapping data in opencv matrix, no copying occurs here
                cv::Mat cvColor;
                cv::cvtColor(cvRGBA, cvColor, cv::COLOR_RGBA2RGB);
                FRCalibrationImages.push_back(cvColor.clone());

                //note: invalid values in depth and cvDepth are nan, not 0
                auto depth = pcd.copyPointsZ();
                auto cvDepth = cv::Mat(depth.height(), depth.width(), CV_32F, const_cast<void *>(static_cast<const void *>(depth.data())));

                // camera pcd (undistorted)
                auto pcdPoints = pcd.copyPointsXYZ();
                auto pcdNormals = pcd.copyNormalsXYZ();
                auto ZividPcd = make_shared<open3d::geometry::PointCloud>();
                for (int i = 0; i < pcdPoints.height(); i++)
                {
                    for (int j = 0; j < pcdPoints.width(); j++)
                    {
                        if (!pcdPoints(i, j).isNaN() && !pcdNormals(i, j).isNaN())
                        {
                            Eigen::Vector3d p = Eigen::Vector3d(pcdPoints(i, j).x, pcdPoints(i, j).y, pcdPoints(i, j).z);
                            p /= 1000;
                            Eigen::Vector3d n = Eigen::Vector3d(pcdNormals(i, j).x, pcdNormals(i, j).y, pcdNormals(i, j).z);
                            n.normalize();
                            Eigen::Vector3d rgb = Eigen::Vector3d(cvColor.at<cv::Vec3b>(i, j)(0), cvColor.at<cv::Vec3b>(i, j)(1), cvColor.at<cv::Vec3b>(i, j)(2));
                            rgb /= 255;
                            ZividPcd->points_.push_back(p);
                            ZividPcd->normals_.push_back(n);
                            ZividPcd->colors_.push_back(rgb);
                        }
                    }
                }

                // convert to open3d
                std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
                rgbdImg->color_.Prepare(cvColor.cols, cvColor.rows, cvColor.channels(), cvColor.elemSize() / cvColor.channels());
                rgbdImg->depth_.Prepare(cvDepth.cols, cvDepth.rows, cvDepth.channels(), cvDepth.elemSize() / cvDepth.channels());
                if (cvColor.isContinuous() && cvDepth.isContinuous())
                {
                    memcpy(rgbdImg->color_.data_.data(), cvColor.data, cvColor.total() * cvColor.elemSize());
                    memcpy(rgbdImg->depth_.data_.data(), cvDepth.data, cvDepth.total() * cvDepth.elemSize());
                }
                else
                {
                    cout << "Warning opencv images are not continous! \n";
                }
                rgbdImg->depth_ = *rgbdImg->depth_.ConvertDepthToFloatImage();

                // lastly push back pcd and rgbdImg
                Pcds.push_back(ZividPcd);
                rgbdImages.push_back(rgbdImg);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error in record: " << Zivid::toString(e) << std::endl;
            }
        }
        return rgbdImages.size() > nFramesOld ? true : false;
    }
    if (CameraType == EnsensoCamera)
    {
        // note: there are raw and rectified images (color), as well as a pointMap (pcd)
        // raw is distorted, rectified is undistorted and they have the same dimensions
        // pointmap has smaller dimensions than images, thus the depth image is smaller too
        // we use full resolution rectified images for calibration and resize them to pointmap dimensions for rgbd image
        // we ignore distorted raw images as distortion coeffs are unclear and not well documented by ensenso
        nFramesOld = rgbdImages.size();
        for (int i = 0; i < n_wait; i++)
        {
            try
            {
                NxLibCommand(cmdCapture, EnsensoCam.name()).execute(); // Without parameters, most commands just operate on all open cameras
            }
            catch (NxLibException &ex)
            {
                cout << "Error recording warmup frame " << i << ": " << ex.getItemPath() << " " << ex.getErrorText() << endl;
            }
        }
        int nFail = 0;
        int nFailMax = 5;
        for (int i = 0; i < n + nFail; i++)
        {
            try
            {
                // Execute the 'Capture', 'ComputeDisparityMap' and 'ComputePointMap' commands
                NxLibCommand(cmdCapture, EnsensoCam.name()).execute(); // Without parameters, most commands just operate on all open cameras
                NxLibCommand(cmdRectifyImages, EnsensoCam.name()).execute();
                NxLibCommand(cmdComputeDisparityMap, EnsensoCam.name()).execute();
                NxLibCommand(cmdComputePointMap, EnsensoCam.name()).execute();

                int width, height, channels, bytesPerElement;
                EnsensoCam[itmImages][itmRectified].getBinaryDataInfo(&width, &height, &channels, &bytesPerElement, 0, 0);
                cv::Mat cvColorFR; //undistorted
                if (channels == 1 && bytesPerElement == 1)
                {
                    std::vector<uint8_t> colorData;
                    EnsensoCam[itmImages][itmRectified].getBinaryData(colorData, 0);
                    cvColorFR = cv::Mat(height, width, CV_8UC1, colorData.data()).clone();
                    cv::cvtColor(cvColorFR, cvColorFR, cv::ColorConversionCodes::COLOR_GRAY2RGB);
                }
                else
                {
                    cout << "WARNING: Unexpected ensenso color image type." << endl;
                }
                FRCalibrationImages.push_back(cvColorFR);

                // depth image, color resize and camera pcd
                // NOTE for Ensenso S10: pointmap is 364x272, raw and rectified imgs are 1456x1088
                std::vector<float> pointMap;
                EnsensoCam[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, &channels, &bytesPerElement, 0, 0);
                EnsensoCam[itmImages][itmPointMap].getBinaryData(pointMap, 0);
                cv::Mat cvColor;
                cv::resize(cvColorFR, cvColor, cv::Size(width, height), 0, 0, cv::InterpolationFlags::INTER_LINEAR); //resize to depth dimensions
                cv::Mat cvDepth = cv::Mat(height, width, CV_32F, Scalar(0));
                auto CamPcd = make_shared<open3d::geometry::PointCloud>();
                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        // Get X,Y,Z coordinates of the point at image pixel (x,y)
                        float px = pointMap[(y * width + x) * 3];
                        float py = pointMap[(y * width + x) * 3 + 1];
                        float pz = pointMap[(y * width + x) * 3 + 2];

                        if (isnan(px) || isnan(py) || isnan(pz))
                            continue; // NaN values indicate missing pixels

                        cvDepth.at<float>(y, x) = pz;
                        Eigen::Vector3d point = Eigen::Vector3d(px, py, pz);
                        point /= 1000;
                        CamPcd->points_.push_back(point);
                        Eigen::Vector3d color = Eigen::Vector3d(cvColor.at<cv::Vec3b>(y, x)(0), cvColor.at<cv::Vec3b>(y, x)(1), cvColor.at<cv::Vec3b>(y, x)(2));
                        color /= 255;
                        CamPcd->colors_.push_back(color);
                    }
                }

                // convert images to open3d
                std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
                rgbdImg->color_.Prepare(cvColor.cols, cvColor.rows, cvColor.channels(), cvColor.elemSize() / cvColor.channels());
                rgbdImg->depth_.Prepare(cvDepth.cols, cvDepth.rows, cvDepth.channels(), cvDepth.elemSize() / cvDepth.channels());
                if (cvColor.isContinuous() && cvDepth.isContinuous())
                {
                    memcpy(rgbdImg->color_.data_.data(), cvColor.data, cvColor.total() * cvColor.elemSize());
                    memcpy(rgbdImg->depth_.data_.data(), cvDepth.data, cvDepth.total() * cvDepth.elemSize());
                }
                else
                {
                    cout << "Warning opencv images are not continous! \n";
                }
                rgbdImg->depth_ = *rgbdImg->depth_.ConvertDepthToFloatImage(1000, 10);

                // lastly push back pcd and rgbdImg
                Pcds.push_back(CamPcd);
                rgbdImages.push_back(rgbdImg);

                //debug
                // auto o3dpcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbdImg, DepthLDTIntrinsic);
                // o3dpcd->PaintUniformColor(Eigen::Vector3d(0,0,1));
                // cout << "o3d pcd size " << o3dpcd->points_.size() << endl;
                // cout << "cam pcd size " << o3dpcd->points_.size() << endl;
                // open3d::visualization::DrawGeometries({CamPcd, getOrigin()});
                // open3d::visualization::DrawGeometries({o3dpcd, getOrigin()});
                // open3d::visualization::DrawGeometries({o3dpcd, CamPcd, getOrigin()});
            }
            catch (NxLibException &ex)
            {
                cout << "Error recording frame " << i << ": " << ex.getItemPath() << " " << ex.getErrorText() << endl;
                if (nFail < nFailMax)
                {
                    cout << "Recording another.." << endl;
                    nFail++;
                }
            }
        }
        return rgbdImages.size() > nFramesOld ? true : false;
    }
    return false;
}

void CameraWrapper::AlignUndistortandConvertO3D()
{
    using namespace cv;
    //iterate through all captures
    if (CameraType == AzureKinect)
    {
        for (auto &c : KinectCaptures)
        {
            k4a::image color = c->get_color_image();
            k4a::image depth = c->get_depth_image();

            FRCalibrationImages.push_back(KinectImageToOpenCV(color));

            // align color to depth
            color = KinectTransform.color_image_to_depth_camera(depth, color);

            //debug
            // open3d::io::WriteImageToPNG("color_distorted.png", *KinectImageToOpen3D(color));
            // open3d::io::WriteImageToPNG("depth_distorted.png", *KinectImageToOpen3D(depth));

            //undistort with kinect functions, since opencv remap creates more artifacts
            //note that final pcds still have artifacts when generating them from undistorted depht images.
            k4a::image k4aDepthUndistorted = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                                                PinholeDepth.width,
                                                                PinholeDepth.height,
                                                                PinholeDepth.width * (int)sizeof(uint16_t));
            remapk4a(depth.handle(), LUTDepth, k4aDepthUndistorted.handle(), interpolation_type_depth);

            k4a::image k4aColorUndistorted = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                                PinholeDepth.width,
                                                                PinholeDepth.height,
                                                                PinholeDepth.width * (int)sizeof(uint32_t));
            remapk4a(color.handle(), LUTDepth, k4aColorUndistorted.handle(), interpolation_type_color); //lut depth because color is aligned to depth

            //debug
            // open3d::io::WriteImageToPNG("depth_undistorted.png", *KinectImageToOpen3D(k4aDepthUndistorted));
            // open3d::io::WriteImageToPNG("color_undistorted.png", *KinectImageToOpen3D(k4aColorUndistorted));

            //convert to open3d
            std::shared_ptr<open3d::geometry::RGBDImage> rgbdImg = make_shared<open3d::geometry::RGBDImage>();
            rgbdImg->color_ = *KinectImageToOpen3D(k4aColorUndistorted);
            rgbdImg->depth_ = *KinectImageToOpen3D(k4aDepthUndistorted);
            rgbdImg->depth_ = *rgbdImg->depth_.ConvertDepthToFloatImage(1000.0, 3.0);
            rgbdImages.push_back(rgbdImg);

            //debug
            // auto _k4aPcd = KinectTransform.depth_image_to_point_cloud(depth, K4A_CALIBRATION_TYPE_DEPTH);
            //pass undistorted color image for k4a pcd color. Note: This might introduce errors.
            // auto k4aPcd = KinectPCDToOpen3D(_k4aPcd, &color);

            // k4aPcd->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
            // auto undistPcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbdImg, DepthLDTIntrinsic);
            // undistPcd->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
            // cout << "undistort pcd size " << undistPcd->points_.size() << endl;
            // cout << "k4aPcd size " << k4aPcd->points_.size() << endl;
            // open3d::visualization::DrawGeometries({undistPcd, k4aPcd, getOrigin()});
            // open3d::visualization::DrawGeometries({k4aPcd, getOrigin()});
            // open3d::visualization::DrawGeometries({undistPcd, getOrigin()});
            // open3d::visualization::DrawGeometries({distortedpcd, k4aPcd, getOrigin()});
        }
    }

    if (CameraType == IntelCamera)
    {
        // note: this is already done in record as realsense frame memory management doesn't allow creating own persistent rs2::frameset objects
        // realsense wait_for_frames returns a reference to a frameset from the internal frame queue
        // frame queue has a constant size and recycles frames in memory
    }
    if (CameraType == DataCamera)
    {
        cout << "Images in Data Cam Mode are already in correct format\n";
    }
}

void CameraWrapper::saveImagesToDisk(std::string fullPath, bool saveFRColor)
{
    if (fullPath.empty())
        fullPath = open3d::utility::filesystem::GetWorkingDirectory();
    std::string colorDir = fullPath + "/color/";
    std::string depthDir = fullPath + "/depth/";
    open3d::utility::filesystem::MakeDirectoryHierarchy(colorDir);
    open3d::utility::filesystem::MakeDirectoryHierarchy(depthDir);
    for (int i = 0; i < rgbdImages.size(); i++)
    {
        open3d::io::WriteImageToPNG(colorDir + getPicNumberString(i) + ".png", rgbdImages[i]->color_);
        open3d::geometry::Image &depthFloat = rgbdImages[i]->depth_; // 4 byte float image scaled with 1/1000
        open3d::geometry::Image depth;
        depth.Prepare(depthFloat.width_, depthFloat.height_, 1, 2);
        for (int i = 0; i < depth.height_; i++)
        {
            for (int j = 0; j < depth.width_; j++)
            {
                *depth.PointerAt<uint16_t>(j, i) = *depthFloat.PointerAt<float>(j, i) * 1000.0;
            }
        }
        open3d::io::WriteImageToPNG(depthDir + getPicNumberString(i) + ".png", depth); //note this cannot write float images!
    }
    if (saveFRColor)
    {
        std::string colorFRDir = fullPath + "/colorFR/";
        open3d::utility::filesystem::MakeDirectoryHierarchy(colorFRDir);
        for (int i = 0; i < FRCalibrationImages.size(); i++)
        {
            open3d::io::WriteImageToPNG(colorFRDir + getPicNumberString(i) + ".png", *OpenCVToOpen3D(FRCalibrationImages.at(i)));
        }
    }
}

void CameraWrapper::saveCameraInfoToDisk(std::string fullPath)
{
    if (fullPath.empty())
        fullPath = open3d::utility::filesystem::GetWorkingDirectory();

    Eigen::Matrix4d intrCol = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d intrDepth = Eigen::Matrix4d::Identity();
    intrCol.block<3,3>(0,0) = ColorLDTIntrinsic.intrinsic_matrix_;
    intrDepth.block<3,3>(0,0) = DepthLDTIntrinsic.intrinsic_matrix_;
    saveMatrixToDisc(fullPath, "/ColorLDTIntrinsic.txt", intrCol);
    saveMatrixToDisc(fullPath, "/DepthLDTIntrinsic.txt", intrDepth);
    saveVectorToDisc(fullPath, "/ColorDistortionCoeffs.txt", ColorDistortionCoefficients);
    saveVectorToDisc(fullPath, "/DepthDistortionCoeffs.txt", DepthDistortionCoefficients);
}

void CameraWrapper::GenerateVGPcd(int m, int nwait)
{
    cout << "integrating in tsdf" << endl;
    auto vg = open3d::pipelines::integration::ScalableTSDFVolume(voxel_length, d_trunc, open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
    for (auto rgbd : rgbdImages)
    {
        vg.Integrate(*rgbd, DepthLDTIntrinsic, Eigen::Matrix4d::Identity());
    }
    cout << "extracting pcd from tsdf" << endl;
    VGPcd = vg.ExtractPointCloud();
    VGPcd->EstimateNormals();
}

void CameraWrapper::SetAndSaveThresholdValues(std::shared_ptr<open3d::geometry::PointCloud> pcd)
{
    // reset so thresholds that could be read from file beforehand aren't used again
    dmax = 0;
    leftmax = 0;
    rightmax = 0;
    topmax = 0;
    bottommax = 0;
    // o3d coordinate system: x right, y down, z forward
    for (auto &p : pcd->points_)
    {
        if (p(0) > rightmax)
        {
            rightmax = p(0);
        }
        if (p(0) < leftmax)
        {
            leftmax = p(0);
        }
        if (p(1) < topmax) // o3d top is -y
        {
            topmax = p(1);
        }
        if (p(1) > bottommax) // o3d bottom is +y
        {
            bottommax = p(1);
        }
        if (abs(p(2)) > dmax)
        {
            dmax = abs(p(2));
        }
    }

    ofstream file(ResourceFolderPath + "config/" + SerialNumber + "FOVThresholds.txt");
    if (file.is_open())
    {
        file << rightmax << endl;
        file << leftmax << endl;
        file << topmax << endl;
        file << bottommax << endl;
        file << dmax << endl;
    }
    else
    {
        cout << "Warning: Writing Crop data to file failed! \n";
        return;
    }
    file.close();
}

void CameraWrapper::showOpen3dImages()
{
    for (auto img : rgbdImages)
    {
        open3d::visualization::DrawGeometries({img});
    }
}

void CameraWrapper::showOpen3DPcds()
{
    for (auto img : rgbdImages)
    {
        auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*img, DepthLDTIntrinsic);
        open3d::visualization::DrawGeometries({pcd});
    }
}

void CameraWrapper::saveImagesWithOpenCV()
{
    for (auto img : rgbdImages)
    {

        double min;
        double max;
        auto cvDepth = Open3DToOpenCV(img->depth_).clone();
        cv::minMaxIdx(cvDepth, &min, &max);
        cv::Mat adjMap;
        // expand your range to 0..255. Similar to histEq();
        cvDepth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

        // this is great. It converts your grayscale image into a tone-mapped one,
        // much more pleasing for the eye
        // function is found in contrib module, so include contrib.hpp
        // and link accordingly
        cv::Mat falseColorsMap;
        applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

        // cv::imshow("Out", adjMap);
        cv::imwrite("EdgeOpenCV.png", falseColorsMap);
        open3d::io::WriteImageToPNG("edgetestColor", img->color_);
    }
}

void CameraWrapper::printInfo()
{
    cout << "#################################### \n";
    cout << "Printing Camera Information \n";
    cout << "The Camera connection is " << Connected << endl;
    cout << "The camera SerialNumber is " << SerialNumber << endl;
    if (CameraType == AzureKinect)
    {
        cout << "The camera type is Azure Kinect\n";
        cout << "The list size of KinectCaptures is " << KinectCaptures.size() << endl;
    }
    if (CameraType == IntelCamera)
    {
        cout << "The camera type is IntelCamera\n";
        cout << "The list size of IntelCaptures is " << IntelCaptures.size() << endl;
    }
    if (CameraType == DataCamera)
    {
        cout << "The camera type is DataCamera\n";
        cout << "The datacam path is " << DataCamPath << endl;
    }
    cout << "The list size of rgbdImages is " << rgbdImages.size() << endl;

    cout << "#################################### \n";
}

void CameraWrapper::clearBuffers()
{
    KinectCaptures.clear();
    IntelCaptures.clear();
    rgbdImages.clear();
    CornersListofLists.clear();
    Pcds.clear();
    FRCalibrationImages.clear();
    K4APcds.clear();
    // KinectIMUSamples.clear(); //do not delete imu samples
}


bool CameraWrapper::CropRGBDImages()
{
    float left_thres = leftmax - crop_left / 100 * leftmax;
    float right_thres = rightmax - crop_right / 100 * rightmax;
    float top_thres = topmax - crop_top / 100 * topmax;
    float bottom_thres = bottommax - crop_bottom / 100 * bottommax;
    float d_thres = dmax - crop_depth / 100.0 * dmax;
    float d_thres_inv = dmax - (1 - crop_depth_inv / 100.0) * dmax;
    Eigen::Vector3d n_gravity = GravityVector;
    Eigen::Vector3d p_plane = (crop_ground_height + crop_ground_height_fine / 100) * n_gravity; //point on the plane
    bool allValid = true;
    for (int i = 0; i < rgbdImages.size(); i++)
    {
        int nullCount = 0;
        // #pragma omp parallel for
        for (int j = 0; j < rgbdImages[i]->depth_.height_; j++)
        {
            for (int k = 0; k < rgbdImages[i]->depth_.width_; k++)
            {
                //note the coordinate change
                float d = *rgbdImages[i]->depth_.PointerAt<float>(k, j);
                Eigen::Vector3d p = PixeltoPoint(j, k, d, DepthLDTIntrinsic.intrinsic_matrix_);
                if (!(abs(p(2)) < d_thres && abs(p(2)) > d_thres_inv && p(0) < right_thres &&
                      p(0) > left_thres && p(1) > top_thres &&
                      p(1) < bottom_thres && (p - p_plane).dot(n_gravity) < 0))
                {
                    *rgbdImages[i]->depth_.PointerAt<float>(k, j) = 0;
                }
                if (*rgbdImages[i]->depth_.PointerAt<float>(k, j) == 0)
                {
                    nullCount++;
                }
            }
        }
        if (nullCount == rgbdImages[i]->depth_.width_ * rgbdImages[i]->depth_.height_)
        {
            cout << "Warning: Depth image " << i << " of cam " << SerialNumber << " is all zeros!" << endl;
            allValid = false;
        }
    }
    return allValid;
}
shared_ptr<open3d::geometry::PointCloud> CameraWrapper::getCroppedPcd(shared_ptr<open3d::geometry::PointCloud> pcd_in)
{

    if (pcd_in == nullptr)
    {
        cout << "getCroppedPcd failed as pcd_in is nullptr!" << endl;
        return nullptr;
    }
    auto out = make_shared<open3d::geometry::PointCloud>();
    float left_thres = leftmax - crop_left / 100 * leftmax;
    float right_thres = rightmax - crop_right / 100 * rightmax;
    float top_thres = topmax - crop_top / 100 * topmax;
    float bottom_thres = bottommax - crop_bottom / 100 * bottommax;
    float d_thres = dmax - crop_depth / 100.0 * dmax;
    float d_thres_inv = dmax - (1 - crop_depth_inv / 100.0) * dmax;

    // cout << "left_thres " << left_thres << endl;
    // cout << "right_thres " << right_thres << endl;
    // cout << "top_thres " << top_thres << endl;
    // cout << "bottom_thres " << bottom_thres << endl;
    // cout << "d_thres " << d_thres << endl;
    // cout << "d_thres_inv " << d_thres_inv << endl;

    Eigen::Vector3d n_gravity = GravityVector;
    Eigen::Vector3d p_plane = (crop_ground_height + crop_ground_height_fine / 100) * n_gravity; //point on the plane
    //vis gravity vector
    // shared_ptr<open3d::geometry::LineSet> ls = make_shared<open3d::geometry::LineSet>();
    // ls->points_.push_back(Eigen::Vector3d(0, 0, 0));
    // ls->points_.push_back(n_gravity);
    // ls->lines_.push_back(Eigen::Vector2i(0, 1));
    // open3d::visualization::DrawGeometries({ls, pcd_in, getOrigin()});
    for (int i = 0; i < pcd_in->points_.size(); i++)
    {
        auto &p = pcd_in->points_[i];

        if (abs(p(2)) < d_thres && abs(p(2)) > d_thres_inv && p(0) < right_thres && p(0) > left_thres && p(1) > top_thres && p(1) < bottom_thres && (p - p_plane).dot(n_gravity) < 0)
        {
            out->points_.push_back(pcd_in->points_[i]);
            if (pcd_in->HasColors())
                out->colors_.push_back(pcd_in->colors_[i]);
            if (pcd_in->HasNormals())
                out->normals_.push_back(pcd_in->normals_[i]);
        }
    }
    return out;
}

bool CameraWrapper::ReadFOVDataFromDisk(const string &path)
{
    cout << "Reading field of view data from " << path + SerialNumber + "FOV.txt" << endl;

    ifstream fovdata(path + SerialNumber + "FOV.txt");
    string line;
    vector<float> data;
    if (fovdata.is_open())
    {
        while (getline(fovdata, line))
        {
            data.push_back(stod(line));
        }
        if (data.size() != 8)
        {
            cout << "Warning: fov data is bad.\n";
            return false;
        }
        crop_left = data[0];
        crop_right = data[1];
        crop_top = data[2];
        crop_bottom = data[3];
        crop_depth = data[4];
        crop_depth_inv = data[5];
        crop_ground_height = data[6];
        crop_ground_height_fine = data[7];
    }
    else
    {
        cout << "Warning: Reading fov data from file went wrong\n";
        return false;
    }

    cout << "Reading field of view threshold data from " << path + SerialNumber + "FOVThresholds.txt" << endl;

    ifstream fovdataThreshold(path + SerialNumber + "FOVThresholds.txt");
    vector<float> dataThres;
    if (fovdataThreshold.is_open())
    {
        while (getline(fovdataThreshold, line))
        {
            dataThres.push_back(stod(line));
        }
        if (dataThres.size() != 5)
        {
            cout << "Warning: fov threshold data is bad.\n";
            return false;
        }

        rightmax = dataThres[0];
        leftmax = dataThres[1];
        topmax = dataThres[2];
        bottommax = dataThres[3];
        dmax = dataThres[4];
    }
    else
    {
        cout << "Warning: Reading fov threshold data from file went wrong\n";
        return false;
    }
    return true;
}

bool CameraWrapper::ReadGravityVectorFromDisk(const std::string &path)
{
    cout << "Reading gravity vector from " << path + SerialNumber + "Gravity.txt" << endl;

    ifstream file(path + SerialNumber + "Gravity.txt");
    string line;
    vector<float> data;
    if (file.is_open())
    {
        while (getline(file, line))
        {
            data.push_back(stod(line));
        }
        if (data.size() != 3)
        {
            cout << "Warning: Gravity data is bad.\n";
            return false;
        }
        GravityVector(0) = data[0];
        GravityVector(1) = data[1];
        GravityVector(2) = data[2];
    }
    else
    {
        cout << "Warning: Reading gravity vector from file went wrong\n";
        return false;
    }
    return true;
}


bool CameraWrapper::ReadCameraPositionFromDisk(const std::string &path)
{
    cout << "Reading camera position from " << path + SerialNumber + "Position.txt" << endl;
    ifstream file(path + SerialNumber + "Position.txt");
    string line;
    std::string delimiter = " ";
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    if (file.is_open())
    {
        int row = 0;
        while (getline(file, line))
        {
            std::string token;
            size_t pos;
            int col = 0;
            while ((pos = line.find(delimiter)) != std::string::npos)
            {
                token = line.substr(0, pos);
                if (token.compare(""))
                { //this stupid shit return wrong if strings are equal
                    pose(row, col) = stod(token);
                    line.erase(0, pos + delimiter.length());
                    col++;
                }
                else
                {
                    line.erase(0, 1);
                }
            }
            pose(row, col) = stod(line); //last entry
            row++;
        }
        Extrinsic = pose;
    }
    else
    {
        cout << "Warning: Could not read camera position file \n";
        return false;
    }
    return true;
}

void CameraWrapper::SaveGravityVectorToDisk(const std::string &path)
{
    ofstream file(path + SerialNumber + "Gravity.txt");
    if (file.is_open())
    {
        file << GravityVector(0) << endl;
        file << GravityVector(1) << endl;
        file << GravityVector(2) << endl;
    }
    else
    {
        cout << "Warning: Writing gravity vector to file failed! \n";
        return;
    }
    file.close();
}


void CameraWrapper::SaveCameraPositionToDisk(const std::string &path)
{
    cout << "Saving camera position to " << path + SerialNumber + "Position.txt" << endl;
    ofstream file(path + SerialNumber + "Position.txt");
    if (file.is_open())
    {
        file << Extrinsic << endl;
    }
    else
    {
        cout << "Warning: Writing camera position to file failed! \n";
    }
    file.close();
}

void CameraWrapper::printAllIntelSensorOptions()
{
    for (auto &&s : IntelDevice.query_sensors())
    {
        cout << s.get_info(RS2_CAMERA_INFO_NAME) << " Sensor options: " << endl;
        auto options = s.get_supported_options();
        for (auto o : options)
        {
            if (s.supports(o))
            {
                auto optRange = s.get_option_range(o);
                cout << s.get_option_name(o) << "  #  ";
                cout << "default " << optRange.def << " | ";
                cout << "min " << optRange.min << " | ";
                cout << "max " << optRange.max << " | ";
                cout << "step " << optRange.step << endl;
            }
        }
        cout << endl;
    }
}

void CameraWrapper::printIntelSensorOption(const rs2::sensor &sensor, rs2_option option)
{
    if (!sensor.supports(option))
    {
        cout << "Error: Intel Sensor option is not supported!" << endl;
        return;
    }
    auto optRange = sensor.get_option_range(option);
    cout << sensor.get_option_name(option) << "  #  ";
    cout << "default " << optRange.def << " | ";
    cout << "min " << optRange.min << " | ";
    cout << "max " << optRange.max << " | ";
    cout << "step " << optRange.step << endl;
}

void CameraWrapper::setIntelSensorOption(const rs2::sensor &sensor, rs2_option option, float value)
{
    if (!sensor.supports(option))
    {
        cout << "This option is not supported by this sensor" << std::endl;
        return;
    }

    try
    {
        sensor.set_option(option, value);
    }
    catch (const rs2::error &e)
    {
        // Some options can only be set while the camera is streaming,
        // and generally the hardware might fail so it is good practice to catch exceptions from set_option
        std::cerr << "Failed to set option " << option << ". (" << e.what() << ")" << std::endl;
    }
}

// call rs-enumerate-devices for info of available streams
rs2::stream_profile CameraWrapper::getIntelStreamProfile(const rs2::sensor &s, int w, int h, int fps, rs2_format format, std::string stream_name)
{
    for (auto p : s.get_stream_profiles())
    {
        if (p.as<rs2::video_stream_profile>().width() == w &&
            p.as<rs2::video_stream_profile>().height() == h &&
            p.fps() == fps &&
            p.format() == format &&
            p.as<rs2::video_stream_profile>().stream_name() == stream_name)
        {
            return p;
        }
    }
    cout << "Error: Intel Stream Profile not supported!" << endl;
    return rs2::stream_profile();
}

void CameraWrapper::startStreaming()
{
    int n_warmup = 30;
    if (CameraType == IntelCamera)
    {
        if (!IsStreaming)
        {
            try
            {
                IntelRGBSensor.open(IntelRGBProfile);
                IntelDepthSensor.open(IntelDepthProfiles);
                IntelRGBSensor.start(IntelSyncer);
                IntelDepthSensor.start(IntelSyncer);
                IsStreaming = true;

                // note: warmup frames seem to be more stable than sleep as it can sometimes take longer to get a frame.
                // std::this_thread::sleep_for(2s);
                for (int i = 0; i < n_warmup; i++) // 30 frames warmup for autoexposure and stream stability
                {
                    try
                    {
                        rs2::frameset warumUpCapture = IntelSyncer.wait_for_frames();
                    }
                    catch (const rs2::error &e)
                    {
                        std::cerr << e.what() << '\n';
                        cout << "Error: Getting warmup frame " << i << " in startStreaming failed!" << endl;
                    }
                }
            }
            catch (rs2::error &e)
            {
                cerr << e.what() << endl;
                cout << "Error: Intel sensor start failed!" << endl;
                return;
            }
        }
    }
    else
    {
        cout << "Start not implemented for this cam type" << endl;
    }
}

void CameraWrapper::stopStreaming()
{
    if (CameraType == IntelCamera)
    {
        if (IsStreaming)
        {
            try
            {
                IntelRGBSensor.stop();
                IntelDepthSensor.stop();
                IntelRGBSensor.close();
                IntelDepthSensor.close();
                IsStreaming = false;
            }
            catch (rs2::error &e)
            {
                cerr << e.what() << endl;
                cout << "Error: Intel sensor stop failed!" << endl;
            }
        }
    }
    else
    {
        cout << "Stop not implemented for this cam type" << endl;
    }
}

bool CameraWrapper::InsideDepthImage(const Eigen::Vector2d &p)
{
    if (p(0) > 0 && p(0) < DepthLDTIntrinsic.height_ && p(1) > 0 && p(1) < DepthLDTIntrinsic.width_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//todo use everywhere
bool CameraWrapper::IsVisible(const Eigen::Vector3d& p, const Eigen::MatrixXd& depth_buffer){
    // open3d::visualization::DrawGeometries({EigenToO3DDepthImage(depth_buffer)});
    double epsilon = 0.01; // large for depth noise
    Eigen::Vector2d ind = PointtoPixelExact(p, DepthLDTIntrinsic.intrinsic_matrix_);
    if (InsideDepthImage(ind)){
        if(p(2) <= depth_buffer((int) ind(0), (int)ind(1)) + epsilon){
            return true;
        }
    }
    return false;
}

bool CameraWrapper::disconnect()
{
    bool success = false;
    if (CameraType == PhoxiScanner)
    {
        success = PhoxiDevice->Disconnect(false, true);
    }
    else if (CameraType == ZividCamera)
    {
        try
        {
            ZividCam.disconnect();
            success = true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in disconnect: " << Zivid::toString(e) << std::endl;
            success = false;
        }
    }
    else if (CameraType == EnsensoCamera)
    {
        try
        {
            printf("Closing camera\n");
            NxLibCommand(cmdClose).execute();
            printf("Closing NxLib\n");
            nxLibFinalize();
            success = true;
        }
        catch (NxLibException& ex)
        {
            cout << ex.getItemPath() << endl;
            cout << ex.getErrorText() << endl;
            success = false;
        }
    }
    if (success)
    {
        Connected = false;
    }
    return success;
}

std::shared_ptr<std::vector<PandiaPlane>> CameraWrapper::getFOVPlanes()
{
    float left_thres = leftmax - crop_left / 100 * leftmax;
    float right_thres = rightmax - crop_right / 100 * rightmax;
    float top_thres = topmax - crop_top / 100 * topmax;
    float bottom_thres = bottommax - crop_bottom / 100 * bottommax;
    float d_thres = dmax - crop_depth / 100.0 * dmax;
    float d_thres_inv = dmax - (1 - crop_depth_inv / 100.0) * dmax;
    std::shared_ptr<std::vector<PandiaPlane>> out = make_shared<vector<PandiaPlane>>();
    out->push_back(PandiaPlane(Eigen::Vector3d(left_thres - 1, 0, 0), Eigen::Vector3d(left_thres, 0, 0)));
    out->push_back(PandiaPlane(Eigen::Vector3d(right_thres + 1, 0, 0), Eigen::Vector3d(right_thres, 0, 0)));
    out->push_back(PandiaPlane(Eigen::Vector3d(0, top_thres - 1, 0), Eigen::Vector3d(0, top_thres, 0)));
    out->push_back(PandiaPlane(Eigen::Vector3d(0, bottom_thres + 01, 0), Eigen::Vector3d(0, bottom_thres, 0)));
    out->push_back(PandiaPlane(Eigen::Vector3d(0, 0, d_thres + 1), Eigen::Vector3d(0, 0, d_thres)));
    out->push_back(PandiaPlane(Eigen::Vector3d(0, 0, -d_thres - 1), Eigen::Vector3d(0, 0, d_thres_inv)));
    out->push_back(PandiaPlane(GravityVector, (crop_ground_height + crop_ground_height_fine / 100) * GravityVector.normalized()));
    // open3d::visualization::DrawGeometries({getOrigin(),planes->at(0).getPlaneTriangleMesh(),planes->at(1).getPlaneTriangleMesh(),planes->at(2).getPlaneTriangleMesh(),planes->at(3).getPlaneTriangleMesh(),planes->at(4).getPlaneTriangleMesh(),planes->at(5).getPlaneTriangleMesh(),planes->at(6).getPlaneTriangleMesh()});
    return out;
}
