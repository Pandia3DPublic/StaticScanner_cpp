#pragma once
#include "open3d/camera/PinholeCameraIntrinsic.h"
#include "KinectUndistort.h"
#include "Plane.h"

enum CameraTypes
{
    AzureKinect,
    IntelCamera,
    DataCamera,
    PhoxiScanner,
    ZividCamera,
    EnsensoCamera
};

class CameraWrapper
{
public:
    CameraTypes CameraType = AzureKinect;
    CameraWrapper() = default;
    CameraWrapper(CameraTypes camtype);


    bool connect(int CameraNumber = 0, std::string DataCamPath_ = "");
    bool resetConnection(bool streamAfterReset = true);
    bool record(int n, int nwait = 30, bool checkDepthImage = true);
    void AlignUndistortandConvertO3D();
    void saveImagesToDisk(std::string fullPath = "", bool saveFRColor = false);
    void saveCameraInfoToDisk(std::string fullPath = "");
    void GenerateVGPcd(int m, int nwait = 30);
    void showOpen3dImages();
    void showOpen3DPcds();
    void saveImagesWithOpenCV();
    void printInfo();
    void clearBuffers();
    bool ReadFOVDataFromDisk(const std::string &path);
    bool ReadGravityVectorFromDisk(const std::string &path);
    bool ReadCameraPositionFromDisk(const std::string &path);
    void SaveGravityVectorToDisk(const std::string &path);
    void SaveCameraPositionToDisk(const std::string &path);
    void printAllIntelSensorOptions();
    void printIntelSensorOption(const rs2::sensor &sensor, rs2_option option);
    void setIntelSensorOption(const rs2::sensor &sensor, rs2_option option, float value);
    rs2::stream_profile getIntelStreamProfile(const rs2::sensor &s, int w, int h, int fps, rs2_format format, std::string stream_name);
    void startStreaming();
    void stopStreaming();
    bool IsVisible(const Eigen::Vector3d &p, const Eigen::MatrixXd &depth_buffer);
    bool disconnect();
    std::shared_ptr<std::vector<PandiaPlane>> getFOVPlanes();

    //todo check if necessary and used
    // bool InFieldOfView(const Eigen::Vector3d &p, const Eigen::Matrix4d &T_cam);
    Eigen::Vector2i getImageIndices(const Eigen::Vector3d &p, const Eigen::Matrix4d &T_cam);
    Eigen::Vector2i getImageIndices(const Eigen::Vector3d &p);
    bool InsideDepthImage(const Eigen::Vector2d &p);

    //######## general Variables ############
    std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> rgbdImages;
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Pcds;    //for saving data during a scan
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> K4APcds; //not used atm
    std::shared_ptr<open3d::geometry::RGBDImage> ScannedRGBD;
    std::shared_ptr<open3d::geometry::PointCloud> ScannedPcd;
    std::vector<cv::Mat> FRCalibrationImages;                    //used in Calibration Singleton
    cv::Mat VolumeImage;
    open3d::camera::PinholeCameraIntrinsic DepthLDTIntrinsic;    //refers to the full resolution undistorted depth image
    open3d::camera::PinholeCameraIntrinsic ColorLDTIntrinsic;    //refers to the full resolution distorted color camera
    open3d::camera::PinholeCameraIntrinsic CalibrationIntrinsic; //refers to the full resolution distorted color camera
    Eigen::Matrix4d Extrinsic = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d IMUToCamTransform = Eigen::Matrix4d::Identity();
    bool Connected = false;   //True if a camera is connected
    bool IsStreaming = false; // True if a camera is streaming data to the host device, is set in start/stop
    std::string SerialNumber = "undefined";
    int FullCalibrationImageHeight = 1; //set in connect
    int FullCalibrationImageWidth = 1;
    Eigen::Vector3d GravityVector = Eigen::Vector3d(0, 0, -1);
    double CameraHeight = 0;
    Eigen::Matrix4d CameraToReferenceTrans = Eigen::Matrix4d::Identity();

    //########## Undistort Variables ##############
    std::vector<float> CalibrationCamDistortionCoeffs; //color for kinect, infrared for intel
    std::vector<float> ColorDistortionCoefficients;    // Input/output vector of distortion coefficients (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]]) of 4, 5, 8, 12 or 14 elements.
    std::vector<float> DepthDistortionCoefficients;    // not used atm
    k4a_image_t LUTDepth;                              //using microsoft k4a types, since we get too much artifacts using opencv
    pinhole_t PinholeDepth;
    interpolation_t interpolation_type_depth = INTERPOLATION_NEARESTNEIGHBOR;
    interpolation_t interpolation_type_color = INTERPOLATION_NEARESTNEIGHBOR;
    // interpolation_t interpolation_type_color = INTERPOLATION_BILINEAR; // todo bilinear remap is buggy

    //########## Azure Kinect Variables ##############
    k4a_device_configuration_t KinectConfig;
    k4a::device KinectDevice;
    k4a::calibration KinectCalibration;
    k4a::transformation KinectTransform;
    std::vector<std::shared_ptr<k4a::capture>> KinectCaptures; // todo implement capture buffers as list
    std::vector<k4a_imu_sample_t> KinectIMUSamples;

    //############### Intel Variables #################
    static rs2::context IntelContext; // for multicam
    // rs2::config IntelConfig;
    rs2::device IntelDevice;
    rs2::sensor IntelRGBSensor;
    rs2::sensor IntelDepthSensor; // intel stereo module which includes depth and infrared streams
    rs2::stream_profile IntelRGBProfile;
    std::vector<rs2::stream_profile> IntelDepthProfiles; // first element is depth, second infrared profile
    rs2::syncer IntelSyncer = rs2::syncer(10);           //to get synced rgb and depth data
    // rs2::pipeline IntelPipe = rs2::pipeline(IntelContext);
    std::vector<std::shared_ptr<rs2::frameset>> IntelCaptures;
    std::vector<rs2_vector> IntelIMUSamples;
    std::shared_ptr<rs2::align> Align_to_depth;
    std::shared_ptr<rs2::align> Align_to_color;
    // Intel filters, see https://dev.intelrealsense.com/docs/post-processing-filters and https://dev.intelrealsense.com/docs/depth-post-processing
    rs2::disparity_transform Depth_to_disparity = rs2::disparity_transform(true);  // transforms to disparity space which is 1/Distance
    rs2::disparity_transform Disparity_to_depth = rs2::disparity_transform(false); // back to normal
    rs2::decimation_filter Dec_filter;                                             // Decimation - reduces depth frame density
    rs2::threshold_filter Thr_filter;                                              // Threshold  - removes values outside recommended range
    rs2::spatial_filter Spat_filter;                                               // Spatial    - edge-preserving spatial smoothing

    //############# Data Camera Variables ###############
    std::string DataCamPath;

    //############# Phoxi Variables ###############
    static pho::api::PhoXiFactory PhoxiFactory;
    static std::vector<pho::api::PhoXiDeviceInformation> PhoxiDeviceList; //is set in getNumberofConnectedDevices
    pho::api::PPhoXi PhoxiDevice;

    //############# Zivid Variables ###############
    static Zivid::Application ZividApp;
    static std::vector<Zivid::Camera> ZividCameraList; // set in getNumberofConnectedDevices
    Zivid::Camera ZividCam;
    Zivid::Settings ZividSettings;

    //############# Ensenso Variables ###############
    static NxLibItem EnsensoRoot;
    static NxLibItem EnsensoCameraList; //note: EnsensoCameraList.count() is always 2 more because of additional non camera nodes
    NxLibItem EnsensoCam;
    NxLibItem EnsensoSettings;
    
    //############# Camera Field of View Variables ###########
    void SetAndSaveThresholdValues(std::shared_ptr<open3d::geometry::PointCloud> pcd);
    std::shared_ptr<open3d::geometry::PointCloud> getCroppedPcd(std::shared_ptr<open3d::geometry::PointCloud> pcd_in);
    std::shared_ptr<open3d::geometry::PointCloud> VGPcd;
    std::shared_ptr<open3d::geometry::PointCloud> DownsampledPcd;
    bool CropRGBDImages();
    float voxel_length = 0.01;
    float d_trunc = 0.03;
    float crop_top = 0;
    float crop_bottom = 0;
    float crop_left = 0;
    float crop_right = 0;
    float crop_depth = 0;
    float crop_depth_inv = 0;
    float crop_ground_height = 1.5;
    float crop_ground_height_fine = 0.0;
    float dmax = 0;
    float leftmax = 0;
    float rightmax = 0;
    float topmax = 0;
    float bottommax = 0;

    //############# Camera Calibration Variables #############
    std::vector<std::vector<cv::Point2f>> CornersListofLists;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
