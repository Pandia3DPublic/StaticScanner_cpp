#pragma once
#include "CameraWrapper.h"
#define PANDIA_PI 3.14159265358979323846264

//######### Disk Stuff ###########
std::string getPicNumberString(int a);
void setFromIntrinsicFile(const std::string &filepath, open3d::camera::PinholeCameraIntrinsic &intrinsic);
void IntrinsicFileHelperFunction(std::string value, int lineCounter, open3d::camera::PinholeCameraIntrinsic &intrinsic);
bool fileExists(const std::string &filename);
bool saveMatrixToDisc(std::string path, std::string name, const Eigen::Matrix4d &matrix);
bool saveVectorToDisc(std::string path, std::string name, const std::vector<float> vector);
bool saveStringToDisc(std::string path, std::string name, const std::string& content);
Eigen::Matrix4d readMatrixFromDisc(const std::string &PathAndName);
std::string readStringFromDisc(const std::string& path,const std::string& name);
//######## conversion functions ###########
cv::Matx33d eigenToOpenCv33(const Eigen::Matrix3d &m);
cv::Mat Open3DToOpenCV(const open3d::geometry::Image &img);
std::shared_ptr<open3d::geometry::Image> OpenCVToOpen3D(const cv::Mat &img);
cv::Mat KinectImageToOpenCV(const k4a::image &im);
std::shared_ptr<open3d::geometry::Image> KinectImageToOpen3D(const k4a::image &im);
k4a::image Open3DToKinectImage(const open3d::geometry::Image &img);
std::shared_ptr<open3d::geometry::PointCloud> KinectPCDToOpen3D(const k4a::image &k4apcd, const k4a::image *color = nullptr);
cv::Mat FrametoMat(const rs2::frame &f);
cv::Mat FrametoMatMeters(const rs2::depth_frame &f);
cv::Mat FrametoMatMillimeters(const rs2::depth_frame &f);
double toDegrees(const double &radians);
double toRadians(const double &degrees);
Eigen::Matrix4d arraysToEigenRow(float *rot, float *trans);
Eigen::Matrix4d arraysToEigenColumn(float *rot, float *trans);
std::shared_ptr<open3d::geometry::PointCloud> EigenDepthToPCD(const Eigen::MatrixXd &depth_buffer, open3d::camera::PinholeCameraIntrinsic &intr);
std::shared_ptr<open3d::geometry::Image> EigenToO3DDepthImage(const Eigen::MatrixXd &depth);
Eigen::MatrixXd O3DDepthtoEigen(std::shared_ptr<open3d::geometry::Image> depth);
Json::Value StringToJson(const std::string &json_str);
std::string JsonToString(const Json::Value json);
//########## vis helper ###############
std::shared_ptr<open3d::geometry::LineSet> getOrigin();
std::shared_ptr<open3d::geometry::LineSet> getLineVis(const Eigen::Matrix3d &axis);
std::shared_ptr<open3d::geometry::TriangleMesh> getCameraArrow(const Eigen::Matrix4d &trans);

//########## misc #####################
CameraWrapper *getCameraBySerialNumber(std::shared_ptr<std::vector<CameraWrapper>> cameras, std::string SerialNumber);
bool SortbySerialNumber(const CameraWrapper &c1, const CameraWrapper &c2);
bool SortbyAlphabet(const std::string &s1, const std::string &s2);
int getNumberofConnectedDevices(CameraTypes camtype);
Eigen::Matrix4d getflip();
Eigen::Matrix3d getflip3();
Eigen::Vector3d PixeltoPoint(int &i, int &j, const float &depth, const Eigen::Matrix3d &cameraMatrix);
Eigen::Vector2i PointtoPixelRounded(const Eigen::Vector3d &p, const Eigen::Matrix3d &cameraMatrix);
Eigen::Vector2d PointtoPixelExact(const Eigen::Vector3d &p, const Eigen::Matrix3d &cameraMatrix);
void addEdges(std::shared_ptr<open3d::geometry::RGBDImage> img, std::vector<Eigen::Vector2i> &edges, const Eigen::Vector3d &color, int depthvalue);
Eigen::Matrix4d getRotationMatrixFromVectorAndAngle(const Eigen::Vector3d &vec_in, const double &a);
Eigen::Matrix3d getRotationMatrixFromAtoB(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
std::vector<double> getDistanceFromMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, std::shared_ptr<open3d::geometry::PointCloud> pcd,
                                        std::vector<Eigen::Vector3d> &nearestPoints);
void applyNormalFilter(std::shared_ptr<open3d::geometry::PointCloud> &pcd_in);
std::vector<double> getPointCloudPointToPlaneDistances(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2);
std::vector<double> getPointCloudPointToPlaneDistancesBidirectional(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2);
std::vector<int> getInnerToOuterEdgeMapping(std::vector<Eigen::Vector2i> &innerEdge, std::vector<Eigen::Vector2i> &outerEdge);

void applyNormalFilter(std::shared_ptr<open3d::geometry::RGBDImage> rgbd, open3d::camera::PinholeCameraIntrinsic &intrinsic);
void applyNormalFilter(open3d::geometry::Image &depth, open3d::camera::PinholeCameraIntrinsic &intrinsic);
void applyNormalFilter(open3d::geometry::Image &depth, std::shared_ptr<open3d::geometry::PointCloud> pcd_in, open3d::camera::PinholeCameraIntrinsic &intrinsic);

std::shared_ptr<open3d::geometry::RGBDImage> getAverageImgFilter(std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> &rgbds);
std::shared_ptr<open3d::geometry::RGBDImage> getSoftAverageImgFilter(std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> &rgbds);
std::shared_ptr<open3d::geometry::RGBDImage> getAverageImg(std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> &rgbds);
void showEdges(cv::Mat img);

double edgeFunction(const Eigen::Vector2d &v0, const Eigen::Vector2d &v1, const Eigen::Vector2d &p);
double edgeFunctionCCW(const Eigen::Vector2d &v0, const Eigen::Vector2d &v1, const Eigen::Vector2d &p);

std::vector<Eigen::Vector3d> getHalfsphereVectors();
std::shared_ptr<open3d::geometry::LineSet> visualizeHalfsphereVectors(const std::vector<Eigen::Vector3d> &vecs, const Eigen::Vector3d &p = Eigen::Vector3d::Zero());
std::shared_ptr<open3d::geometry::PointCloud> getScannablePcd(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, std::shared_ptr<open3d::geometry::PointCloud> pcd);

Eigen::Matrix3d kabsch(const Eigen::Matrix3Xd &in, const Eigen::Matrix3Xd &out);

bool IsDepthImageValid(const open3d::geometry::Image &depth);
bool IsIntelFramesetValid(const rs2::frameset &fs);

std::string getExtensionFromFilename(const std::string& filename);
open3d::camera::PinholeCameraIntrinsic getScaledIntrinsic(const open3d::camera::PinholeCameraIntrinsic &intr, int newWidth, int newHeight);

template <typename T>
Eigen::Matrix4d getRx(T a)
{
    Eigen::Matrix4d rx;
    rx << 1, 0, 0, 0,
        0, std::cos(a), -std::sin(a), 0,
        0, std::sin(a), std::cos(a), 0,
        0, 0, 0, 1;
    return rx;
}

template <typename T>
Eigen::Matrix4d getRy(T a)
{
    Eigen::Matrix4d ry;
    ry << std::cos(a), 0, std::sin(a), 0,
        0, 1, 0, 0,
        -std::sin(a), 0, std::cos(a), 0,
        0, 0, 0, 1;
    return ry;
}
template <typename T>
Eigen::Matrix4d getRz(T a)
{
    Eigen::Matrix4d rz;
    rz << std::cos(a), -std::sin(a), 0, 0,
        std::sin(a), std::cos(a), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return rz;
}

template <typename T>
Eigen::Matrix4d getTrans(const T &a)
{
    Eigen::Matrix4d t;
    t << 1, 0, 0, a(0),
        0, 1, 0, a(1),
        0, 0, 1, a(2),
        0, 0, 0, 1;
    return t;
}
