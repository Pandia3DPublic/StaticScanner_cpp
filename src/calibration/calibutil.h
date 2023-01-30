#pragma once
#include "CameraWrapper.h"

//################# conversion functions ############################
cv::Matx33f k4aColorCalibtoOpencvMatrix(const k4a::calibration &cal);
std::vector<float> k4aColorCalibtoOpencvDistortionCoeffs(const k4a::calibration &cal);
cv::Matx33f CalibrationIntrToOpenCVMat(CameraWrapper& cam);
Eigen::Matrix4d OpencvTransformationToEigen(cv::Matx33d R, cv::Vec3d t);
Eigen::Matrix4d refineRegistrationICP(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d &init, int maxIterations = 30, double maxCorsDistance = 0.02);
Eigen::Matrix4d refineRegistrationICP(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d &init, double &goodness, int maxIterations = 30, double maxCorsDistance = 0.02);
Eigen::Matrix4d refineRegistrationICPCenteredPCDs(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target, double &goodness, const Eigen::Matrix4d &init = Eigen::Matrix4d::Identity(),  int maxIterations = 30, double maxCorsDistance = 0.02);