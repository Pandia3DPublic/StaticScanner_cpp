#pragma once
#include "CameraWrapper.h"

Eigen::MatrixXd getFOVDepthBuffer(CameraWrapper &cam, open3d::geometry::TriangleMesh mesh, const Eigen::Matrix4d &refToCamTrans);
double getCurrentVolume(CameraWrapper &cam);
std::shared_ptr<open3d::geometry::TriangleMesh> getReferenceMesh(std::string name);
