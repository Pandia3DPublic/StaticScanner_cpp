#pragma once
#include "CameraWrapper.h"
void o3dToOgreMesh(Ogre::ManualObject *mesh, std::shared_ptr<open3d::geometry::TriangleMesh> o3dMesh, 
const std::string &MaterialName, const Eigen::Matrix3d &trans= Eigen::Matrix3d::Identity(), float alpha = 1.0f);
void o3dToOgrePcd(Ogre::ManualObject *pcd, std::shared_ptr<open3d::geometry::PointCloud> o3dpcd, 
const std::string &MaterialName, const Eigen::Matrix3d &trans = Eigen::Matrix3d::Identity(), float alpha = 1.0f);
void o3dToOgreLineList(Ogre::ManualObject *mesh, std::shared_ptr<open3d::geometry::LineSet> o3dLineList,
                       const std::string &MaterialName, const Eigen::Matrix3d &trans, float alpha);
std::shared_ptr<open3d::geometry::TriangleMesh> readMesh(const std::string &path);
std::shared_ptr<open3d::geometry::PointCloud> readPcd(const std::string &path);
std::vector<std::string> getFileNames(const std::string& folder);
std::vector<std::string> getCameraSerialNumbers(std::shared_ptr<std::vector<CameraWrapper>> cameras);
Ogre::Vector3 o3dToOgre(const Eigen::Vector3d &in);
std::shared_ptr<open3d::geometry::TriangleMesh> getFOVGeometryfromPlanes(std::shared_ptr<std::vector<PandiaPlane>> planes, std::shared_ptr<open3d::geometry::LineSet> lines);