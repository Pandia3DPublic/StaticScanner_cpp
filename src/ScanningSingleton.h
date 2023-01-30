#pragma once
#include "CameraWrapper.h"

class ScanningSingleton
{
public:
    ScanningSingleton(std::shared_ptr<std::vector<CameraWrapper>> cameras_);
    ~ScanningSingleton();

    //################# Current Scan Variables ####################
    std::shared_ptr<open3d::geometry::PointCloud> CurrentPCD;
    std::shared_ptr<open3d::geometry::PointCloud> CurrentDownsampledPCD;
    std::shared_ptr<open3d::geometry::PointCloud> OldPCD;
    std::shared_ptr<open3d::geometry::PointCloud> OldDownsampledPCD;
    std::vector<Eigen::Vector3d> CurrentPCDColors;
    std::shared_ptr<open3d::pipelines::integration::ScalableTSDFVolume> CurrentVoxelGrid;

    //########### Basic Variables ########################
    std::shared_ptr<open3d::geometry::TriangleMesh> ReferenceMesh;
    std::shared_ptr<open3d::geometry::PointCloud> ReferencePCD;
    std::shared_ptr<open3d::geometry::PointCloud> SmallReferencePCD;
    Eigen::Vector3d SmallReferencePCDCenter = Eigen::Vector3d::Zero();
    std::shared_ptr<std::vector<CameraWrapper>> Cameras;
    bool isReferenceMesh = true; //is reference model a mesh or pcd

    std::vector<double> distances; //Distances of points in scanned pcd to transformed reference mesh/pcd

    Eigen::Matrix4d CurrentToReferenceTransform = Eigen::Matrix4d::Identity();
    float voxel_length = 0.004;
    float d_trunc = 0.03;

    void reset();
    void ComputeDistanceToReference();
    void ColorCurrentPCDwithDistance();
    void ColorCurrentPCDGradient();
    void RecolorCurrentPCD();
    Eigen::MatrixXd getFOVDepthBuffer(CameraWrapper &cam);

    void ColorFOV();
    void FilterCurrentPcdUsingMulitCamVisibility();


    Eigen::Matrix3d ReferencePCAVectors = Eigen::Matrix3d::Identity(); //column wise
    std::shared_ptr<open3d::geometry::PointCloud> getMissingPoints();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::vector<Eigen::Vector2i> getEdgePixels(std::shared_ptr<open3d::geometry::RGBDImage> img);
std::vector<Eigen::Vector2i> getBackgroundEdgePixels(std::shared_ptr<open3d::geometry::RGBDImage> img, std::vector<Eigen::Vector2i> &edges);
void applyEdgeColorPersistenceFilter(std::shared_ptr<open3d::geometry::RGBDImage> rgbd);


Eigen::Matrix3d getPCAVectors(std::shared_ptr<open3d::geometry::PointCloud> pcd);
Eigen::Matrix4d getPCABasedRotation(std::shared_ptr<open3d::geometry::PointCloud> source,
                                    std::shared_ptr<open3d::geometry::PointCloud> target);
Eigen::Matrix4d getIterativePCABasedAlignment(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target);

Eigen::MatrixXd getDepthBufferFromPCD(std::shared_ptr<open3d::geometry::PointCloud> pcd, const open3d::camera::PinholeCameraIntrinsic &intr);
