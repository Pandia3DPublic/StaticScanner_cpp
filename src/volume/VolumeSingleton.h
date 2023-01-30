#pragma once

class VolumeSingleton
{
private:
    /* data */
public:
    VolumeSingleton(/* args */);
    ~VolumeSingleton();

    bool ReadAlignmentDataFromDisk(const std::string &path);

    //reference model transformation variables
    Eigen::Matrix4d T_FirstCamToCAD = Eigen::Matrix4d::Identity();
    double x_offset = 0;
    double y_offset = 0;
    double z_offset = 0;
    double alpha = 0;
    double beta = 0;
    double gamma = 0;
    Eigen::Vector3d centerTranslation = Eigen::Vector3d::Zero();
    Eigen::Vector3d PCDCenterVec = Eigen::Vector3d::Zero();

    std::shared_ptr<open3d::geometry::PointCloud> VolumePcd;
    std::atomic<bool> VolumePcdChanged{false};
};
