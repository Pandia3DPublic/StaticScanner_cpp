#include "calibutil.h"
#include "util.h"
using namespace std;
cv::Matx33f k4aColorCalibtoOpencvMatrix(const k4a::calibration &cal)
{
    const k4a_calibration_intrinsic_parameters_t::_param &i = cal.color_camera_calibration.intrinsics.parameters.param;
    cv::Matx33f camera_matrix = cv::Matx33f::eye();
    camera_matrix(0, 0) = i.fx;
    camera_matrix(1, 1) = i.fy;
    camera_matrix(0, 2) = i.cx;
    camera_matrix(1, 2) = i.cy;
    return camera_matrix;
}
vector<float> k4aColorCalibtoOpencvDistortionCoeffs(const k4a::calibration &cal)
{
    const k4a_calibration_intrinsic_parameters_t::_param &i = cal.color_camera_calibration.intrinsics.parameters.param;
    return {i.k1, i.k2, i.p1, i.p2, i.k3, i.k4, i.k5, i.k6};
}

cv::Matx33f CalibrationIntrToOpenCVMat(CameraWrapper& cam)
{
    cv::Matx33f camera_matrix = cv::Matx33f::eye();
    for (int i =0; i<3; i++){
        for (int j=0;j<3;j++){
            camera_matrix(i,j) = cam.CalibrationIntrinsic.intrinsic_matrix_(i,j);
        }
    }
    return camera_matrix;
}

Eigen::Matrix4d OpencvTransformationToEigen(cv::Matx33d R, cv::Vec3d t)
{
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out(0, 0) = R(0, 0);
    out(0, 1) = R(0, 1);
    out(0, 2) = R(0, 2);
    out(1, 0) = R(1, 0);
    out(1, 1) = R(1, 1);
    out(1, 2) = R(1, 2);
    out(2, 0) = R(2, 0);
    out(2, 1) = R(2, 1);
    out(2, 2) = R(2, 2);
    out(0, 3) = t(0);
    out(1, 3) = t(1);
    out(2, 3) = t(2);
    return out;
}


Eigen::Matrix4d refineRegistrationICP(std::shared_ptr<open3d::geometry::PointCloud> pcd1, 
std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d& init, int maxIterations, double maxCorsDistance)
{
    double goodness;
    return refineRegistrationICP(pcd1,pcd2,init,goodness,maxIterations,maxCorsDistance);
}
Eigen::Matrix4d refineRegistrationICP(std::shared_ptr<open3d::geometry::PointCloud> pcd1, 
std::shared_ptr<open3d::geometry::PointCloud> pcd2, const Eigen::Matrix4d& init,double& goodness, int maxIterations, double maxCorsDistance)
{
    if (!pcd1->HasNormals())
        pcd1->EstimateNormals();
    if (!pcd2->HasNormals())
        pcd2->EstimateNormals();
    if (!pcd1->HasColors())
        cout << "warning pcd1 has no colors!" << endl;
    if (!pcd2->HasColors())
        cout << "warning pcd2 has no colors!" << endl;

    shared_ptr<open3d::geometry::PointCloud> pcd1f = make_shared<open3d::geometry::PointCloud>(*pcd1);
    shared_ptr<open3d::geometry::PointCloud> pcd2f = make_shared<open3d::geometry::PointCloud>(*pcd2);

    //move pcds to the center. ICP works MUCH better this way
    Eigen::Vector3d center1 = pcd1f->GetCenter();
    Eigen::Matrix4d Tcenter1 = Eigen::Matrix4d::Identity();
    Tcenter1.block<3,1>(0,3) = -center1;
    pcd1f->Transform(Tcenter1);
    Eigen::Vector3d center2= pcd2f->GetCenter();
    Eigen::Matrix4d Tcenter2 = Eigen::Matrix4d::Identity();
    Tcenter2.block<3, 1>(0, 3) = -center2;
    pcd2f->Transform(Tcenter2);
    Eigen::Matrix4d newinit = Tcenter2 * init * Tcenter1.inverse();
    // pcd1f->Transform(newinit);
    // open3d::visualization::DrawGeometries({pcd1f, pcd2f, getOrigin()});

    bool withGPU = false;
    if (withGPU)
    {
        // note: gpu variant is instable for bad init transform. need to make sure transform is any good beforehand. cpu variant handles this better.
        // it crashes in first icp iteration in RegistrationICP -> ComputeTransformation -> DecodeAndSolve6x6 : Singular 6x6 linear system detected, tracking failed
        cout << "in gpu icp" << endl;
        open3d::core::Device device("CUDA:0");
        open3d::core::Device host("CPU:0");
        open3d::core::Tensor newinit_t = open3d::core::eigen_converter::EigenMatrixToTensor(newinit).To(device);
        open3d::t::geometry::PointCloud pcd_source_t = open3d::t::geometry::PointCloud::FromLegacy(*pcd1f, open3d::core::Dtype::Float32, device);
        open3d::t::geometry::PointCloud pcd_target_t = open3d::t::geometry::PointCloud::FromLegacy(*pcd2f, open3d::core::Dtype::Float32, device);

        open3d::t::pipelines::registration::ICPConvergenceCriteria crit;
        crit.max_iteration_ = maxIterations;
        auto result = open3d::t::pipelines::registration::RegistrationICP(pcd_source_t, pcd_target_t, maxCorsDistance, newinit_t,
                                                                          open3d::t::pipelines::registration::TransformationEstimationPointToPlane(), crit);
        goodness = (result.fitness_ / result.inlier_rmse_) / (result.fitness_ + 1/result.inlier_rmse_);
        if (isnan(goodness))
        {
            cout << "Warning: Goodness in icp is nan \n";
        }
        Eigen::Matrix4d rtrans = open3d::core::eigen_converter::TensorToEigenMatrixXd(result.transformation_);
        return Tcenter2.inverse() * rtrans * Tcenter1;
    }
    else {
        // cout << "in cpu icp" << endl;
        open3d::pipelines::registration::ICPConvergenceCriteria crit;
        crit.max_iteration_ = maxIterations;
        //colored does not work at all for some reason
        auto result = open3d::pipelines::registration::RegistrationICP(*pcd1f, *pcd2f, maxCorsDistance, newinit, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
        goodness = (result.fitness_ / result.inlier_rmse_) / (result.fitness_ + 1/result.inlier_rmse_);
        if (isnan(goodness)){
            cout << "Warning: Goodness in icp is nan \n";
        }
        return Tcenter2.inverse() * result.transformation_ * Tcenter1;
    }

}

//for this variant to work the pcds must already be centered
Eigen::Matrix4d refineRegistrationICPCenteredPCDs(std::shared_ptr<open3d::geometry::PointCloud> source,
                                                  std::shared_ptr<open3d::geometry::PointCloud> target, double &fitness, 
                                                  const Eigen::Matrix4d &init, int maxIterations, double maxCorsDistance)
{
    if (!source->HasNormals())
        source->EstimateNormals();
    if (!target->HasNormals())
        target->EstimateNormals();
    if (!source->HasColors())
        cout << "warning source pcd has no colors!" << endl;
    if (!target->HasColors())
        cout << "warning target pcd has no colors!" << endl;

    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    crit.max_iteration_ = maxIterations;
    //colored does not work at all for some reason
    auto result = open3d::pipelines::registration::RegistrationICP(*source, *target, maxCorsDistance, init, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
    fitness = result.fitness_;
    // cout << "rsme " <<  result.inlier_rmse_ << endl;
    if (isnan(fitness))
    {
        cout << "Warning: Goodness in icp is nan \n";
    }
    return result.transformation_;
}