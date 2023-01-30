#pragma once
#include "CameraWrapper.h"

class CalibrationSingleton
{
public:
    CalibrationSingleton();
    //########## Basic Variables #############
    // float g_SquareLength = 0.0491; //in m.
    // cv::Size g_Pattern{9, 6};  //width, height. Both need to be set.
    std::vector<double> StereoErrors; //error values in stereoCalibrate to get avg


    cv::Mat getImagewithCorners(CameraWrapper &cam);
    bool DetectandSaveCorners(std::shared_ptr<std::vector<CameraWrapper>> cameras, bool fastEnd = true);
    bool DetectandSaveCorners(std::shared_ptr<std::vector<CameraWrapper>> cameras, std::vector<CameraWrapper *> &badCams, bool fastEnd = false);
    void CalibrateCameras(std::shared_ptr<std::vector<CameraWrapper>> cameras);
    Eigen::Matrix4d getTransformBetween(CameraWrapper &cam1, CameraWrapper &cam2);
    void AdjustCameraExposure(std::shared_ptr<std::vector<CameraWrapper>> cameras);
    void ResetCamerasToAutoExposure(std::shared_ptr<std::vector<CameraWrapper>> cameras);
    void SetCameraEmitterOnOff(std::shared_ptr<std::vector<CameraWrapper>> cameras, bool setOn);
    void setGravityVectors(std::shared_ptr<std::vector<CameraWrapper>> cameras);
};
