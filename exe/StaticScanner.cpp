#include "GUI/StaticScannerGUI.h"
#include "util.h"
#include "calibration/CalibrationSingleton.h"

int main(int argc, char *argv[])
{
    using namespace std;
    readconfig("config.txt");
    if (!checkConfigVariables())
    {
        cout << "Error: Not all config variables could be read. Aborting \n";
        return 0;
    }
    // CameraTypes CameraType = IntelCamera;
    // CameraTypes CameraType = AzureKinect;
    // CameraTypes CameraType = PhoxiScanner;
    CameraTypes CameraType = ZividCamera;
    // CameraTypes CameraType = EnsensoCamera;


    // if (CameraType == IntelCamera)
    // {
    //     g_ColorMinThreshold = 0.015;
    //     g_ColorMaxThreshold = 0.015;
    // }
    // else if (CameraType == EnsensoCamera)
    // {
    //     g_ColorMinThreshold = 0.003;
    //     g_ColorMaxThreshold = 0.005;
    //         g_UseVoxelGrid =false;
    // }
    // else if (CameraType == ZividCamera)
    // {
    //     g_ColorMinThreshold = 0.005;
    //     g_ColorMaxThreshold = 0.005;
    // g_UseVoxelGrid =false;
    // }

    //######### connect to cameras ##########
    shared_ptr<vector<CameraWrapper>> cameras = make_shared<vector<CameraWrapper>>();
    int n_cams = getNumberofConnectedDevices(CameraType);
    // n_cams = 1;
    cout << "Detected " << n_cams << " Cameras! \n";
    if (n_cams == 0)
    {
        cout << "No cameras detected... closing program" << endl;
        return 0;
    }
    for (int i = 0; i < n_cams; i++)
    {
        cameras->push_back(CameraWrapper(CameraType));
        if (!cameras->back().connect(i))
        {
            cout << "Connection failed\n";
            return -1;
        }
        cameras->back().printInfo();
    }
    sort(cameras->begin(), cameras->end(), SortbySerialNumber);
    shared_ptr<ScanningSingleton> ScanningManager = make_shared<ScanningSingleton>(cameras);
    shared_ptr<CalibrationSingleton> CalibrationManager = make_shared<CalibrationSingleton>();

    bool allPositionsFound = true;
    for (auto &cam : *cameras)
    {
        allPositionsFound = cam.ReadCameraPositionFromDisk(ResourceFolderPath + "config/") && allPositionsFound;
        cam.ReadFOVDataFromDisk(ResourceFolderPath + "config/");
        cam.ReadGravityVectorFromDisk(ResourceFolderPath + "config/");
    }
    //########### check extrinsic correctness #############
    if (!allPositionsFound)
    {
        cout << "Please calibrate your cameras, one or more positions are missing \n";
    }
    else
    {
        int IdentityCount = 0;
        for (auto &cam : *cameras)
        {
            if (cam.Extrinsic == Eigen::Matrix4d::Identity())
                IdentityCount++;
        }
        if (IdentityCount == 0) //case one ore more cameras from the front of the list are no longer connected
        {
            cout << "Warning: No camera has an identity extrinsic! Recalculating camera positions according to first cam." << endl;
            Eigen::Matrix4d Mc = cameras->front().Extrinsic.inverse();
            saveMatrixToDisc(ResourceFolderPath + "config/", "correctionMatrix.txt", Mc);
            cameras->front().Extrinsic = Eigen::Matrix4d::Identity(); // set here to have exact values in identity mat
            for (int i = 1; i < cameras->size(); i++)
            {
                cameras->at(i).Extrinsic = Mc * cameras->at(i).Extrinsic;
            }
        }
        if (IdentityCount > 1) //case we reconnected an old camera at the front of the list
        {
            Eigen::Matrix4d Mc = readMatrixFromDisc(ResourceFolderPath + "config/" + "correctionMatrix.txt");
            for (int i = 0; i < cameras->size(); i++)
            {
                cameras->at(i).Extrinsic = Mc * cameras->at(i).Extrinsic;
            }
            Mc = cameras->front().Extrinsic.inverse();
            saveMatrixToDisc(ResourceFolderPath + "config/", "correctionMatrix.txt", Mc);
            cameras->front().Extrinsic = Eigen::Matrix4d::Identity();
            for (int i = 1; i < cameras->size(); i++)
            {
                cameras->at(i).Extrinsic = Mc * cameras->at(i).Extrinsic;
            }
        }
    }

    //############### Start GUI ####################
    StaticScannerGUI GUIInstance(cameras, ScanningManager, CalibrationManager);
    cout << "############# Initializing StaticScanner GUI ###############" << endl;
    GUIInstance.initApp();

    cout << "############# Start Rendering ###############" << endl;
    while (!GUIInstance.getRoot()->endRenderingQueued())
    {
        GUIInstance.getRoot()->renderOneFrame();
        GUIInstance.updateGUIState();
        GUIInstance.updateLogoDimensions();
        GUIInstance.updateLoadingDimensions();
    }
    // GUIInstance.getRoot()->startRendering();
    cout << "############# Closing StaticScanner GUI ###############" << endl;
    for (auto &cam : *cameras)
    {
        cam.disconnect();
    }
    GUIInstance.closeApp();

    return 0;
}