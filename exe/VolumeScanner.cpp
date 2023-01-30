#include "volume/VolumeGUI.h"
#include "util.h"
#include "calibration/CalibrationSingleton.h"

int main(int argc, char *argv[])
{
    using namespace std;
    // auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1);
    // cout << getVolumeFromTriangleMesh(sphere) << endl;
    readconfig("config.txt");
    if(!checkConfigVariables()){
        cout << "Error: Not all config variables could be read. Aborting \n";
        return 0;
    }
    // system("../resetUSB.sh");
    // CameraTypes CameraType = IntelCamera;
    // CameraTypes CameraType = AzureKinect;
    // CameraTypes CameraType = PhoxiScanner;
    CameraTypes CameraType = EnsensoCamera;
    // CameraTypes CameraType = ZividCamera;

//######### connect to cameras ##########
    shared_ptr<vector<CameraWrapper>> cameras = make_shared<vector<CameraWrapper>>();
    int n_cams = getNumberofConnectedDevices(CameraType);
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
    VolumeGUI GUIInstance(cameras, ScanningManager, CalibrationManager);
    cout << "############# Initializing StaticScanner GUI ###############" << endl;
    GUIInstance.initApp();

    cout << "############# Start Rendering ###############" << endl;
    while (!GUIInstance.getRoot()->endRenderingQueued())
    {
        GUIInstance.getRoot()->renderOneFrame();
        GUIInstance.updateGUIState();
        GUIInstance.updateLogoDimensions();
        GUIInstance.updateLoadingDimensions();
        GUIInstance.updateVolumeVis();
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