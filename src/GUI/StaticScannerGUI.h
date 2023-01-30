#pragma once
#include "CameraWrapper.h"
#include "calibration/CalibrationSingleton.h"
#include "OgreHardwarePixelBuffer.h"
#include "GUIImage.h"
#include "ScanningSingleton.h"
#include "Plane.h"
enum ProgramStates
{
    REFERENCEMODEL,
    FIELDOFVIEW,
    CAMERAPOSITION,
    ACTIVE
};
//todo split util.cpp
class StaticScannerGUI : public OgreBites::ApplicationContext, public OgreBites::InputListenerChain, public OgreBites::TrayListener
{
public:
    StaticScannerGUI(std::shared_ptr<std::vector<CameraWrapper>> cameras_,
                     std::shared_ptr<ScanningSingleton> ScanningManager_,
                     std::shared_ptr<CalibrationSingleton> CalibrationManger_);
    //####### Basic Variables #####################
    std::shared_ptr<std::vector<CameraWrapper>> Cameras;
    std::shared_ptr<ScanningSingleton> ScanningManager;
    std::shared_ptr<CalibrationSingleton> CalibrationManager;
    //####### Basic Ogre Variables ################
    Ogre::SceneManager *SceneManager;
    OgreBites::TrayManager *SettingsTrayManager; // main, almost always visible
    std::map<int, OgreBites::TrayManager *> TrayManagers;
    Ogre::Camera *OgreCamera;
    OgreBites::CameraMan *CameraMan;
    Ogre::SceneNode *CameraNode;       // to translate camera
    Ogre::SceneNode *CameraTargetNode; // for camera orbit point
    Ogre::LogManager *LogManager;
    std::vector<Ogre::MaterialPtr> PandiaMaterials; //to switch to wireframe and more

    //####### Ogre Functions Override #############
    void initApp();
    void setup();
    void shutdown();
    bool keyPressed(const OgreBites::KeyboardEvent &evt);
    void buttonHit(OgreBites::Button *b);
    void sliderMoved(OgreBites::Slider *s);
    void itemSelected(OgreBites::SelectMenu *sm);

    //################### further functions ################
    void initBackGround();
    void savePcdFunction(std::shared_ptr<open3d::geometry::PointCloud> pcd, bool setAsReferenceModel);

    //####### Reference Model Variables ###############
    std::string modelfolder = "/../resources/models/";
    std::string PcdFolder = "../resources/pcdModels/";
    Ogre::ManualObject *ReferenceModelPointer;   //memory managed by scene manager
    Ogre::ManualObject *ReferenceObjectPointer;  //memory managed by scene manager
    Ogre::ManualObject *ReferenceMissingPointer; //for geometry thats in the cad and visible but not in the scan
    bool ListModels(std::vector<std::string> &modelList, std::vector<std::string> &filenames);
    bool lsmModelInit = false;

    //####### FOV Variables ###############
    Ogre::ManualObject *CroppedPcdPointer; //memory managed by scene manager
    Ogre::ManualObject *FOVCameraObject;   //memory managed by scene manager
    CameraWrapper *CurrentCamera;          //memory is always outside of class
    std::string CurrentCameraSerialNumber;
    void SaveSliderValues(CameraWrapper &cam);
    //Cube + Plane for visible FOV
    Ogre::ManualObject *FOVIndicator;
    Ogre::ManualObject *FOVWireIndicator;
    std::shared_ptr<open3d::geometry::LineSet> o3dWireIndicator = std::make_shared<open3d::geometry::LineSet>();
    std::shared_ptr<open3d::geometry::TriangleMesh> o3dFOVIndicator;
    const float CubeAlpha = 0.05;
    const std::string CubeMaterial = "Pandia/SimpleAlphaFOV";
    std::shared_ptr<std::vector<PandiaPlane>> FOVplanes;
    void initFOV();
    void recordButtonFunction();

    //####### CameraPosition Variables ###############
    std::vector<GUIImage> GUIImages;
    void initGUIImage(GUIImage &g_img, int n, int widht, int height);
    void DisplayFirstCalibrationImages();
    Ogre::ManualObject *MergedPCDPreview; //memory managed by scene manager
    std::vector<Ogre::ManualObject *> CameraMeshes;

    //####### OCMI Variables ###############
    int OCMIScanCount = 0;
    bool ScanRevertable = false;

    //####### ACTIVE Variables ###############
    Ogre::ManualObject *CurrentScanPointer; //memory managed by scene manager

    //####### General Variabless #########
    ProgramStates ProgramState = REFERENCEMODEL;
    ProgramStates LastProgramState = REFERENCEMODEL;
    bool WireframeToggle = true;
    void updateGUIState();
    void updateCameraOrbit(const Eigen::Vector3d &target);
    void HideAll();
    void showCameraModels(const Eigen::Matrix4d &trans = Eigen::Matrix4d::Identity());
    //######### Panida Logo Overlay ################
    Ogre::Overlay *LogoOverlay;             // to show or hide logo
    Ogre::OverlayContainer *LogoContainer;  // to resize logo
    double logoAspectRatio = 200.0 / 415.0; // image height/width
    double logoWidth = 0.15;                // 0.0-1.0
    void initLogo();
    void updateLogoDimensions();
    //########### Loading Symbol ##################
    Ogre::Overlay *LoadingOverlay; // to show or hide
    Ogre::OverlayContainer *LoadingSymbol;
    Ogre::TextAreaOverlayElement *LoadingText;
    float MeasureText(std::string fontName, std::string text, float charHeight);
    double loadingSymbolAR = 512.0 / 512.0;
    double loadingSymbolWidth = 0.03;
    double loadingTextHeight = 0.03;
    void initLoadingStuff();
    void updateLoadingCaption(const std::string &caption);
    void updateLoadingDimensions();
    //########## Camera Models ################
    double CameraModelScaleFactor = 1.0;
    std::string CameraModelPath;
    Ogre::MaterialPtr CameraMaterial;
    std::shared_ptr<open3d::geometry::TriangleMesh> CameraModel;
    void initCameraModel();
    //########## DEBUG Variables ###############
    open3d::geometry::TriangleMesh dbmesh;
    bool firstscan = true;
    int debugcounter = 0;
};
