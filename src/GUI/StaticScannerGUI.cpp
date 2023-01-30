#include "StaticScannerGUI.h"
#include "guiUtil.h"
#include "util.h"
#include "ColoredGradient2D.h"
#include "calibration/calibutil.h"
#include "OgreFontManager.h"
#include <sstream>
#include <iomanip>
using namespace std;
using namespace Ogre;
using namespace OgreBites;

StaticScannerGUI::StaticScannerGUI(std::shared_ptr<std::vector<CameraWrapper>> cameras_,
                                   std::shared_ptr<ScanningSingleton> ScanningManager_,
                                   std::shared_ptr<CalibrationSingleton> CalibrationManager_) : OgreBites::ApplicationContext("StaticScannerGUI")
{
    Cameras = cameras_;
    ScanningManager = ScanningManager_;
    CalibrationManager = CalibrationManager_;
}

//######## Ogre Functions Override #############
void StaticScannerGUI::initApp()
{
    LogManager = new Ogre::LogManager();
    LogManager->createLog("OgreLog", true, true, true);
    LogManager->getDefaultLog()->setMinLogLevel(Ogre::LML_WARNING);
    OgreBites::ApplicationContextBase::initApp();
}

void StaticScannerGUI::setup()
{
    // do not forget to call the base first
    OgreBites::ApplicationContext::setup();
    // register for input events, necessary for some reason
    addInputListener(this);
    // create sceneManager which handles all attached nodes
    SceneManager = getRoot()->createSceneManager();
    SceneManager->addRenderQueueListener(getOverlaySystem()); // for gui overlay
    // register our scene with the RTSS
    Ogre::RTShader::ShaderGenerator *shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(SceneManager);
    // create a camnode for the cam
    CameraNode = SceneManager->getRootSceneNode()->createChildSceneNode();
    CameraTargetNode = SceneManager->getRootSceneNode()->createChildSceneNode();
    // create the camera
    OgreCamera = SceneManager->createCamera("myCam");
    OgreCamera->setNearClipDistance(0.1);
    OgreCamera->setAutoAspectRatio(true);
    CameraNode->attachObject(OgreCamera);
    // camera man for camera controls, very useful method!
    CameraNode->setPosition(0, 0, 0);
    CameraNode->translate(0, 0, 2.5);
    // CameraNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);
    // CameraNode->setTarg
    CameraMan = new OgreBites::CameraMan(CameraNode);
    CameraMan->setStyle(CS_ORBIT);
    CameraMan->setTarget(CameraTargetNode);
    // updateCameraOrbit(Eigen::Vector3d(0, 0, 0));

    // without light we would just get a black screen
    Ogre::Light *light = SceneManager->createLight("MainLight");
    light->setPowerScale(0.5);
    Ogre::SceneNode *lightNode = CameraNode->createChildSceneNode();
    lightNode->attachObject(light);
    // tell it to render into the main window
    Ogre::Viewport *viewport = getRenderWindow()->addViewport(OgreCamera);
    viewport->setOverlaysEnabled(true);

    SceneManager->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
    // materials, only to change to wireframe atm
    PandiaMaterials.push_back(Ogre::MaterialManager::getSingleton().getByName("Pandia/Simple"));
    PandiaMaterials.push_back(Ogre::MaterialManager::getSingleton().getByName("Pandia/SimpleAlpha"));
    PandiaMaterials.push_back(Ogre::MaterialManager::getSingleton().getByName("Pandia/SimpleGrey"));
    // init pandia logo and loading stuff
    initLogo();
    initLoadingStuff();
    initBackGround();

    // Image rendering components
    Ogre::SceneNode *rectangleNode = SceneManager->getRootSceneNode()->createChildSceneNode();
    int count = 0;
    for (auto &cam : *Cameras)
    {
        GUIImages.emplace_back(GUIImage());
        initGUIImage(GUIImages[count], count, cam.FullCalibrationImageWidth, cam.FullCalibrationImageHeight);
        rectangleNode->attachObject(GUIImages[count].RenderRectangle);
        count++;
    }

    // SettingsTrayManager manages all gui elements that are present in every gui state.
    SettingsTrayManager = new OgreBites::TrayManager("SettingsTrayManager", getRenderWindow(), this);
    SettingsTrayManager->showAll();
    SettingsTrayManager->hideCursor();
    float widgetWidth = 0.14f * getRenderWindow()->getWidth();
    SettingsTrayManager->createLabel(TL_TOPLEFT, "lb_CurrentState", "Status: Soll-Modell", widgetWidth);
    SettingsTrayManager->createLabel(TL_TOP, "lb_Header", "Setup", widgetWidth);
    SettingsTrayManager->createButton(TL_TOPLEFT, "bn_Soll-Modell", "Soll-Modell", widgetWidth);
    SettingsTrayManager->createButton(TL_TOPLEFT, "bn_Sichtfeld Definition", "Sichtfeld Definition", widgetWidth);
    SettingsTrayManager->createButton(TL_TOPLEFT, "bn_Kamera Positionierung", "Kamera Positionserkennung", widgetWidth);
    SettingsTrayManager->createButton(TL_BOTTOMLEFT, "bn_Start", "Scan Modus", widgetWidth);
    // SettingsTrayManager->createDecorWidget(TL_CENTER, "dw_logo", "SdkTrays/Logo");

    // ################## REFERENCEMODEL ########################
    ReferenceModelPointer = SceneManager->createManualObject("ReferenceModel");
    SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(ReferenceModelPointer);
    ReferenceObjectPointer = SceneManager->createManualObject("ReferencePcd");
    SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(ReferenceObjectPointer);
    TrayManagers[REFERENCEMODEL] = new OgreBites::TrayManager("REFERENCEMODEL Tray Manager", getRenderWindow(), this);
    TrayManagers[REFERENCEMODEL]->createLabel(TL_LEFT, "lb_Mesh1", "Modelpfad: resources/models");
    auto filenames = getFileNames(modelfolder);
    TrayManagers[REFERENCEMODEL]->createLongSelectMenu(TL_LEFT, "lsm_Model", "Model", widgetWidth, 0.65 * widgetWidth, 8, filenames);
    dynamic_cast<OgreBites::SelectMenu *>(TrayManagers[REFERENCEMODEL]->getWidget("lsm_Model"))->selectItem(filenames.at(0), true);
    // Reference Scan
    //  TrayManagers[REFERENCEMODEL]->createLabel(TL_LEFT, "lb_ReferenceScan", "Reference Scan", widgetWidth);
    TrayManagers[REFERENCEMODEL]->createButton(TL_LEFT, "bn_ReferenceScan", "Record Reference", widgetWidth);
    TrayManagers[REFERENCEMODEL]->createButton(TL_LEFT, "bn_SaveReferenceScan", "Save Reference", widgetWidth);
    TrayManagers[REFERENCEMODEL]->showAll();
    TrayManagers[REFERENCEMODEL]->hideCursor();

    // ################## FIELDOFVIEW ########################
    auto CameraSerialNumbers = getCameraSerialNumbers(Cameras);
    CurrentCameraSerialNumber = CameraSerialNumbers.at(0);
    CurrentCamera = getCameraBySerialNumber(Cameras, CurrentCameraSerialNumber);
    TrayManagers[FIELDOFVIEW] = new OgreBites::TrayManager("FIELDOFVIEW Tray Manager", getRenderWindow(), this);
    TrayManagers[FIELDOFVIEW]->createLongSelectMenu(TL_LEFT, "lsm_FOV", "Kamera", widgetWidth, 0.65 * widgetWidth, 8, CameraSerialNumbers);
    TrayManagers[FIELDOFVIEW]->createButton(TL_LEFT, "bn_Record", "Record", widgetWidth);
    TrayManagers[FIELDOFVIEW]->createThickSlider(TL_LEFT, "s_CropLeft", "Links", widgetWidth, 50, 0.0f, 100.0f, 101);
    TrayManagers[FIELDOFVIEW]->createThickSlider(TL_LEFT, "s_CropRight", "Rechts", widgetWidth, 50, 0.0f, 100.0f, 101);
    TrayManagers[FIELDOFVIEW]->createThickSlider(TL_LEFT, "s_CropTop", "Oben", widgetWidth, 50, 0.0f, 100.0f, 101);
    TrayManagers[FIELDOFVIEW]->createThickSlider(TL_LEFT, "s_CropBottom", "Unten", widgetWidth, 50, 0.0f, 100.0f, 101);
    TrayManagers[FIELDOFVIEW]->createThickSlider(TL_LEFT, "s_Depth", "Tiefe", widgetWidth, 50, 0.0f, 100.0f, 101);
    TrayManagers[FIELDOFVIEW]->createThickSlider(TL_LEFT, "s_DepthInv", "Min Dist", widgetWidth, 50, 0.0f, 100.0f, 101);
    TrayManagers[FIELDOFVIEW]->createThickSlider(TL_LEFT, "s_Ground", "Kamerahoehe (m)", widgetWidth, 50, 0.0f, 1.5f, 301);
    TrayManagers[FIELDOFVIEW]->createThickSlider(TL_LEFT, "s_GroundFine", "Kamerahoehe Detail (mm)", widgetWidth, 50, -15.0f, 15.0f, 61);
    dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_CropLeft"))->setValue(CurrentCamera->crop_left, false);
    dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_CropRight"))->setValue(CurrentCamera->crop_right, false);
    dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_CropTop"))->setValue(CurrentCamera->crop_top, false);
    dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_CropBottom"))->setValue(CurrentCamera->crop_bottom, false);
    dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_Depth"))->setValue(CurrentCamera->crop_depth, false);
    dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_DepthInv"))->setValue(CurrentCamera->crop_depth_inv, false);
    dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_Ground"))->setValue(CurrentCamera->crop_ground_height, false);
    dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_GroundFine"))->setValue(CurrentCamera->crop_ground_height_fine, false);
    CroppedPcdPointer = SceneManager->createManualObject("CroppedPcdPointer");
    SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(CroppedPcdPointer);
    FOVCameraObject = SceneManager->createManualObject("FOVCameraObject");
    SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(FOVCameraObject);
    initFOV();
    TrayManagers[FIELDOFVIEW]->hideAll();

    // ################## CAMERAPOSITION ########################
    TrayManagers[CAMERAPOSITION] = new OgreBites::TrayManager("CAMERAPOSITION Tray Manager", getRenderWindow(), this);
    TrayManagers[CAMERAPOSITION]->createButton(TL_LEFT, "bn_Positionen Anzeigen", "Positionen Anzeigen", widgetWidth);
    MergedPCDPreview = SceneManager->createManualObject("MergedPCDPreview");
    SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(MergedPCDPreview);
    for (int i = 0; i < Cameras->size(); i++)
    {
        CameraMeshes.emplace_back(SceneManager->createManualObject("CameraMesh" + to_string(i)));
        SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(CameraMeshes[i]);
    }
    TrayManagers[CAMERAPOSITION]->hideAll();

    //################### ACTIVE #################################
    TrayManagers[ACTIVE] = new OgreBites::TrayManager("ACTIVE Tray Manager", getRenderWindow(), this);
    TrayManagers[ACTIVE]->createLabel(TL_TOP, "lb_Headline", "Scan Modus", widgetWidth);
    TrayManagers[ACTIVE]->createButton(TL_BOTTOMLEFT, "bn_Settings", "Setup", widgetWidth);
    TrayManagers[ACTIVE]->createButton(TL_BOTTOMRIGHT, "bn_ZeigeScan", "Zeige Scan", widgetWidth);
    TrayManagers[ACTIVE]->createButton(TL_BOTTOMRIGHT, "bn_ZeigeScanUndCAD", "Zeige Scan & CAD", widgetWidth);
    TrayManagers[ACTIVE]->createButton(TL_BOTTOMRIGHT, "bn_ZeigeExtraTeile", "Zeige extra Teile", widgetWidth);
    TrayManagers[ACTIVE]->createButton(TL_BOTTOMRIGHT, "bn_ZeigeFehlendeTeile", "Zeige fehlende Teile", widgetWidth);
    TrayManagers[ACTIVE]->createButton(TL_BOTTOMRIGHT, "bn_ZeigeDifferenz", "Zeige Differenzen", widgetWidth);
    TrayManagers[ACTIVE]->createButton(TL_BOTTOMRIGHT, "bn_SichtbarkeitZeigen", "Zeige Sichtbarkeit", widgetWidth);
    TrayManagers[ACTIVE]->createButton(TL_BOTTOMRIGHT, "bn_StartScan", "Scan Starten", widgetWidth);
    TrayManagers[ACTIVE]->createButton(TL_RIGHT, "bn_SavePcd", "Scan Speichern", widgetWidth);
    // OCMI
    //  TrayManagers[ACTIVE]->createLabel(TL_LEFT, "lb_OCMI", "One Cam Multiple Images (OCMI)", widgetWidth);
    //  TrayManagers[ACTIVE]->createLongSelectMenu(TL_LEFT, "lsm_OCMICamera", "Kamera", widgetWidth, 0.65 * widgetWidth, 8, CameraSerialNumbers);
    //  TrayManagers[ACTIVE]->createButton(TL_LEFT, "bn_OCMIScan", "Scan", widgetWidth);
    //  TrayManagers[ACTIVE]->createButton(TL_LEFT, "bn_OCMIRevertLastScan", "Revert Last Scan", widgetWidth);
    //  TrayManagers[ACTIVE]->createButton(TL_LEFT, "bn_OCMIReset", "Reset", widgetWidth);
    // Threshold
    TrayManagers[ACTIVE]->createThickSlider(TL_RIGHT, "s_ColorSlider", "Fehlertoleranz (mm)", widgetWidth, 50, 0.0f, 30.0f, 61);
    dynamic_cast<OgreBites::Slider *>(TrayManagers[ACTIVE]->getWidget("s_ColorSlider"))->setValue(g_ColorMaxThreshold * 1000, false);

    CurrentScanPointer = SceneManager->createManualObject("ScannedObject");
    SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(CurrentScanPointer);
    ReferenceMissingPointer = SceneManager->createManualObject("CADandMissing");
    SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(ReferenceMissingPointer);

    TrayManagers[ACTIVE]->hideAll();

    // add input listeners
    mListenerChain = {TrayManagers[ProgramState], SettingsTrayManager, CameraMan};
    // init Camera Model
    initCameraModel();
}

void StaticScannerGUI::buttonHit(OgreBites::Button *b)
{
    // note switch case only works for integral types so we need stupid if/else statements
    if (b->getName() == "bn_Soll-Modell")
    {
        LastProgramState = ProgramState;
        dynamic_cast<OgreBites::Label *>(SettingsTrayManager->getWidget("lb_CurrentState"))->setCaption("Status: " + b->getCaption());
        ProgramState = REFERENCEMODEL;
        HideAll();
        ReferenceModelPointer->setVisible(true);
        updateCameraOrbit(Eigen::Vector3d(0, 0, 0));
    }
    else if (b->getName() == "bn_Sichtfeld Definition")
    {
        LastProgramState = ProgramState;
        dynamic_cast<OgreBites::Label *>(SettingsTrayManager->getWidget("lb_CurrentState"))->setCaption("Status: " + b->getCaption());
        ProgramState = FIELDOFVIEW;
        HideAll();
        CroppedPcdPointer->setVisible(true);
        FOVCameraObject->setVisible(true);
        // if(CroppedPcdPointer->getSections() != 0){
        FOVIndicator->setVisible(true);
        FOVIndicator->setVisible(true);
        // }
        updateCameraOrbit(Eigen::Vector3d(0, 0, 0));
        auto lsmFOV = dynamic_cast<OgreBites::SelectMenu *>(TrayManagers[FIELDOFVIEW]->getWidget("lsm_FOV"));
        lsmFOV->selectItem(lsmFOV->getItems()[0], true);
    }
    else if (b->getName() == "bn_Kamera Positionierung")
    {
        HideAll();
        for (auto &img : GUIImages)
        {
            img.RenderRectangle->setVisible(true);
        }
        LastProgramState = ProgramState;
        ProgramState = CAMERAPOSITION;
        dynamic_cast<OgreBites::Label *>(SettingsTrayManager->getWidget("lb_CurrentState"))->setCaption("Status: " + b->getCaption());
        // turn off emitter for clean infrared calibration images which are needed for chessboard corner detection
        CalibrationManager->SetCameraEmitterOnOff(Cameras, false);
        CalibrationManager->AdjustCameraExposure(Cameras);
        //################# using the optimal exposure do the actual calibration ########################
        for (auto &cam : *Cameras)
        {
            cam.clearBuffers();
            if (cam.CameraType == PhoxiScanner || cam.CameraType == ZividCamera)
            {
                cam.record(1, 0, false);
            }
            else
            {
                cam.record(5, 5, false);
            }
            cam.AlignUndistortandConvertO3D();
        }
        // display images
        DisplayFirstCalibrationImages(); // takes long if no corners are found
        // perform stereo calibration
        // 1.Search for corners in all images and save them inside the cameras
        bool foundAllCorners = CalibrationManager->DetectandSaveCorners(Cameras);
        // 2.Pass these to the actual calibrate function and save the extrinsic inside the cameras
        if (foundAllCorners)
        {
            CalibrationManager->CalibrateCameras(Cameras);
            CalibrationManager->setGravityVectors(Cameras);
            double err = 0;
            for (auto &e : CalibrationManager->StereoErrors)
            {
                err += e;
            }
            err /= CalibrationManager->StereoErrors.empty() ? 1 : CalibrationManager->StereoErrors.size();
            cout << "Stereo calibrate avg error: " << err << endl;
        }
        else
        {
            cout << "Calibration failed \n";
        }
        CalibrationManager->StereoErrors.clear();
        CalibrationManager->ResetCamerasToAutoExposure(Cameras);
        CalibrationManager->SetCameraEmitterOnOff(Cameras, true);
        // record again with emitter for pcd
        for (auto &cam : *Cameras)
        {
            cam.clearBuffers();
            cam.record(1, 0);
            cam.AlignUndistortandConvertO3D();
        }
    }
    else if (b->getName() == "bn_Positionen Anzeigen")
    {
        HideAll();
        shared_ptr<open3d::geometry::PointCloud> mergedpcd = make_shared<open3d::geometry::PointCloud>();
        for (int i = 0; i < Cameras->size(); i++)
        {
            auto &cam = Cameras->at(i);
            auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*cam.rgbdImages[0], cam.DepthLDTIntrinsic, cam.Extrinsic.inverse());
            *mergedpcd = *mergedpcd + *pcd;
        }
        showCameraModels();
        o3dToOgrePcd(MergedPCDPreview, mergedpcd, "Pandia/Simple", getflip3());
        MergedPCDPreview->setVisible(true);
        updateCameraOrbit(-mergedpcd->GetCenter());
        cout << "showing positions\n";
    }
    else if (b->getName() == "bn_Record")
    {
        recordButtonFunction();
    }
    else if (b->getName() == "bn_Start")
    {
        LastProgramState = ProgramState;
        ProgramState = ACTIVE;
        dynamic_cast<OgreBites::Label *>(TrayManagers[ACTIVE]->getWidget("lb_Headline"))->setCaption(b->getCaption());
        HideAll();
    }
    else if (b->getName() == "bn_ZeigeScan")
    {
        HideAll();
        ScanningManager->RecolorCurrentPCD();
        o3dToOgrePcd(CurrentScanPointer, ScanningManager->CurrentPCD, "Pandia/SimpleNoLight", getflip3());
        dynamic_cast<OgreBites::Label *>(TrayManagers[ACTIVE]->getWidget("lb_Headline"))->setCaption(b->getCaption());
        CurrentScanPointer->setVisible(true);
    }
    else if (b->getName() == "bn_ZeigeScanUndCAD")
    {
        HideAll();
        dynamic_cast<OgreBites::Label *>(TrayManagers[ACTIVE]->getWidget("lb_Headline"))->setCaption(b->getCaption());
        ScanningManager->RecolorCurrentPCD();
        o3dToOgrePcd(CurrentScanPointer, ScanningManager->CurrentPCD, "Pandia/SimpleNoLight", getflip3());
        if (ScanningManager->isReferenceMesh)
            o3dToOgreMesh(ReferenceObjectPointer, ScanningManager->ReferenceMesh, "Pandia/SimpleAlpha", getflip3(), 0.7);
        else
            o3dToOgrePcd(ReferenceObjectPointer, ScanningManager->ReferencePCD, "Pandia/SimpleAlpha", getflip3(), 0.7);
        CurrentScanPointer->setVisible(true);
        ReferenceObjectPointer->setVisible(true);
    }
    else if (b->getName() == "bn_ZeigeDifferenz")
    {
        HideAll();
        dynamic_cast<OgreBites::Label *>(TrayManagers[ACTIVE]->getWidget("lb_Headline"))->setCaption(b->getCaption());
        // ScanningManager->ColorCurrentPCDwithDistance();
        ScanningManager->ColorCurrentPCDGradient();
        shared_ptr<open3d::geometry::PointCloud> missingPoints = ScanningManager->getMissingPoints();
        missingPoints->Transform(ScanningManager->CurrentToReferenceTransform);
        missingPoints->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
        o3dToOgrePcd(CurrentScanPointer, ScanningManager->CurrentPCD, "Pandia/SimpleNoLight", getflip3());
        // o3dToOgrePcd(CurrentScanPointer, scannedAndProjectedPoints, "Pandia/SimpleNoLight", getflip3());
        o3dToOgreMesh(ReferenceObjectPointer, ScanningManager->ReferenceMesh, "Pandia/SimpleAlpha", getflip3(), 0.7);
        o3dToOgrePcd(ReferenceMissingPointer, missingPoints, "Pandia/SimpleNoLight", getflip3()); // blue
        CurrentScanPointer->setVisible(true);
        ReferenceObjectPointer->setVisible(true);
        ReferenceMissingPointer->setVisible(true);
    }
    else if (b->getName() == "bn_SichtbarkeitZeigen")
    {
        HideAll();
        ScanningManager->ColorFOV();
        showCameraModels(ScanningManager->CurrentToReferenceTransform);
        o3dToOgrePcd(ReferenceObjectPointer, ScanningManager->ReferencePCD, "Pandia/SimpleNoLight", getflip3());
        dynamic_cast<OgreBites::Label *>(TrayManagers[ACTIVE]->getWidget("lb_Headline"))->setCaption(b->getCaption());
        ReferenceObjectPointer->setVisible(true);
    }
    else if (b->getName() == "bn_ZeigeFehlendeTeile")
    {
        HideAll();
        dynamic_cast<OgreBites::Label *>(TrayManagers[ACTIVE]->getWidget("lb_Headline"))->setCaption(b->getCaption());
        // ScanningManager->ColorCurrentPCDwithDistance();


        shared_ptr<open3d::geometry::PointCloud> missingPoints = ScanningManager->getMissingPoints();
        // open3d::visualization::DrawGeometries({missingPoints});
        missingPoints->Transform(ScanningManager->CurrentToReferenceTransform);
        missingPoints->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
        if (ScanningManager->isReferenceMesh)
            o3dToOgreMesh(ReferenceObjectPointer, ScanningManager->ReferenceMesh, "Pandia/SimpleAlpha", getflip3(), 0.7);
        else
            o3dToOgrePcd(ReferenceObjectPointer, ScanningManager->ReferencePCD, "Pandia/SimpleAlpha", getflip3(), 0.7);
        o3dToOgrePcd(ReferenceMissingPointer, missingPoints, "Pandia/SimpleNoLight", getflip3()); //blue

        ReferenceMissingPointer->setRenderQueueGroupAndPriority(RENDER_QUEUE_9, 100);

        ReferenceObjectPointer->setVisible(true);
        ReferenceMissingPointer->setVisible(true);

    }
    else if (b->getName() == "bn_ZeigeExtraTeile")
    {
        HideAll();
        dynamic_cast<OgreBites::Label *>(TrayManagers[ACTIVE]->getWidget("lb_Headline"))->setCaption(b->getCaption());
        // ScanningManager->ColorCurrentPCDwithDistance();
        ScanningManager->ColorCurrentPCDGradient();
        if (ScanningManager->isReferenceMesh)
            o3dToOgreMesh(ReferenceObjectPointer, ScanningManager->ReferenceMesh, "Pandia/SimpleAlpha", getflip3(), 0.7);
        else
            o3dToOgrePcd(ReferenceObjectPointer, ScanningManager->ReferencePCD, "Pandia/SimpleAlpha", getflip3(), 0.7);
        o3dToOgrePcd(CurrentScanPointer, ScanningManager->CurrentPCD, "Pandia/SimpleNoLight", getflip3());
        ReferenceObjectPointer->setVisible(true);
        CurrentScanPointer->setVisible(true);
    }

    else if (b->getName() == "bn_StartScan")
    {
        PandiaTimer ptStartScan;
        ScanningManager->reset();
        LoadingOverlay->show(); // loading bar
        //############# various variables ###################
        open3d::pipelines::integration::ScalableTSDFVolume vg(0.004, 0.015, open3d::pipelines::integration::TSDFVolumeColorType::RGB8, 16, 1);
        shared_ptr<open3d::geometry::PointCloud> vgPCD;
        //######## Record and filter rgbd images #############
        int recordCount = 1;
        for (auto &cam : *Cameras)
        {
            updateLoadingCaption("Recording pointclouds (" + to_string(recordCount++) + " / " + to_string(Cameras->size()) + ")");
            cam.clearBuffers();
            if (cam.CameraType == PhoxiScanner || cam.CameraType == ZividCamera)
            {
                if (!cam.record(1, 0, false))
                {
                    cout << "recording failed \n";
                    LoadingOverlay->hide();
                    return;
                }
            }
            else
            {
                if (!cam.record(10, 0))
                {
                    cout << "recording failed \n";
                    LoadingOverlay->hide();
                    return;
                }
            }
            // todo remove other checks
            cam.AlignUndistortandConvertO3D();
            cam.CropRGBDImages();
            // cam.showOpen3dImages();
            //############ apply single frame rgbd filters ################
            // for (auto &rgbd : cam.rgbdImages)
            // {
            // applyNormalFilter(rgbd, cam.DepthLDTIntrinsic); //todo redundant pcd normal calc
            // applyEdgeColorPersistenceFilter(rgbd);
            // }
            //############# combine frames #######################
            shared_ptr<open3d::geometry::RGBDImage> combinedRgbd;
            if (cam.CameraType == IntelCamera)
            {
                combinedRgbd = getSoftAverageImgFilter(cam.rgbdImages);
            }
            else
            {
                combinedRgbd = getAverageImg(cam.rgbdImages);
            }
            // auto combinedRgbd = getAverageImgFilter(cam.rgbdImages);
            applyNormalFilter(combinedRgbd, cam.DepthLDTIntrinsic); // todo redundant pcd normal calc
            cam.ScannedRGBD = combinedRgbd;
            if (cam.CameraType == PhoxiScanner || cam.CameraType == ZividCamera)
            {
                cam.ScannedPcd = cam.Pcds.front(); // use undistorted camera pcd instead of distorted rgbd images
                *cam.ScannedPcd = *cam.getCroppedPcd(cam.ScannedPcd);
            }
            else
            {
                cam.ScannedPcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*combinedRgbd, cam.DepthLDTIntrinsic);
            }
            if (cam.ScannedPcd->IsEmpty())
            {
                std::cout << "Error: Pcd of cam " << cam.SerialNumber << " is empty! Aborting" << endl;
                LoadingOverlay->hide();
                return;
            }
            if (!cam.ScannedPcd->HasNormals())
            {
                cam.ScannedPcd->EstimateNormals();
                cam.ScannedPcd->OrientNormalsTowardsCameraLocation();
            }
            // open3d::visualization::DrawGeometries({getOrigin(),cam.ScannedPcd});
            //########### Apply pcd filters ##################
            // applyNormalFilter(cam.Pcds.front());

            //################# get center pcd and CurrentToRefCenterVector #######################
            auto transformed = *cam.ScannedPcd;
            transformed.Transform(cam.Extrinsic);
            *ScanningManager->CurrentPCD = *ScanningManager->CurrentPCD + transformed;
        }
        ScanningManager->CurrentDownsampledPCD = ScanningManager->CurrentPCD->VoxelDownSample(0.005);
        updateLoadingCaption("Counting points in pointclouds");
        updateLoadingCaption("Shuffling points around");

        //############ Align Scan with CAD Modell ##########################
        // open3d::visualization::DrawGeometries({getOrigin(),ScanningManager->CurrentDownsampledPCD});

        // we build an initial transformation that roughly alings the currentpcd with the reference pcd.
        Eigen::Matrix4d T_init = getIterativePCABasedAlignment(ScanningManager->CurrentDownsampledPCD, ScanningManager->SmallReferencePCD);
        ScanningManager->CurrentToReferenceTransform = refineRegistrationICP(ScanningManager->CurrentPCD, ScanningManager->SmallReferencePCD, T_init, 220, 0.01);
        ScanningManager->CurrentPCD->Transform(ScanningManager->CurrentToReferenceTransform);

        //############# set camera transformations #################
        for (auto &cam : *Cameras)
        {
            cam.CameraToReferenceTrans = ScanningManager->CurrentToReferenceTransform * cam.Extrinsic;
        }
        //########### further options #####################

        if (g_RefineAlignWithCAD)
        {
            ScanningManager->CurrentPCD->Clear();
            for (auto &cam : *Cameras)
            {
                Eigen::Matrix4d T = ScanningManager->CurrentToReferenceTransform * cam.Extrinsic;
                cam.CameraToReferenceTrans = refineRegistrationICP(cam.ScannedPcd, ScanningManager->SmallReferencePCD, T, 60, 0.015);
                if (!g_UseVoxelGrid)
                {
                    auto transformed = *cam.ScannedPcd;
                    transformed.Transform(cam.CameraToReferenceTrans);
                    *ScanningManager->CurrentPCD = *ScanningManager->CurrentPCD + transformed;
                }
            }
        }

        // ############ integrate into vg ###################
        if (g_UseVoxelGrid)
        {
            for (auto &cam : *Cameras)
            {
                vg.Integrate(*cam.ScannedRGBD, cam.DepthLDTIntrinsic, cam.CameraToReferenceTrans.inverse());
            }
            ScanningManager->CurrentPCD = vg.ExtractPointCloud();
        }
        // ScanningManager->FilterCurrentPcdUsingMulitCamVisibility();

        // ############ display ###################

        updateLoadingCaption("Cleaning up");
        ScanningManager->CurrentPCDColors = ScanningManager->CurrentPCD->colors_;
        // 6. Measure distances of each scanpoint to reference model
        ScanningManager->ComputeDistanceToReference();
        // 7. Apply Color
        // ScanningManager->ColorCurrentPCDwithDistance();
        // showCameraModels();//todo wrong position
        CurrentScanPointer->setRenderQueueGroupAndPriority(RENDER_QUEUE_9, 100);
        o3dToOgrePcd(CurrentScanPointer, ScanningManager->CurrentPCD, "Pandia/Simple", getflip3());
        if (ScanningManager->isReferenceMesh)
        {
            updateCameraOrbit(-ScanningManager->ReferenceMesh->GetCenter());
            o3dToOgreMesh(ReferenceObjectPointer, ScanningManager->ReferenceMesh, "Pandia/SimpleAlpha", getflip3(), 0.7);
        }
        else
        {
            updateCameraOrbit(-ScanningManager->ReferencePCD->GetCenter());
            o3dToOgrePcd(ReferenceObjectPointer, ScanningManager->ReferencePCD, "Pandia/SimpleAlpha", getflip3(), 0.7);
        }
        cout << "Scan time was " << ptStartScan.seconds() << " seconds." << endl;
        CurrentScanPointer->setVisible(true);
        ReferenceObjectPointer->setVisible(true);
        LoadingOverlay->hide();
    }
    // todo work in progress
    else if (b->getName() == "bn_OCMIScan")
    {
        if (OCMIScanCount == 0)
            ScanningManager->reset();
        // currentpcd is complete scan
        // scannedpcds get added to currentpcd
        CameraWrapper &cam = *CurrentCamera;
        cam.clearBuffers();
        cam.record(1, 0);
        cam.AlignUndistortandConvertO3D();
        cam.CropRGBDImages();
        //############# combine frames #######################
        auto combinedRgbd = getSoftAverageImgFilter(cam.rgbdImages);
        applyNormalFilter(combinedRgbd, cam.DepthLDTIntrinsic); // todo redundant pcd normal calc
        cam.ScannedRGBD = combinedRgbd;
        if (cam.CameraType == PhoxiScanner || cam.CameraType == ZividCamera)
        {
            cam.ScannedPcd = cam.Pcds.front(); // use undistorted camera pcd instead of distorted rgbd images
            *cam.ScannedPcd = *cam.getCroppedPcd(cam.ScannedPcd);
        }
        else
        {
            cam.ScannedPcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*combinedRgbd, cam.DepthLDTIntrinsic);
        }
        if (cam.ScannedPcd->IsEmpty())
        {
            std::cout << "Error: Pcd of cam " << cam.SerialNumber << " is empty! Aborting" << endl;
            LoadingOverlay->hide();
            return;
        }
        if (!cam.ScannedPcd->HasNormals())
        {
            cam.ScannedPcd->EstimateNormals();
            cam.ScannedPcd->OrientNormalsTowardsCameraLocation();
        }

        // #################### transformation stuff ###########################
        auto ScannedPcdSmall = cam.ScannedPcd->VoxelDownSample(0.005);
        if (OCMIScanCount == 0)
        {
            *ScanningManager->CurrentPCD = *cam.ScannedPcd;
            *ScanningManager->CurrentDownsampledPCD = *ScannedPcdSmall;
            updateCameraOrbit(-ScanningManager->CurrentPCD->GetCenter());
        }
        if (OCMIScanCount > 0)
        {
            auto transformed = make_shared<open3d::geometry::PointCloud>(*cam.ScannedPcd);
            // find out transformation here
            // we build an initial transformation that roughly alings the scanned pcd with the current pcd.
            Eigen::Matrix4d T_init = getIterativePCABasedAlignment(ScannedPcdSmall, ScanningManager->CurrentDownsampledPCD);
            Eigen::Matrix4d T_sparse = refineRegistrationICP(ScannedPcdSmall, ScanningManager->CurrentDownsampledPCD, T_init, 220, 0.015);
            Eigen::Matrix4d T_dense = refineRegistrationICP(cam.ScannedPcd, ScanningManager->CurrentPCD, T_sparse, 60, 0.01);
            cout << T_dense << endl;
            if (OCMIScanCount == 1)
                transformed->PaintUniformColor(Eigen::Vector3d(0, 0, 1));
            if (OCMIScanCount == 2)
                transformed->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
            if (OCMIScanCount == 3)
                transformed->PaintUniformColor(Eigen::Vector3d(1, 1, 0));
            if (OCMIScanCount == 4)
                transformed->PaintUniformColor(Eigen::Vector3d(0, 1, 1));
            if (OCMIScanCount == 5)
                transformed->PaintUniformColor(Eigen::Vector3d(1, 0, 1));

            transformed->Transform(T_dense);
            // open3d::visualization::DrawGeometries({ScanningManager->CurrentPCD, transformed, getOrigin()});

            *ScanningManager->OldPCD = *ScanningManager->CurrentPCD;                       // to revert last scan
            *ScanningManager->OldDownsampledPCD = *ScanningManager->CurrentDownsampledPCD; // to revert last scan
            *ScanningManager->CurrentPCD = *ScanningManager->CurrentPCD + *transformed;
            // *ScanningManager->CurrentDownsampledPCD = *ScanningManager->CurrentDownsampledPCD + *transformed->VoxelDownSample(0.005);
            *ScanningManager->CurrentDownsampledPCD = *ScanningManager->CurrentPCD->VoxelDownSample(0.005);
        }
        cout << "CurrentPCD size " << ScanningManager->CurrentPCD->points_.size() << endl;

        // visualize
        // open3d::visualization::DrawGeometries({ScanningManager->CurrentPCD, getOrigin()});
        CurrentScanPointer->setRenderQueueGroupAndPriority(RENDER_QUEUE_9, 100);
        o3dToOgrePcd(CurrentScanPointer, ScanningManager->CurrentPCD, "Pandia/SimpleNoLight", getflip3());
        CurrentScanPointer->setVisible(true);
        ScanRevertable = true;
        OCMIScanCount++; // comment out for reference scan use
    }
    else if (b->getName() == "bn_OCMIRevertLastScan")
    {
        if (OCMIScanCount > 0 && ScanRevertable)
        {
            OCMIScanCount--;
            ScanRevertable = false;
            *ScanningManager->CurrentPCD = *ScanningManager->OldPCD;
            *ScanningManager->CurrentDownsampledPCD = *ScanningManager->OldDownsampledPCD;
            o3dToOgrePcd(CurrentScanPointer, ScanningManager->CurrentPCD, "Pandia/SimpleNoLight", getflip3());
            CurrentScanPointer->setVisible(true);
        }
    }
    else if (b->getName() == "bn_OCMIReset")
    {
        OCMIScanCount = 0;
        HideAll();
        ScanningManager->reset();
        for (auto &cam : *Cameras)
        {
            cam.clearBuffers();
        }
    }
    else if (b->getName() == "bn_SavePcd")
    {
        savePcdFunction(ScanningManager->CurrentPCD, false);
    }
    else if (b->getName() == "bn_Settings")
    {
        HideAll();
        ReferenceModelPointer->setVisible(true);
        LastProgramState = ProgramState;
        ProgramState = REFERENCEMODEL;
        updateCameraOrbit(Eigen::Vector3d(0, 0, 0));
    }
    else if (b->getName() == "bn_ReferenceScan")
    {
        ScanningManager->reset();
        LoadingOverlay->show(); // loading bar
        //############# various variables ###################
        open3d::pipelines::integration::ScalableTSDFVolume vg(0.004, 0.015, open3d::pipelines::integration::TSDFVolumeColorType::RGB8, 16, 1);
        shared_ptr<open3d::geometry::PointCloud> vgPCD;
        //######## Record and filter rgbd images #############
        int recordCount = 1;
        for (auto &cam : *Cameras)
        {
            updateLoadingCaption("Recording pointclouds (" + to_string(recordCount++) + " / " + to_string(Cameras->size()) + ")");
            cam.clearBuffers();
            if (cam.CameraType == PhoxiScanner || cam.CameraType == ZividCamera)
            {
                if (!cam.record(1, 0, false))
                {
                    cout << "recording failed \n";
                    LoadingOverlay->hide();
                    return;
                }
            }
            else
            {
                if (!cam.record(10, 0))
                {
                    cout << "recording failed \n";
                    LoadingOverlay->hide();
                    return;
                }
            }
            // todo remove other checks
            cam.AlignUndistortandConvertO3D();
            cam.CropRGBDImages();
            // cam.showOpen3dImages();
            //############ apply single frame rgbd filters ################
            // for (auto &rgbd : cam.rgbdImages)
            // {
            // applyNormalFilter(rgbd, cam.DepthLDTIntrinsic); //todo redundant pcd normal calc
            // applyEdgeColorPersistenceFilter(rgbd);
            // }
            //############# combine frames #######################
            shared_ptr<open3d::geometry::RGBDImage> combinedRgbd;
            if (cam.CameraType == IntelCamera)
            {
                combinedRgbd = getSoftAverageImgFilter(cam.rgbdImages);
            }
            else
            {
                combinedRgbd = getAverageImg(cam.rgbdImages);
            }
            // auto combinedRgbd = getAverageImgFilter(cam.rgbdImages);
            applyNormalFilter(combinedRgbd, cam.DepthLDTIntrinsic); // todo redundant pcd normal calc
            cam.ScannedRGBD = combinedRgbd;
            if (cam.CameraType == PhoxiScanner || cam.CameraType == ZividCamera)
            {
                cam.ScannedPcd = cam.Pcds.front(); // use undistorted camera pcd instead of distorted rgbd images
                *cam.ScannedPcd = *cam.getCroppedPcd(cam.ScannedPcd);
            }
            else
            {
                cam.ScannedPcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*combinedRgbd, cam.DepthLDTIntrinsic);
            }
            if (cam.ScannedPcd->IsEmpty())
            {
                std::cout << "Error: Pcd of cam " << cam.SerialNumber << " is empty! Aborting" << endl;
                LoadingOverlay->hide();
                return;
            }
            if (!cam.ScannedPcd->HasNormals())
            {
                cam.ScannedPcd->EstimateNormals();
                cam.ScannedPcd->OrientNormalsTowardsCameraLocation();
            }
            // open3d::visualization::DrawGeometries({getOrigin(),cam.ScannedPcd});
            //########### Apply pcd filters ##################
            // applyNormalFilter(cam.Pcds.front());

            //################# get center pcd and CurrentToRefCenterVector #######################
            auto transformed = *cam.ScannedPcd;
            transformed.Transform(cam.Extrinsic);
            *ScanningManager->CurrentPCD = *ScanningManager->CurrentPCD + transformed;
        }
        ScanningManager->CurrentDownsampledPCD = ScanningManager->CurrentPCD->VoxelDownSample(0.005);
        updateLoadingCaption("Shuffling points around");

        // ############ integrate into vg ###################
        if (!(Cameras->front().CameraType == ZividCamera || Cameras->front().CameraType == PhoxiScanner))
        {
            for (auto &cam : *Cameras)
            {
                vg.Integrate(*cam.ScannedRGBD, cam.DepthLDTIntrinsic, cam.Extrinsic);
            }
            ScanningManager->CurrentPCD = vg.ExtractPointCloud();
        }

        // ScanningManager->FilterCurrentPcdUsingMulitCamVisibility();

        // Center Pcd
        ScanningManager->CurrentPCD->Translate(-ScanningManager->CurrentPCD->GetCenter());

        // ############ display ###################
        updateLoadingCaption("Cleaning up");
        ScanningManager->CurrentPCDColors = ScanningManager->CurrentPCD->colors_;
        CurrentScanPointer->setRenderQueueGroupAndPriority(RENDER_QUEUE_9, 100);
        o3dToOgrePcd(CurrentScanPointer, ScanningManager->CurrentPCD, "Pandia/Simple", getflip3());
        updateCameraOrbit(-ScanningManager->CurrentPCD->GetCenter());
        HideAll();
        CurrentScanPointer->setVisible(true);
        LoadingOverlay->hide();
    }
    else if (b->getName() == "bn_SaveReferenceScan")
    {
        savePcdFunction(ScanningManager->CurrentPCD, true);
    }
}

void StaticScannerGUI::sliderMoved(OgreBites::Slider *s)
{
    if (s->getName() == "s_CropLeft")
    {
        CurrentCamera->crop_left = s->getValue();
    }
    else if (s->getName() == "s_CropRight")
    {
        CurrentCamera->crop_right = s->getValue();
    }
    else if (s->getName() == "s_CropTop")
    {
        CurrentCamera->crop_top = s->getValue();
    }
    else if (s->getName() == "s_CropBottom")
    {
        CurrentCamera->crop_bottom = s->getValue();
    }
    else if (s->getName() == "s_Depth")
    {
        CurrentCamera->crop_depth = s->getValue();
    }
    else if (s->getName() == "s_DepthInv")
    {
        CurrentCamera->crop_depth_inv = s->getValue();
    }
    else if (s->getName() == "s_Ground")
    {
        CurrentCamera->crop_ground_height = s->getValue();
    }
    else if (s->getName() == "s_GroundFine")
    {
        CurrentCamera->crop_ground_height_fine = s->getValue();
    }
    else if (s->getName() == "s_ColorSlider")
    {
        g_ColorMaxThreshold = s->getValue() / 1000;
        // ScanningManager->ColorCurrentPCDwithDistance();
        ScanningManager->ColorCurrentPCDGradient();
        o3dToOgrePcd(CurrentScanPointer, ScanningManager->CurrentPCD, "Pandia/SimpleNoLight", getflip3());
        CurrentScanPointer->setVisible(true);
        ReferenceObjectPointer->setVisible(true);
    }

    if (s->getName() != "s_ColorSlider")
    {
        SaveSliderValues(*CurrentCamera);
        auto pcd = CurrentCamera->getCroppedPcd(CurrentCamera->DownsampledPcd);
        if (pcd != nullptr)
            o3dToOgrePcd(CroppedPcdPointer, pcd, "Pandia/SimpleNoLight", getflip3());
        o3dToOgreMesh(FOVIndicator, getFOVGeometryfromPlanes(CurrentCamera->getFOVPlanes(), o3dWireIndicator), CubeMaterial, getflip3(), CubeAlpha);
        o3dToOgreLineList(FOVWireIndicator, o3dWireIndicator, "Pandia/SimpleAlphaWire", getflip3(), 1);
    }
}

void StaticScannerGUI::itemSelected(OgreBites::SelectMenu *sm)
{
    if (sm->getName() == "lsm_FOV")
    {
        CurrentCameraSerialNumber = (std::string)sm->getSelectedItem();
        CurrentCamera = getCameraBySerialNumber(Cameras, CurrentCameraSerialNumber);
        dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_CropLeft"))->setValue(CurrentCamera->crop_left, false);
        dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_CropRight"))->setValue(CurrentCamera->crop_right, false);
        dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_CropTop"))->setValue(CurrentCamera->crop_top, false);
        dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_CropBottom"))->setValue(CurrentCamera->crop_bottom, false);
        dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_Depth"))->setValue(CurrentCamera->crop_depth, false);
        dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_DepthInv"))->setValue(CurrentCamera->crop_depth_inv, false);
        dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_Ground"))->setValue(CurrentCamera->crop_ground_height, false);
        dynamic_cast<OgreBites::Slider *>(TrayManagers[FIELDOFVIEW]->getWidget("s_GroundFine"))->setValue(CurrentCamera->crop_ground_height_fine, false);
        recordButtonFunction();
    }
    if (sm->getName() == "lsm_Model")
    {
        ScanningManager->ReferenceMesh = make_shared<open3d::geometry::TriangleMesh>();
        if (lsmModelInit)
        {
            HideAll();
            ReferenceModelPointer->setVisible(true);
        }
        lsmModelInit = true;
        string referenceName = (std::string)sm->getSelectedItem();
        string referenceFileType = getExtensionFromFilename(referenceName);
        if (referenceFileType == "obj")
        {
            ScanningManager->isReferenceMesh = true;
            auto o3dmesh = readMesh(get_current_dir_name() + modelfolder + (std::string)sm->getSelectedItem());
            cout << get_current_dir_name() << endl;
            // o3dmesh->Transform(getRx(toRadians(140))); //210deg for big one
            // open3d::io::WriteTriangleMeshToOBJ("ClaasGrossChanged210deg.obj",*o3dmesh,false,false,true,false,false,true);
            o3dmesh->Translate(-o3dmesh->GetCenter());
            o3dmesh->Scale(0.001, o3dmesh->GetCenter());
            o3dmesh->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
            o3dmesh->ComputeTriangleNormals();
            o3dmesh->NormalizeNormals();

            o3dToOgreMesh(ReferenceModelPointer, o3dmesh, "Pandia/Simple");
            ScanningManager->ReferenceMesh = o3dmesh;

            double pointsPercm2 = 9;
            double pointsPercm2Small = 0.9;
            string pcdName = sm->getSelectedItem();
            pcdName = pcdName.substr(0, pcdName.size() - 4); // remove the .obj ending
            string smallPcdName = pcdName + "Small";
            pcdName = pcdName + ".ply";
            smallPcdName = smallPcdName + ".ply";
            //############### normal sized pcd ##########################
            ScanningManager->ReferencePCD = readPcd(PcdFolder + pcdName);
            if (ScanningManager->ReferencePCD->points_.size() == 0)
            {
                double area = o3dmesh->GetSurfaceArea();
                // this generates colors and normals
                auto pcd = ScanningManager->ReferenceMesh->SamplePointsPoissonDisk(pointsPercm2 * 10000 * area);
                ScanningManager->ReferencePCD = getScannablePcd(o3dmesh, pcd);
                ScanningManager->ReferencePCD->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
                open3d::io::WritePointCloudOption params;
                open3d::io::WritePointCloudToPLY(PcdFolder + pcdName, *ScanningManager->ReferencePCD, params);
            }
            // open3d::visualization::DrawGeometries({getOrigin(), ScanningManager->ReferencePCD});

            //############### small sized pcd ##########################
            ScanningManager->SmallReferencePCD = readPcd(PcdFolder + smallPcdName);
            if (ScanningManager->SmallReferencePCD->points_.size() == 0)
            {
                double area = o3dmesh->GetSurfaceArea();
                auto pcd = ScanningManager->ReferenceMesh->SamplePointsPoissonDisk(pointsPercm2Small * 10000 * area);
                ScanningManager->SmallReferencePCD = getScannablePcd(o3dmesh, pcd);
                ScanningManager->SmallReferencePCD->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
                open3d::io::WritePointCloudOption params;
                open3d::io::WritePointCloudToPLY(PcdFolder + smallPcdName, *ScanningManager->SmallReferencePCD, params);
            }
            // small pcd should give same result as large
            ScanningManager->ReferencePCAVectors = getPCAVectors(ScanningManager->SmallReferencePCD);
            ScanningManager->SmallReferencePCDCenter = ScanningManager->SmallReferencePCD->GetCenter();
        }
        else if (referenceFileType == "ply") // todo atm this is expermental as ReferenceMesh is needed in functions like getFOVBuffer
        {
            // todo check all parts where ReferenceMesh is needed.
            ScanningManager->isReferenceMesh = false;
            //############### normal sized pcd ##########################
            ScanningManager->ReferencePCD = readPcd(get_current_dir_name() + modelfolder + (std::string)sm->getSelectedItem());
            if (ScanningManager->ReferencePCD->points_.size() == 0)
            {
                cout << "Error: Reference Pcd is empty!" << endl;
                return;
            }
            ScanningManager->ReferencePCD->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5)); // grey
            ScanningManager->ReferencePCD->Translate(-ScanningManager->ReferencePCD->GetCenter());
            ScanningManager->ReferencePCD->Scale(0.001, ScanningManager->ReferencePCD->GetCenter());
            // open3d::visualization::DrawGeometries({getOrigin(), ScanningManager->ReferencePCD});

            o3dToOgrePcd(ReferenceModelPointer, ScanningManager->ReferencePCD, "Pandia/Simple");

            // auto mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(*ScanningManager->ReferencePCD, 0.01);
            // ScanningManager->ReferenceMesh = mesh;
            // o3dToOgreMesh(ReferenceModelPointer, ScanningManager->ReferenceMesh, "Pandia/Simple");

            string pcdName = sm->getSelectedItem();
            pcdName = pcdName.substr(0, pcdName.size() - 4); // remove the .ply ending
            string smallPcdName = pcdName + "Small";
            pcdName = pcdName + ".ply";
            smallPcdName = smallPcdName + ".ply";
            //############### small sized pcd ##########################
            ScanningManager->SmallReferencePCD = readPcd(PcdFolder + smallPcdName);
            if (ScanningManager->SmallReferencePCD->points_.size() == 0)
            {
                ScanningManager->SmallReferencePCD = ScanningManager->ReferencePCD->VoxelDownSample(0.01);
                open3d::io::WritePointCloudOption params;
                open3d::io::WritePointCloudToPLY(PcdFolder + smallPcdName, *ScanningManager->SmallReferencePCD, params);
            }
            // small pcd should give same result as large
            ScanningManager->ReferencePCAVectors = getPCAVectors(ScanningManager->SmallReferencePCD);
            ScanningManager->SmallReferencePCDCenter = ScanningManager->SmallReferencePCD->GetCenter();
        }
        else
        {
            cout << "Error: File Type not supported." << endl;
            return;
        }
    }
    if (sm->getName() == "lsm_OCMICamera")
    {
        CurrentCameraSerialNumber = (std::string)sm->getSelectedItem();
        CurrentCamera = getCameraBySerialNumber(Cameras, CurrentCameraSerialNumber);
    }
}

bool StaticScannerGUI::keyPressed(const OgreBites::KeyboardEvent &evt)
{
    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
    {
        getRoot()->queueEndRendering();
    }
    if (evt.keysym.sym == OgreBites::SDLK_F1)
    {
        LoadingOverlay->show();
        updateLoadingCaption("Loading");
    }
    if (evt.keysym.sym == OgreBites::SDLK_F2)
    {
        LoadingOverlay->hide();
    }
    if (evt.keysym.sym == 'W' || evt.keysym.sym == 'w')
    {
        if (WireframeToggle == true)
        {
            for (auto &m : PandiaMaterials)
            {
                m->getTechnique(0)->getPass(0)->setPolygonMode(PM_WIREFRAME);
                m->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
                m->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
            }
            WireframeToggle = false;
        }
        else
        {
            for (auto &m : PandiaMaterials)
            {
                m->getTechnique(0)->getPass(0)->setPolygonMode(PM_SOLID);
                m->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
                m->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
            }
            WireframeToggle = true;
        }
    }
    return true;
}

//######## general gui functions #############

// we need to call this out of render Frame, otherwise ogre segfaults.
void StaticScannerGUI::updateGUIState()
{
    if (LastProgramState != ProgramState)
    { // something changed!
        TrayManagers[LastProgramState]->hideAll();
        TrayManagers[ProgramState]->showAll();
        TrayManagers[ProgramState]->hideCursor();
        if (LastProgramState == ACTIVE)
        {
            SettingsTrayManager->showAll();
            SettingsTrayManager->hideCursor();
        }
        if (ProgramState == ACTIVE)
        {
            mListenerChain = {TrayManagers[ProgramState], CameraMan};
            SettingsTrayManager->hideAll();
        }
        else
        {
            mListenerChain = {TrayManagers[ProgramState], SettingsTrayManager, CameraMan};
        }
    }
    LastProgramState = ProgramState;
}

void StaticScannerGUI::updateCameraOrbit(const Eigen::Vector3d &target)
{
    Ogre::Vector3 t = Ogre::Vector3(target(0), target(1), target(2));
    CameraNode->setPosition(t);
    CameraNode->translate(0, 0, 2.5);
    CameraNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);
    CameraTargetNode->setPosition(t);
    CameraMan->setTarget(CameraTargetNode);
    CameraMan->setStyle(CS_MANUAL); // need this switch for cameraman to reset his camera distance and axis
    CameraMan->setStyle(CS_ORBIT);
}

// ############## Panida Logo Overlay ####################
void StaticScannerGUI::initLogo()
{
    Ogre::OverlayManager &OverlayManager = Ogre::OverlayManager::getSingleton();
    LogoContainer = static_cast<Ogre::OverlayContainer *>(OverlayManager.createOverlayElement("Panel", "LogoPanel"));
    LogoContainer->setMetricsMode(Ogre::GMM_RELATIVE);
    LogoContainer->setPosition(1 - logoWidth, 0);
    updateLogoDimensions();

    Ogre::MaterialPtr LogoMaterial = Ogre::MaterialManager::getSingleton().create("LogoMaterial", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    LogoMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("Logo_Pandia3D.png");
    LogoMaterial->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
    LogoMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    LogoMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    LogoMaterial->getTechnique(0)->getPass(0)->setTextureFiltering(TFO_TRILINEAR);
    LogoMaterial->getTechnique(0)->getPass(0)->setSceneBlending(SBT_TRANSPARENT_ALPHA);
    LogoContainer->setMaterial(LogoMaterial);

    LogoOverlay = OverlayManager.create("LogoOverlay");
    LogoOverlay->add2D(LogoContainer);
    LogoOverlay->show();
}

void StaticScannerGUI::updateLogoDimensions()
{
    double arwindow = (double)getRenderWindow()->getHeight() / (double)getRenderWindow()->getWidth();
    double height = logoWidth * logoAspectRatio / arwindow; // we need to compensate for window aspect ratio
    LogoContainer->setDimensions(logoWidth, height);
}

void StaticScannerGUI::initLoadingStuff()
{
    double arwindow = (double)getRenderWindow()->getHeight() / (double)getRenderWindow()->getWidth();
    double loadingSymbolHeight = loadingSymbolWidth * loadingSymbolAR / arwindow; // we need to compensate for window aspect ratio
    Ogre::OverlayManager &OverlayManager = Ogre::OverlayManager::getSingleton();

    LoadingSymbol = static_cast<Ogre::OverlayContainer *>(OverlayManager.createOverlayElement("Panel", "LoadingSymbol"));
    LoadingSymbol->setMetricsMode(Ogre::GMM_RELATIVE);
    LoadingSymbol->setPosition(0.5 - 0.5 * loadingSymbolWidth, 0.95 - loadingSymbolHeight);
    LoadingSymbol->setDimensions(loadingSymbolWidth, loadingSymbolHeight);

    auto font = Ogre::FontManager::getSingleton().getByName("Pandia/LoadingFont");
    font->setTrueTypeResolution(200);
    LoadingText = static_cast<Ogre::TextAreaOverlayElement *>(OverlayManager.createOverlayElement("TextArea", "LoadingText"));
    LoadingText->setMetricsMode(Ogre::GMM_RELATIVE);
    LoadingText->setCharHeight(loadingTextHeight);
    LoadingText->setColour(ColourValue(1, 1, 1));
    LoadingText->setFontName(font->getName());

    Ogre::MaterialPtr LoadingSymbolMaterial = Ogre::MaterialManager::getSingleton().create("LoadingMaterial", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    LoadingSymbolMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("LoadingSymbol_white.png");
    LoadingSymbolMaterial->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setRotateAnimation(1.0);
    LoadingSymbolMaterial->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureAddressingMode(TAM_CLAMP);
    LoadingSymbolMaterial->getTechnique(0)->getPass(0)->setTextureFiltering(TFO_NONE); // filtering causes artifacts in animation
    LoadingSymbolMaterial->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
    LoadingSymbolMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    LoadingSymbolMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    LoadingSymbolMaterial->getTechnique(0)->getPass(0)->setSceneBlending(SBT_TRANSPARENT_ALPHA);
    LoadingSymbol->setMaterial(LoadingSymbolMaterial);

    LoadingOverlay = OverlayManager.create("LoadingOverlay");
    LoadingSymbol->addChild(LoadingText);
    LoadingOverlay->add2D(LoadingSymbol);
    LoadingOverlay->hide();
}

void StaticScannerGUI::updateLoadingCaption(const std::string &caption)
{
    LoadingText->setCaption(caption);
    LoadingText->setWidth(0); // need to do all of this otherwise ogre doesn't update new caption width
    // LoadingText->initialise();
    LoadingText->_update();
    LoadingText->setPosition(-0.5 * LoadingText->getWidth() + 0.5 * loadingSymbolWidth, -loadingTextHeight);
    getRoot()->renderOneFrame();
}

void StaticScannerGUI::updateLoadingDimensions()
{
    if (LoadingOverlay->isVisible())
    {
        double arwindow = (double)getRenderWindow()->getHeight() / (double)getRenderWindow()->getWidth();
        double loadingSymbolHeight = loadingSymbolWidth * loadingSymbolAR / arwindow;
        LoadingSymbol->setDimensions(loadingSymbolWidth, loadingSymbolHeight);
        LoadingText->setWidth(0);
        LoadingText->_update();
        LoadingText->setPosition(-0.5 * LoadingText->getWidth() + 0.5 * loadingSymbolWidth, -loadingTextHeight);
    }
}

//######## Field of view functions #############

void StaticScannerGUI::SaveSliderValues(CameraWrapper &cam)
{

    ofstream file(ResourceFolderPath + "config/" + cam.SerialNumber + "FOV.txt");
    if (file.is_open())
    {
        file << cam.crop_left << endl;
        file << cam.crop_right << endl;
        file << cam.crop_top << endl;
        file << cam.crop_bottom << endl;
        file << cam.crop_depth << endl;
        file << cam.crop_depth_inv << endl;
        file << cam.crop_ground_height << endl;
        file << cam.crop_ground_height_fine << endl;
    }
    else
    {
        cout << "Writing Crop data to file failed! \n";
        return;
    }
    file.close();
}

// cant be done in GUIimage since all the Manager Stuff is needed.
void StaticScannerGUI::initGUIImage(GUIImage &g_img, int n, int widht, int height)
{
    g_img.RenderRectangle = new Ogre::Rectangle2D(true);
    g_img.RenderRectangle->setCorners(0.0, 0.0, 0.0, 0.0);
    g_img.RectangleTexture = TextureManager::getSingleton().createManual(
        "DynamicTexture" + to_string(n), // name
        ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        TEX_TYPE_2D, // type
        widht,
        height,
        0,           // number of mipmaps
        PF_BYTE_BGR, // pixel format
        TU_DEFAULT); // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
                     // textures updated very often (e.g. each frame)

    g_img.RectangleMaterial = MaterialManager::getSingleton().create(
        "DynamicTextureMaterial" + to_string(n), // name
        ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    g_img.RectangleMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("DynamicTexture" + to_string(n));
    g_img.RectangleMaterial->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
    g_img.RectangleMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    g_img.RectangleMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    // material->getTechnique(0)->getPass(0)->setSceneBlending(SBT_TRANSPARENT_ALPHA);
    g_img.RenderRectangle->setMaterial(g_img.RectangleMaterial);
}

//######## Calibration functions #############

void StaticScannerGUI::DisplayFirstCalibrationImages()
{
    double arwindow = (double)getRenderWindow()->getHeight() / (double)getRenderWindow()->getWidth();
    double ar = (double)Cameras->front().FullCalibrationImageHeight / (double)Cameras->front().FullCalibrationImageWidth;

    int camCount = Cameras->size();
    if (camCount == 1)
    {
        double width = 1.2;
        double left = -width / 2;
        double height = width * ar / arwindow; // we need to compensate for window aspect ratio
        double bottom = -height / 2;
        GUIImages[0].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
    }
    else if (camCount == 2)
    {
        double width = 0.65;
        double offset = 0.04; // space between rectangles
        double left = -width - offset / 2;
        double height = width * ar / arwindow; // we need to compensate for window aspect ratio
        double bottom = -height / 2;
        GUIImages[0].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[1].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
    }
    else if (camCount == 3)
    {
        double bottom = 0;
        double width = 0.65;
        double offset = 0.04; // space between rectangles
        double left = -width - offset / 2;
        double height = width * ar / arwindow; // we need to compensate for window aspect ratio
        GUIImages[0].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[1].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = -width / 2;
        bottom = bottom - height - offset;
        GUIImages[2].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
    }
    else if (camCount == 4)
    {
        double bottom = 0;
        double width = 0.65;
        double offset = 0.04; // space between rectangles
        double left = -width - offset / 2;
        double height = width * ar / arwindow; // we need to compensate for window aspect ratio
        GUIImages[0].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[1].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = -width - offset / 2;
        bottom = bottom - height - offset;
        GUIImages[2].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[3].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
    }
    else if (camCount == 5)
    {
        double bottom = 0;
        double width = 0.5;
        double offset = 0.02; // space between rectangles
        double left = -1.5 * width - offset;
        double height = width * ar / arwindow; // we need to compensate for window aspect ratio
        GUIImages[0].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[1].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[2].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = -width - offset / 2;
        bottom = bottom - height - offset;
        GUIImages[3].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[4].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
    }
    else if (camCount == 6)
    {
        double bottom = 0;
        double width = 0.5;
        double offset = 0.02; // space between rectangles
        double left = -1.5 * width - offset;
        double height = width * ar / arwindow; // we need to compensate for window aspect ratio
        GUIImages[0].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[1].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[2].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = -1.5 * width - offset;
        bottom = bottom - height - offset;
        GUIImages[3].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[4].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[5].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
    }
    else if (camCount == 7)
    {
        double bottom = 0;
        double width = 0.38;
        double offset = 0.02; // space between rectangles
        double left = -2 * width - 1.5 * offset;
        double height = width * ar / arwindow; // we need to compensate for window aspect ratio
        GUIImages[0].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[1].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[2].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[3].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = -1.5 * width - offset;
        bottom = bottom - height - offset;
        GUIImages[4].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[5].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[6].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
    }
    else if (camCount == 8)
    {
        double bottom = 0;
        double width = 0.38;
        double offset = 0.02; // space between rectangles
        double left = -2 * width - 1.5 * offset;
        double height = width * ar / arwindow; // we need to compensate for window aspect ratio
        GUIImages[0].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[1].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[2].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[3].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = -2 * width - 1.5 * offset;
        bottom = bottom - height - offset;
        GUIImages[4].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[5].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[6].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
        left = left + width + offset;
        GUIImages[7].RenderRectangle->setCorners(left, bottom + height, left + width, bottom);
    }
    else
    {
        cout << "Visualisierung für " << camCount << " Kameras noch nicht implementiert! \n";
    }
    int count = 0;
    for (auto &cam : *Cameras)
    {
        auto img = CalibrationManager->getImagewithCorners(cam);
        GUIImages[count].FillTexture(img);
        count++;
    }
}

void StaticScannerGUI::HideAll()
{
    ReferenceModelPointer->setVisible(false);
    ReferenceObjectPointer->setVisible(false);
    ReferenceMissingPointer->setVisible(false);
    CroppedPcdPointer->setVisible(false);
    MergedPCDPreview->setVisible(false);
    CurrentScanPointer->setVisible(false);
    FOVCameraObject->setVisible(false);
    FOVIndicator->setVisible(false);
    FOVWireIndicator->setVisible(false);
    for (auto &img : GUIImages)
    {
        img.RenderRectangle->setVisible(false);
    }
    for (auto &m : CameraMeshes)
    {
        m->setVisible(false);
    }
}

void StaticScannerGUI::shutdown()
{
    PandiaMaterials.clear();
    CameraMaterial.reset();
    mListenerChain.clear(); // clear here as ApplicationContext deletes inputlisteners and not chains
    if (getRoot()->getRenderSystem() != NULL)
    {
        SceneManager->removeRenderQueueListener(getOverlaySystem());
        SceneManager->destroyAllManualObjects();
        SceneManager->destroyAllCameras();
        SceneManager->clearScene();
        getRenderWindow()->removeAllViewports();
        getRoot()->destroySceneManager(SceneManager);
    }
    TrayManagers.clear();
    delete CameraMan;
    delete SettingsTrayManager;
    OgreBites::ApplicationContext::shutdown(); // call default shutdown function for the rest
}

void StaticScannerGUI::showCameraModels(const Eigen::Matrix4d &trans)
{
    for (int i = 0; i < Cameras->size(); i++)
    {
        auto &cam = Cameras->at(i);
        auto CameraModel = readMesh(get_current_dir_name() + CameraModelPath);
        CameraModel->Transform(trans * cam.Extrinsic);
        CameraModel->Scale(CameraModelScaleFactor, CameraModel->GetCenter());
        o3dToOgreMesh(CameraMeshes[i], CameraModel, CameraMaterial->getName(), getflip3());
        CameraMeshes[i]->setVisible(true);
    }
}

void StaticScannerGUI::initCameraModel()
{
    if (CurrentCamera->CameraType == AzureKinect)
    {
        CameraModelPath = "/../resources/CameraModels/Axzurcam.obj";
        CameraModelScaleFactor = 0.08;
        CameraMaterial = MaterialManager::getSingleton().create("AzureKinectMaterial", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        CameraMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("AzureKinect.png");
        PandiaMaterials.push_back(CameraMaterial);
    }
    else if (CurrentCamera->CameraType == IntelCamera)
    {
        CameraModelPath = "/../resources/CameraModels/IntelRealSeneseD455.obj";
        CameraModelScaleFactor = 0.7;
        CameraMaterial = MaterialManager::getSingleton().create("IntelCameraMaterial", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        CameraMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("IntelRealSenseD455.png");
        PandiaMaterials.push_back(CameraMaterial);
    }
    else // todo use a default cam model
    {
        CameraModelPath = "/../resources/CameraModels/IntelRealSeneseD455.obj";
        CameraModelScaleFactor = 0.7;
        CameraMaterial = MaterialManager::getSingleton().create("IntelCameraMaterial", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        CameraMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("IntelRealSenseD455.png");
        PandiaMaterials.push_back(CameraMaterial);
    }
}

void StaticScannerGUI::initBackGround()
{
    // create background material
    Ogre::MaterialPtr backgroundmaterial = Ogre::MaterialManager::getSingleton().create("Background", "General");
    backgroundmaterial->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
    backgroundmaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    backgroundmaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    // create background rectangle covering the wholescreen
    ColoredGradient2D *gradientRectangle = new ColoredGradient2D();
    gradientRectangle->setCorners(-1.0, 1.0, 1.0, -1.0);
    gradientRectangle->setMaterial(backgroundmaterial);
    // set the colors
    //  Ogre::ColourValue topLeft(0.434, 0.522, 0.65);
    //  Ogre::ColourValue bottomLeft(0.18, 0.18, 0.18);
    Ogre::ColourValue topLeft(1.5 * 0.434, 1.5 * 0.522, 1.5 * 0.65);
    Ogre::ColourValue topRight = topLeft;
    Ogre::ColourValue bottomLeft(3 * 0.18, 3 * 0.18, 3 * 0.18);
    Ogre::ColourValue bottomRight = bottomLeft;

    gradientRectangle->setColors(topLeft, bottomLeft, topRight, bottomRight);

    // Render the background before everything else
    gradientRectangle->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
    // Use infinite AAB to always stay visible
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    gradientRectangle->setBoundingBox(aabInf);
    // Attach background to the scene
    SceneNode *backnode = SceneManager->getRootSceneNode()->createChildSceneNode("Background");
    backnode->attachObject(gradientRectangle);
    // viewport->setBackgroundColour(ColourValue(0.9, 0.9, 0.9)); // white background
}

void StaticScannerGUI::savePcdFunction(std::shared_ptr<open3d::geometry::PointCloud> pcd, bool setAsReferenceModel)
{
    if (pcd->IsEmpty())
        return;
    auto scaledPcd = *pcd;
    scaledPcd.Scale(1000, scaledPcd.GetCenter());

    // filename stuff
    // note: stored pcds are named like this: ScanSave000.ply, ScanSave001.ply, ...
    auto filenames = getFileNames(modelfolder); // already sorted
    string filename = "ScanSave";
    string n_max = "0";
    for (auto &fn : filenames)
    {
        if (fn.substr(0, filename.size()) == filename)
        {
            n_max = fn.substr(fn.size() - 7, 3); // get 3-digit number from pcd filename
        }
    }
    int scanNumber_int = stoi(n_max.erase(0, min(n_max.find_first_not_of('0'), n_max.size() - 1))); // remove leading zeros
    string scanNumber = to_string(scanNumber_int + 1);
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << scanNumber;
    scanNumber = ss.str();

    // write pcd to disk
    open3d::io::WritePointCloudOption option;
    option.write_ascii = open3d::io::WritePointCloudOption::IsAscii::Ascii;
    bool success = open3d::io::WritePointCloudToPLY(modelfolder.substr(1) + filename + scanNumber + ".ply", scaledPcd, option);
    if (success)
        cout << "Saved scanned Pcd to " << modelfolder.substr(1) << endl;
    else
    {
        cout << "Failed to save scanned Pcd" << endl;
        return;
    }

    // update lsm_model entries
    dynamic_cast<OgreBites::SelectMenu *>(TrayManagers[REFERENCEMODEL]->getWidget("lsm_Model"))->setItems(getFileNames(modelfolder));
    if (setAsReferenceModel)
        dynamic_cast<OgreBites::SelectMenu *>(TrayManagers[REFERENCEMODEL]->getWidget("lsm_Model"))->selectItem(filename + scanNumber + ".ply", true);
}

void StaticScannerGUI::initFOV()
{
    FOVIndicator = SceneManager->createManualObject("FOVCube");
    FOVWireIndicator = SceneManager->createManualObject("WireCube");
    FOVplanes = CurrentCamera->getFOVPlanes();
    o3dFOVIndicator = getFOVGeometryfromPlanes(FOVplanes, o3dWireIndicator);
    o3dToOgreMesh(FOVIndicator, o3dFOVIndicator, CubeMaterial, getflip3(), CubeAlpha);
    o3dToOgreLineList(FOVWireIndicator, o3dWireIndicator, "Pandia/SimpleAlphaWire", getflip3(), 1.0);
    FOVIndicator->setVisible(false);
    FOVWireIndicator->setVisible(false);
    SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(FOVIndicator);
    SceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(FOVWireIndicator);
}

void StaticScannerGUI::recordButtonFunction()
{

    cout << "Current cam serial number " << CurrentCamera->SerialNumber << endl;
    CurrentCamera->clearBuffers();
    if (!CurrentCamera->record(1, 0))
    {
        cout << "Error: Recording images failed! Aborting" << endl;
        return;
    }
    CurrentCamera->AlignUndistortandConvertO3D();
    auto pcd = make_shared<open3d::geometry::PointCloud>();
    if (CurrentCamera->CameraType == PhoxiScanner || CurrentCamera->CameraType == ZividCamera)
    {
        *pcd = *CurrentCamera->Pcds.front();
    }
    else
    {
        pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*CurrentCamera->rgbdImages.front(), CurrentCamera->DepthLDTIntrinsic);
    }
    // CurrentCamera->showOpen3dImages();
    pcd = pcd->VoxelDownSample(0.01);
    CurrentCamera->SetAndSaveThresholdValues(pcd);
    CurrentCamera->DownsampledPcd = pcd; // used for downsampling in slider movement
    auto CroppedPcd = CurrentCamera->getCroppedPcd(pcd);
    o3dToOgrePcd(CroppedPcdPointer, CroppedPcd, "Pandia/SimpleNoLight", getflip3());

    auto CameraModels = readMesh(get_current_dir_name() + CameraModelPath);
    CameraModels->Scale(CameraModelScaleFactor, CameraModels->GetCenter());
    o3dToOgreMesh(FOVCameraObject, CameraModels, CameraMaterial->getName(), getflip3());

    FOVCameraObject->setVisible(true);
    FOVIndicator->setVisible(true);
    FOVWireIndicator->setVisible(true);

    updateCameraOrbit(-CroppedPcd->GetCenter());
}