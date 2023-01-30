#pragma once
#include <iostream>
#include <string>
#include <k4a/k4a.hpp> //Azure Kinect C++ wrapper
#include <open3d/Open3D.h>
#include "open3d/pipelines/registration/GlobalOptimization.h"
#include "open3d/core/EigenConverter.h"
#include <json/json.h> //jsoncpp
#include <thread>
#include <chrono>
#include <list>
#include <fstream>
#include <cmath>
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <dirent.h> // for file handling
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"
#include "OgreTrays.h"
#include "OgreCameraMan.h"
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#define PHOXI_OPENCV_SUPPORT
#include "PhoXi.h" //Phoxi Scanner
#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Zivid.h>
#include <pybind11/pybind11.h>
#include "nxLib.h" //ensenso
#include "PandiaTimer.h"
#include "GlobalDefines.h"
#include "configVariables.h"
