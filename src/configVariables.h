#pragma once

void readconfig(std::string s);
//checks if all config vars have been set
bool checkConfigVariables();

extern float g_SquareLength; //in m.
//width, height. Both need to be set.
//Used for Volume Measurement in Kamera Calibration. Denotes a height where the Silo is empty.
//Note: not very general for multiple cams.
extern cv::Size g_Pattern;
extern double g_Clearheight;
//The camera noise we assume for volume calculation so that walls dont get measured.
extern double g_VolumeNoise;
// Color Thresholds
extern double g_ColorMinThreshold;
extern double g_ColorMaxThreshold;
// Scanning variables
extern bool g_UseVoxelGrid;
extern bool g_RefineAlignWithCAD;