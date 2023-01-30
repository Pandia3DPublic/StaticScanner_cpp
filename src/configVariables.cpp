#include "configVariables.h"

using namespace std;
void readconfig(std::string s);

//########## Calibration Variables #############
float g_SquareLength = -1; //in m.
cv::Size g_Pattern{-1,-1};      //width, height. Both need to be set.
double g_Clearheight = -1;    //Used for Volume Measurement in Kamera Calibration. Denotes a height where the Silo is empty.
double g_VolumeNoise = -1;
double g_ColorMinThreshold = -1;
double g_ColorMaxThreshold = -1;
bool g_UseVoxelGrid = false;
bool UseVoxelGrid_Set =false;
bool g_RefineAlignWithCAD = false;
bool RefineAlignWithCAD_Set = false;

//assign to global variables here
template <typename T>
void assignvalue(std::string &s, T &value)
{
    if (s == "SquareLength")
    {
        g_SquareLength = std::stof(value);
        return;
    }
    if (s == "Pattern_w")
    {
        g_Pattern.width = std::stoi(value);
        return;
    }
    if (s == "Pattern_h")
    {
        g_Pattern.height = std::stoi(value);
        return;
    }
    if (s == "Clearheight")
    {
        g_Clearheight = std::stod(value);
        return;
    }
    if (s == "VolumeNoise")
    {
        g_VolumeNoise = std::stod(value);
        return;
    }
    if (s == "ColorMinThreshold")
    {
        g_ColorMinThreshold = std::stod(value);
        return;
    }
    if (s == "ColorMaxThreshold")
    {
        g_ColorMaxThreshold = std::stod(value);
        return;
    }
    if (s == "UseVoxelGrid")
    {
        int tmp = std::stoi(value);
        if(tmp == 1){
            g_UseVoxelGrid = true;
            UseVoxelGrid_Set = true;
        } else if(tmp == 0){
            g_UseVoxelGrid = false;
            UseVoxelGrid_Set = true;
        }
        return;
    }
    if (s == "RefineAlignWithCAD")
    {
        int tmp = std::stoi(value);
        if (tmp == 1)
        {
            g_RefineAlignWithCAD = true;
            RefineAlignWithCAD_Set = true;
        }
        else if (tmp == 0)
        {
            g_RefineAlignWithCAD = false;
            RefineAlignWithCAD_Set = true;
        }
        return;
    }
}



void readconfig(std::string s)
{
    // std::ifstream is RAII, i.e. no need to call close
    cout << "Reading config from " << s << endl;
    std::ifstream cFile(s);
    if (cFile.is_open())
    {
        std::string line;
        while (getline(cFile, line))
        {
            line.erase(std::remove_if(line.begin(), line.end(), ::isspace),
                       line.end());
            if (line[0] == '#' || line.empty())
                continue;
            auto delimiterPos = line.find("=");
            auto name = line.substr(0, delimiterPos);
            auto value = line.substr(delimiterPos + 1);
            assignvalue(name, value);
        }
    }
    else
    {
        std::cerr << "Couldn't open config file for reading.\n";
        std::cout << "Working dir is available in c++17 \n"
                  << std::endl;
    }

    if (!checkConfigVariables())
    {
        std::cerr << "Not all config variables have been set correctly! \n";
    }
}

bool checkConfigVariables()
{

    bool out = true;

    if (g_SquareLength == -1)
        out = false;
    if (g_Pattern.height == -1)
        out = false;
    if (g_Pattern.width == -1)
        out = false;
    if (g_Clearheight == -1)
        out = false;
    if (g_VolumeNoise == -1)
        out = false;
    if (g_ColorMinThreshold == -1)
        out = false;
    if (g_ColorMaxThreshold == -1)
        out = false;
    if (UseVoxelGrid_Set == false)
        out = false;
    if (RefineAlignWithCAD_Set == false)
        out = false;

    return out;
}
