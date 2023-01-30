#include "VolumeSingleton.h"
using namespace std;

VolumeSingleton::VolumeSingleton(/* args */)
{
    VolumePcd = make_shared<open3d::geometry::PointCloud>();
}

VolumeSingleton::~VolumeSingleton()
{
}

bool VolumeSingleton::ReadAlignmentDataFromDisk(const string &path)
{
    cout << "Reading alignment data from " << path + "SiloAlignment.txt" << endl;

    ifstream fovdata(path + "SiloAlignment.txt");
    string line;
    vector<float> data;
    if (fovdata.is_open())
    {
        while (getline(fovdata, line))
        {
            data.push_back(stod(line));
        }
        if (data.size() != 12)
        {
            cout << "Warning: alignment data is bad.\n";
            return false;
        }
        x_offset = data[0];
        y_offset = data[1];
        z_offset = data[2];
        alpha = data[3];
        beta = data[4];
        gamma = data[5];
        centerTranslation(0) = data[6];
        centerTranslation(1) = data[7];
        centerTranslation(2) = data[8];
        PCDCenterVec(0) = data[9];
        PCDCenterVec(1) = data[10];
        PCDCenterVec(2) = data[11];
    }
    else
    {
        cout << "Warning: Reading alignment data from file went wrong\n";
        return false;
    }

    return true;
}