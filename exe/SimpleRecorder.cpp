#include "CameraWrapper.h"
#include "util.h"

int readRecordCountFromDisk(std::string dir)
{
    using namespace std;
    ifstream file(dir + "/RecordCount.txt");
    string line;
    if (file.is_open())
    {
        getline(file, line);
        if (!line.empty())
            return std::stoi(line);
        else
            return 0;
    }
    else
    {
        return 0;
    }
};

bool saveRecordCountToDisk(std::string dir, int count)
{
    using namespace std;
    cout << "Saving record count to " << dir << "\n";
    ofstream file(dir + "/RecordCount.txt");
    if (file.is_open())
    {
        file << count << endl;
    }
    else
    {
        cout << "Writing record count to file failed! \n";
        return false;
    }
    file.close();
    return true;
};

int main(int argc, char *argv[])
{
    using namespace std;
    CameraTypes CameraType = AzureKinect;
    CameraWrapper Camera(CameraType);
    int n_cams = getNumberofConnectedDevices(CameraType);
    cout << "Detected " << n_cams << " Cameras! \n";
    if (n_cams == 0)
    {
        cout << "No cameras detected... closing program" << endl;
        return 0;
    }
    if (!Camera.connect())
    {
        cout << "Connection failed\n";
        return -1;
    }
    Camera.printInfo();

    bool stop = false;
    int fps = 30;
    float seconds = 5;
    int n_images;
    std::string workingDir = open3d::utility::filesystem::GetWorkingDirectory();
    int recordCount = readRecordCountFromDisk(workingDir);
    while (!stop)
    {
        cout << "FPS is set to " << fps << endl;
        cout << "Enter record time in s: ";
        cin >> seconds;
        n_images = fps * seconds;
        cout << "Press enter to start recording";
        cin.ignore();
        cin.get();

        cout << "Recording ..." << endl;
        Camera.record(n_images, 0);
        Camera.AlignUndistortandConvertO3D();
        cout << "Writing rgbd images to disk..." << endl;
        Camera.saveImagesToDisk(workingDir + "/rgbd" + std::to_string(recordCount), false);
        cout << "Done writing rgbd images to disk." << endl;
        Camera.saveCameraInfoToDisk(workingDir + "/rgbd" + std::to_string(recordCount));
        Camera.clearBuffers();

        recordCount++;
        saveRecordCountToDisk(workingDir, recordCount);
        cout << "Record again? (y/n) ";
        std::string ans;
        cin >> ans;
        if (ans.empty() || ans.at(0) != 'y')
        {
            stop = true;
        }
    }

    return 0;
}