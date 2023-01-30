#include <Zivid/Zivid.h>
#include "CameraWrapper.h"
#include "pythonCompatible/VolumeCalculation.h"
#include "util.h"
using namespace std;

int main()
{
    CameraWrapper cam(EnsensoCamera);
	int n_cams = getNumberofConnectedDevices(EnsensoCamera);
    cam.connect();
    // cam.ReadFOVDataFromDisk(ResourceFolderPath + "config/");
	int n = 10000;
	for (int i = 0; i < n; i++)
	{
		if (cam.record(1,0))
			cout << "recorded " << i << endl;
		
		cam.rgbdImages.clear();
		cam.Pcds.clear();
	}


	return 0;
}