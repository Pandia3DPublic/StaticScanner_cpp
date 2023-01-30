#include "GlobalDefines.h"

std::string ResourceFolderPath = "../resources/";

std::atomic<bool> g_StopVolumeThread{false};
std::mutex g_VolumeLock; //for the volume pcd
int g_test =5;