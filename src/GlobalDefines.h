#pragma once

extern std::string ResourceFolderPath;

extern std::atomic<bool> g_StopVolumeThread;
extern std::mutex g_VolumeLock; //for the volume pcd
extern int g_test;