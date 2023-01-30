#!/bin/bash
[ "$UID" -eq 0 ] || exec sudo "$0" "$@"

#On Linux, once attached, the device should automatically enumerate and load all drivers. However, in order to use the Azure Kinect SDK with the device and without being 'root', you will need to setup udev rules. Once complete, the Azure Kinect camera is available without being 'root'
echo -n "Run script? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting"
    exit
fi

sudo cp initFiles/99-k4a.rules /etc/udev/rules.d/
echo "Done. Detach and reattach Azure Kinect devices if attached during this process."
