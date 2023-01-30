#!/bin/bash
[ "$UID" -eq 0 ] || exec sudo "$0" "$@"

cvpath=/home/$(logname)/dev/StaticScanner_cpp/3rdParty/opencv/lib/

echo "This script adds OpenCV librariy path to the environmental variable LD_LIBRARY_PATH"
echo "OpenCV librariy path in this script is set to:"
echo $cvpath

echo -n "Run script? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting"
    exit
fi

sudo echo $cvpath > /etc/ld.so.conf.d/OpenCVFix.conf
sudo ldconfig
