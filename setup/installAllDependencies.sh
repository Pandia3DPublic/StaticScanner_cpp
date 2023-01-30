#!/bin/bash

echo "This script installs all necessary ubuntu packages"
echo -n "Do you want to continue? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting"
    exit
fi

mkdir temp
cd temp

echo "Updating installed packages and distro"
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade

echo "Installing essential ubuntu packages"
packages=(\
    build-essential \
    gcc-8 \
    g++-8 \
    lld \
    curl \
    ninja-build \
    dpkg-dev \
    wget \
    htop \
    )
sudo apt-get install -y ${packages[@]}

echo "Installing nvidia-cuda-toolkit"
sudo apt-get install -y nvidia-cuda-toolkit

echo "Installing git and git lfs"
sudo apt-get install git -y
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs -y
git lfs install

CmakeVersion=3.21.2
echo "Installing cmake version " $CmakeVersion " using binary files"
wget https://github.com/Kitware/CMake/releases/download/v$CmakeVersion/cmake-$CmakeVersion-linux-x86_64.sh
sudo mkdir /opt/cmake
sudo sh cmake-$CmakeVersion-linux-x86_64.sh --prefix=/opt/cmake --skip-license --exclude-subdir
sudo ln -s /opt/cmake/bin/cmake /usr/bin/cmake

echo "Installing ogre dependencies"
sudo apt-get install -y libgles2-mesa-dev
sudo apt-get install -y libsdl2-dev libxt-dev libxaw7-dev doxygen

echo "Installing Azure Kinect SDK"
wget https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
sudo apt-get update
sudo apt-get install -y k4a-tools libk4a1.4-dev

echo "Installing Intel Realsense SDK"
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install -y librealsense2-dkms librealsense2-utils 
sudo apt-get install -y librealsense2-dev librealsense2-dbg

echo "Successfully installed all dependencies!"
cd ..
