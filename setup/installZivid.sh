#!/bin/bash

mkdir temp
cd temp

# install required opencl drivers for ubuntu (intel packages)
# see https://support.zivid.com/latest/getting-started/software-installation/gpu/install-opencl-drivers-ubuntu.html
mkdir opencl
cd opencl
wget https://github.com/intel/compute-runtime/releases/download/19.07.12410/intel-gmmlib_18.4.1_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/19.07.12410/intel-igc-core_18.50.1270_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/19.07.12410/intel-igc-opencl_18.50.1270_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/19.07.12410/intel-opencl_19.07.12410_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/19.07.12410/intel-ocloc_19.07.12410_amd64.deb
sudo dpkg -i *.deb
# verify opencl install
sudo apt install -y clinfo
/usr/bin/clinfo -l
cd ..

# install zivid software
# see https://support.zivid.com/latest/getting-started/software-installation.html
mkdir zivid
cd zivid
wget \
https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/u20/zivid-telicam-driver_3.0.1.1-3_amd64.deb \
https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/u20/zivid_2.5.0+19fa6891-1_amd64.deb \
https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/u20/zivid-studio_2.5.0+19fa6891-1_amd64.deb \
https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/u20/zivid-tools_2.5.0+19fa6891-1_amd64.deb
sudo dpkg -i *.deb

cd ../..
echo "Done. Re-plug the Zivid camera from PC"
