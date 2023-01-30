#!/bin/bash

mkdir temp
cd temp

# see https://www.ensenso.com/de/support/sdk-download/

# qt4 for viewer
sudo add-apt-repository ppa:rock-core/qt4
sudo apt update
sudo apt install libqt4-dev -y

# optional ueye driver (we don't need this atm)
# wget https://download.ensenso.com/s/idsdrivers/download?files=uEye_4.93.0_Linux_64.tgz
# mv download?files=uEye_4.93.0_Linux_64.tgz uEye_4.93.0_Linux_64.tgz
# tar -xvf uEye_4.93.0_Linux_64.tgz
# sudo ./ueye_4.93.0.989_amd64.run

# ensenso sdk
sdk=ensenso-sdk-3.2.489-x64.deb
wget https://download.optonic.com/s/ensensosdk/download?files=$sdk
mv download?files=$sdk $sdk
sudo dpkg -i $sdk

cd ..
echo "Done. Please reboot system"
