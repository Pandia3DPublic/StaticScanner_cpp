#!/bin/bash

mkdir temp
cd temp

# this scripts installs 1.2.31, newest version can be found here https://www.photoneo.com/downloads/phoxi-control/
wget https://photoneo.com/files/installer/PhoXi/1.2.31/PhotoneoPhoXiControlInstaller-1.2.31-Ubuntu20-STABLE.tar.gz
tar -xvf PhotoneoPhoXiControlInstaller-1.2.31-Ubuntu20-STABLE.tar.gz
sudo ./PhotoneoPhoXiControlInstaller-1.2.31-Ubuntu20-STABLE.run --accept

cd ..
