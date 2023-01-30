#!/bin/bash
echo "This script installs anaconda"
echo -n "Do you want to continue? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting."
    exit
fi

ans=$(which conda)
if [ "$ans" != "" ] ;then
    echo "Aborting. Anaconda is already installed."
    exit
fi

#install anaconda
cd temp
curl -O https://repo.anaconda.com/archive/Anaconda3-2021.11-Linux-x86_64.sh
sha256sum Anaconda3-2021.11-Linux-x86_64.sh
bash Anaconda3-2021.11-Linux-x86_64.sh

#verify install
source ~/.bashrc
ans=$(which conda)
if [ "$ans" != "" ] ;then
    echo $ans
    conda --version
    echo "Sucessfully installed Anaconda!"
else
    echo "Error: Something went wrong in anaconda installation process!"
fi

cd ..
echo "Done."
