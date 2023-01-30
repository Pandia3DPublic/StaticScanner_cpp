#!/bin/bash
if [ $UID -eq 0 ] ;then
    echo "Aborting. Please run without sudo."
    exit
fi

echo -n "Run Ogre build script? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting"
    exit
fi

cd ../3rdParty/ogre
rm -r build
rm -r install
mkdir build
mkdir install
./runCmake.sh
cd build
make install -j8
cd ../../../setup

echo "Done. Ogre is built and ready"
