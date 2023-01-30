#!/bin/bash
if [ $UID -eq 0 ] ;then
    echo "Aborting. Please run without sudo."
    exit
fi

cd ..
mkdir buildStaticScanner
cd resources/
mkdir config
mkdir models
mkdir pcdModels
cd ../setup
echo "Done. Please put at least one mesh in resources/models folder."
