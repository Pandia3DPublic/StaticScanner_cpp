#!/bin/bash
echo "This script initializes conda server virtual environment"
ans=$(which conda)
if [ "$ans" == "" ] ;then
    echo "Aborting. Please install anaconda first."
    exit
fi

conda create -n ServerEnv python=3.8
source /home/$USER/anaconda3/etc/profile.d/conda.sh
conda activate ServerEnv

#install packages
pip install -r ../webinterface/requirements.txt

conda deactivate
