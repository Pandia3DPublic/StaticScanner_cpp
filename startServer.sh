#!/bin/bash
cd webinterface
source /home/$USER/anaconda3/etc/profile.d/conda.sh
conda activate ServerEnv
python3 main.py
conda deactivate
cd ..