#!/bin/bash

pip install numpy
pip install scipy
pip install pandas
pip install tk
pip install -U catkin_tools


mkdir -p /home/shared/visavis_ws/src
cd home/shared/visavis_ws/src
git clone https://github.com/AndreaRoberti/vivavis_vision.git
caktin build


docker run -it --rm -v /home/shared:/shared -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 NOME_DOCKER %*