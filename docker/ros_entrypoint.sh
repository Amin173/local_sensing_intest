#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$OVERLAY_WS/devel/setup.bash"
apt install pip
apt install pip3
pip install numpy
pip install netifaces
pip install pyparticleio
pip install pandas
pip install socket
pip3 install matplotlib
pip3 install tensorflow
pip3 install cv2
pip3 install open3d
pip3 install ctypes
pip3 install scipy
pip install copy
exec "$@"