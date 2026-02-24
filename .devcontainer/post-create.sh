#!/bin/bash

set -xe


function add_config_if_not_exist {
    if ! grep -F -q "$1" $HOME/.bashrc; then
        echo "$1" >> $HOME/.bashrc
    fi
}

add_config_if_not_exist "source /opt/ros/humble/setup.bash"
add_config_if_not_exist "source /opt/ros/lcas/install/setup.bash"
add_config_if_not_exist "alias rviz_sensors='rviz2 -d /opt/ros/lcas/install/limo_description/share/limo_description/rviz/model_sensors_real.rviz'"
add_config_if_not_exist "alias tidybot_sim='ros2 launch uol_tidybot tidybot.launch.py'"

# Add cuDNN library path for GPU acceleration
CUDNN_PATH="$HOME/.local/lib/python3.10/site-packages/nvidia/cudnn/lib"
add_config_if_not_exist "export LD_LIBRARY_PATH=$CUDNN_PATH:\$LD_LIBRARY_PATH"

source /opt/ros/humble/setup.bash
source /opt/ros/lcas/install/setup.bash

# Update package lists
sudo apt-get update
# Install ROS package
sudo apt-get install -y ros-humble-vision-msgs ros-humble-image-geometry ros-humble-navigation2  ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

# Install Python dependencies for YOLO and other tools
/usr/bin/python3 -m pip install "numpy<2" opencv-python python-dotenv ultralytics roboflow -q || true

# Clone explore_lite from source (always get latest)
cd src
# Remove if exists to get fresh clone
rm -rf m-explore-ros2
# git clone https://github.com/robo-friends/m-explore-ros2.git
git clone https://github.com/toluolatubosun/m-explore-ros2.git
cd ..

colcon build --symlink-install --continue-on-error || true

LOCAL_SETUP_FILE=`pwd`/install/setup.bash
add_config_if_not_exist "if [ -r $LOCAL_SETUP_FILE ]; then source $LOCAL_SETUP_FILE; fi"

sleep 10
DISPLAY=:1 xfconf-query -c xfce4-desktop -p $(xfconf-query -c xfce4-desktop -l | grep "workspace0/last-image") -s /usr/share/backgrounds/xfce/lcas.jpg  || true
