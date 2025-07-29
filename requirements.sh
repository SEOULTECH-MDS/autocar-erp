#!/bin/bash

sudo apt update
sudo apt install python3-pip
sudo apt install -y python3-argcomplete
sudo apt install -y ros-humble-colcon-common-extensions rosdep python3-vcstool
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-vision-msgs
sudo apt install -y ros-humble-lanelet2
sudo apt install -y librange-v3-dev

sudo apt-get install gfortran
cd $HOME
git clone -b rtklib_ros_bridge_b34 https://github.com/MapIV/RTKLIB.git
cd $HOME/RTKLIB/lib/iers/gcc/
make
cd $HOME/RTKLIB/app/consapp
make

sudo apt-get install -y libgeographic-dev geographiclib-tools geographiclib-doc
sudo geographiclib-get-geoids best
sudo mkdir /usr/share/GSIGEO
sudo cp llh_converter/data/gsigeo2011_ver2_1.asc /usr/share/GSIGEO/

pip install --upgrade pip
pip install -r requirements.txt

pip install ultralytics
