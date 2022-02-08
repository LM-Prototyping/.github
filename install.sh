#! /bin/bash

# LEGO PARTS

# Download and unzip lego parts library
PARTS_LINK="https://www.ldraw.org/library/updates/complete.zip"

wget $PARTS_LINK

unzip complete.zip

mv ldraw legoParts

# Copy custom lego parts
cp brickPiParts/brickpi.dat legoParts/parts/brickpi.dat
cp brickPiParts/squarehole.dat legoParts/parts/squarehole.dat

rm complete.zip

git submodule --init --recursive

# LEO CAD

LEO_CAD_LINK="https://github.com/leozide/leocad/releases/download/v21.06/LeoCAD-Linux-21.06-x86_64.AppImage"

cd LeoCAD
wget $LEO_CAD_LINK
cd ..

# TODO download example models in LeoCAD Models dir
# TODO alias hinzufügen

# Submodules initialisieren


# LDR TO PROTO

cd ldrToProto 
npm i .
cd ..

# TODO Docker builden
# TODO alias hinzufügen

# Create ROS2 Workspace and add package "robotConfiguration" 

# Source ros2 installation 
source ~/ros2_foxy/ros2-linux/setup.bash

mkdir -p ros2Workspace/src

cd ros2Workspace/src
ros2 pkg create --build-type ament_cmake robot_configuration
cd robot_configuration 
mkdir -p models
cd ../../

colcon build

