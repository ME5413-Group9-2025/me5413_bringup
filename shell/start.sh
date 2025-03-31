#!/bin/bash

X_POS=${1:-0}
Y_POS=${2:-0}
YAW=${3:-1.57079632679}

export GAZEBO_MODEL_PATH="$(HOME)/models"
#2D: (SICK)lms1xx, (Hokuyo)ust10, (Hokuyo)usm30, (Velodyne)vlp16, (Velodyne)hdl32e
export JACKAL_LASER=1
export JACKAL_LASER_TOPIC="scan"
export JACKAL_LASER_MODEL="ust10"
export JACKAL_LASER_3D=1
export JACKAL_LASER_3D_MODEL="vlp16"
export JACKAL_LASER_3D_TOPIC="velodyne"
#export JACKAL_BB2=1
export JACKAL_FLEA3=0
export JACKAL_STEREO_FLEA3=1
export JACKAL_FLEA3_TILT="0"


cd $HOME/ME5413_Final_Project

source devel/setup.bash


echo "Start simulation, position: spawn_x=$X_POS, spawn_y=$Y_POS, spawn_Y=$YAW"

roslaunch me5413_bringup simulation.launch spawn_x:=$X_POS spawn_y:=$Y_POS spawn_Y:=$YAW


