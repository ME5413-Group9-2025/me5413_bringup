export GAZEBO_MODEL_PATH="$(HOME)/models"
#2D: (SICK)lms1xx, (Hokuyo)ust10, (Hokuyo)usm30, (Velodyne)vlp16, (Velodyne)hdl32e
export JACKAL_LASER=1
export JACKAL_LASER_TOPIC="scan"
export JACKAL_LASER_MODEL="ust10"
export JACKAL_LASER_3D=1
export JACKAL_LASER_3D_MODEL="vlp16"
export JACKAL_LASER_3D_TOPIC="velodyne"
#export JACKAL_BB2=1

cd $HOME/ME5413_Final_Project

source devel/setup.bash

roslaunch me5413_bringup simulation.launch