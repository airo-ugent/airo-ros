#!bin/bash

# execute all available prepare scripts

cd $COLCON_WS
# run all  configurations
bash src/robots/UR/prepare_build.sh
bash src/grippers/robotiq_85/prepare_build.sh
bash src/moveit/prepare_build.sh
