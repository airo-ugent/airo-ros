cd $COLCON_WS

# clone all non-released dependencies
vcs import src/grippers/robotiq_85 --skip-existing --input src/grippers/robotiq_85/robotiq_85.repos
