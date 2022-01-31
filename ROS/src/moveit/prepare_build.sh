cd $COLCON_WS

# clone all non-released dependencies
vcs import src/moveit/ --skip-existing --input src/moveit/non_released_dependencies.repos
