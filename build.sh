source ~/repos/ros_noetic/install_isolated/setup.bash
colcon build \
--cmake-args "-Wno-dev -Wno-deprecated" \
--packages-up-to faster
# --packages-up-to faster
# --symlink-install \
# --continue-on-error \
# rospack plugins --attrib=plugin rviz