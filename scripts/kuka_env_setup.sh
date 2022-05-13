mkdir -p ~/iiwa_ws/src
cp iiwa_cam ~/iiwa_ws/src/iiwa_cam/iiwa_cam
cp iiwa_cam_moveit ~/iiwa_ws/src/iiwa_cam/iiwa_cam_moveit
cp scripts/.rosinstall ~/iiwa_ws

cd ~/iiwa_ws
rosinstall .
# mv -r ../iiwa_test src/iiwa_test
rosdep install --from-paths src --ignore-src -r -y
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

