mkdir -p ~/iiwa_ws/src/iiwa_cam
cp -r iiwa_cam ~/iiwa_ws/src/iiwa_cam/
cp -r iiwa_cam_moveit ~/iiwa_ws/src/iiwa_cam/
cp scripts/.rosinstall ~/iiwa_ws

cd ~/iiwa_ws
rosinstall .
# mv -r ../iiwa_test src/iiwa_test
rosdep install --from-paths src --ignore-src -r -y
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

