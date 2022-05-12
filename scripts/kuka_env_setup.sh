mkdir -p iiwa_ws/src
mv iiwa_test iiwa_ws/src/iiwa_test
mv scripts/.rosinstall iiwa_ws

cd iiwa_ws
rosinstall .
mv -r ../iiwa_test src/iiwa_test
rosdep install --from-paths src --ignore-src -r -y
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

