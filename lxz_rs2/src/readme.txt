pip install -r requirements.txt
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_create_pkg lxz_rs2 visualization_msgs rospy
cd ..
catkin_make
source devel/setup.bash
rosrun lxz_rs2 run.py
