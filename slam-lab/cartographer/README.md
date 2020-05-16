
1. replace configuration_files and launch files in 
/home/YOUR MACHINE/cartographer_ws/src/cartographer_ros/cartographer_ros

2. in launch/f110_2d.launch, change cartographer_node path to yours. 

3. cd ~/cartographer_ws
$ catkin_make_isolated --install --use-ninja
$ source install_isolated/setup.bash

4. roslaunch cartographer_ros f110_2d.launch

5. new terminal
cd your bagfile folder
rosbag play mapping.bag
(mapping.bag located in our Google Drive and recorded all the topic)

6. new terminal
rosrun rviz rviz

then in Rviz open the demo_2d.rviz in /home/stark/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files 
