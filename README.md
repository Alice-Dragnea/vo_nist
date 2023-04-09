ROS VERSION: ROS2 Humble  
PYTHON VERSION: 3.0+  
  
Installation:  
cp -r vo ~/ros2_ws/src/vo  
cd ~/ros2_ws  
colcon build --packages-select vo  
(if the above doesn't work, try using the --symlink-install flag, then rebuild)  
source ~/ros2_ws/install/setup.bash  
  
Start Node:  
ros2 run vo odom  
  
Plot Results:  
python3 visualize path_to_out.npy  
   
Desc:  
visualize.py - plots x, y, z trajectory when given an inpute file path  
voLib.py - helper functions  
run.py - node file  
