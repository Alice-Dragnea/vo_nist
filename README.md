ROS VERSION: ROS2 Humble  
PYTHON VERSION: 3.0+  
  
Installation:  
cd ~/ros2_ws/src
git clone git@github.com:Alice-Dragnea/vo_nist.git
cd ~/ros2_ws
colcon build --packages-select vo_nist
  
Start Node:  
ros2 run vo_nist odom 

Start Node with Realsense:
ros2 launch vo_nist vo_launch.py 
  
Plot Results:  
python3 ./scripts/visualize path_to_out.npy  
   
Desc:  
visualize.py - plots x, y, z trajectory when given an input file path  
voLib.py - helper functions  
run.py - node file  

Below is a sample trajectory result:  
  
<p align="center">  
<img src="./scripts/SampleTrajectory.png" width=67% height=67%/>  
</p>  
