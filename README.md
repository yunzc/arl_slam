Setup:
----------------------------------------------------------------------------------
Download realsense ros driver: sudo apt-get install ros-kinetic-realsense-camera
Download newer realsense ros driver: (in catkin_ws/src) git clone https://github.com/IntelRealSense/realsense_samples_ros.git
Download rtabmap: sudo apt-get install ros-kinetic-rtabmap-ros
Download the following github: https://github.com/yunzc/arl_slam.git

Run: 
-----------------------------------------------------------------------------------
a. With only visual Odom:
  - comment out the code as indicated in the code (zr300.launch)
  - make sure in rtabmap.launch visual_odometry is set as true
  - roslaunch arl_slam zr300.launch 
  - roslaunch arl_slam rtabmap.launch 
  
b. With 2D SLAM as Odom:
  - make sure the neede code in zr300.launch is not commented out 
  - turn visual_odometry argument in rtabmap.launch to false
  - roslaunch arl_slam zr300.launch
  - roslaunch arl_slam rtabmap.launch 

