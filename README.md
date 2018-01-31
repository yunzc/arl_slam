
# 3D SLAM work Summer 2017

Implement 3D SLAM on a quadcopter

## Hardware 

(current): Intel NUC, Realsense ZR300, Pixhawk2 (PX4)

## Setup

Download realsense ros driver: sudo pat-get install ros-kinetic-realsense-camera 

Download rtabmap: sudo apt-get install ros-kinetic-rtabmap-ros

Download and install https://github.com/cra-ros-pkg/robot_localization (This is for sensor fusion between visual odometry and imu)

Setup Pixhawk and MAVROS: https://dev.px4.io/en/ros/mavros_installation.html 

Download my github: git clone https://github.com/yunzc/arl_slam.git 


## Running SLAM: (Might clean up more later)

```
roslaunch arl_slam 3dslam_sensors.launch
```
and in another terminal 
```
roslaunch arl_slam rtabmap.launch (launches 3D SLAM) 
```



