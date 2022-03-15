
#FAST_LIO

FAST-LIO (Fast LiDAR-Inertial Odometry) is a computationally efficient and robust LiDAR-inertial odometry package. It fuses LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter to allow robust navigation in fast-motion, noisy or cluttered environments where degeneration occurs. Our package address many key issues:

    Fast iterated Kalman filter for odometry optimization;
    Automaticaly initialized at most steady environments;
    Parallel KD-Tree Search to decrease the computation;



## Prerequisites

- Ubuntu >= 16.04
For Ubuntu 18.04 or higher, the default PCL and Eigen is enough for FAST-LIO to work normally.
- ROS >= Noetic , Melodic . 
- PCL >= 1.8, Follow PCL Intallation 
- Eigen >= 3.3.4, Follow Eigen Installation

- Livox_ros_driver : Follow livox_ros_driver Installation here: https://github.com/Livox-SDK/livox_ros_driver

## Build

Clone the repository and catkin_make:

    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/hku-mars/FAST_LIO.git
    cd FAST_LIO
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
Remember to source the livox_ros_driver before build (follow 1.3 livox_ros_driver)
If you want to use a custom build of PCL, add the following line to ~/.bashrc export PCL_ROOT={CUSTOM_PCL_PATH}


## Directly run
### For the velodyne and the OUSTER(32, 64 and 128) 

Step A: Setup before run

Edit config/velodyne.yaml to set the below parameters:

    LiDAR point cloud topic name: lid_topic
    IMU topic name: imu_topic (both internal and external, 6-aixes or 9-axies are fine)
    Set the parameter timestamp_unit based on the unit of time (Velodyne) or t (Ouster) field in PoindCloud2 rostopic
    Line number (we tested 16, 32 and 64 line, but not tested 128 or above): scan_line
    Translational extrinsic: extrinsic_T
    Rotational extrinsic: extrinsic_R (only support rotation matrix)

    The extrinsic parameters in FAST-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame).

Step B: Run below

    cd ~/$FAST_LIO_ROS_DIR$
    source devel/setup.bash
    roslaunch fast_lio mapping_velodyne.launch

Step C: Run LiDAR's ros driver or play rosbag.



## PCD file save

Set pcd_save_enable in launchfile to 1. All the scans (in global frame) will be accumulated and saved to the file FAST_LIO/PCD/scans.pcd after the FAST-LIO is terminated.
pcl_viewer scans.pcd can visualize the point clouds.



ROSBAG OUSTER OS1-128 

## Result of Fast_Lio : 
    
<p align="center">  ### OS1-128 Map Generation in RViz with Fast_Lio :  </p> 
    
   
  <p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/158196791-3807d46c-276b-46f8-8905-d717eae299f6.png?raw=true" alt="Sublime's custom image"/>
</p>

