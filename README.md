# OS1-128-

 For instructions on ROS, start with the Example ROS Code section : 

This repository : https://github.com/ouster-lidar/ouster_example#example-ros-code contains sample code for connecting to and configuring ouster sensors, reading and visualizing data, and interfacing with ROS.

    ouster_client: contains an example C++ client for ouster sensors
    ouster_viz :contains a basic point cloud visualizer
    ouster_ros :contains example ROS nodes for publishing point cloud messages
    python:  contains the code for the ouster sensor python SDK


Sample Client and Visualizer
       1/ Building on Linux
       
1.1. To install build dependencies on Ubuntu, run:
                                 sudo apt install build-essential cmake libglfw3-dev libglew-dev libeigen3-dev \
                                       libjsoncpp-dev libtclap-dev


1.2. To build run the following commands:
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release <path to ouster_example>
make


  2/ Running the Sample Client

Make sure the sensor is connected to the network. See "Connecting to the Sensor" in the Software User Manual for instructions and different options for network configuration.

Navigate to ouster_client under the build directory, which should contain an executable named ouster_client_example. This program will attempt to connect to the sensor, capture lidar data, and write point clouds out to CSV files:

./ouster_client_example <sensor hostname> <udp data destination>

where <sensor hostname> can be the hostname (os-99xxxxxxxxxx) or IP of the sensor and <udp data destingation> is the hostname or IP to which the sensor should send lidar data. You can also supply "", an empty string, to utilize automatic detection.
  
  
  3/ Running the Sample Visualizer

Navigate to ouster_viz under the build directory, which should contain an executable named simple_viz . Run:

./simple_viz [flags] <sensor hostname> [udp data destination]

where <sensor hostname> can be the hostname (os-99xxxxxxxxxx) or IP of the sensor and [udp data destingation] is an optional hostname or IP to which the sensor should send lidar data.
  
  
  
  4/ Example ROS Code

The sample code include tools for publishing sensor data as standard ROS topics. Since ROS uses its own build system, it must be compiled separately from the rest of the sample code.The provided ROS code has been tested on ROS Kinetic, Melodic, and Noetic on Ubuntu 16.04, 18.04, and 20.04, respectively. Use the installation instructions to get started with ROS on your platform.
4.1/ Building: 

The build dependencies include those of the sample code:

sudo apt install build-essential cmake libeigen3-dev libjsoncpp-dev

Additionally, you should install the ros dependencies:

sudo apt install ros-<ROS-VERSION>-ros-core ros-<ROS-VERSION>-pcl-ros \
     ros-<ROS-VERSION>-tf2-geometry-msgs ros-<ROS-VERSION>-rviz

where <ROS-VERSION> is kinetic, melodic, or noetic.

Alternatively, if you would like to install dependencies with rosdep:

rosdep install --from-paths <path to ouster example>

To build:

source /opt/ros/<ROS-VERSION>/setup.bash
mkdir -p ./myworkspace/src
cd myworkspace
ln -s <path to ouster_example> ./src/
catkin_make -DCMAKE_BUILD_TYPE=Release

Warning: Do not create your workspace directory inside the cloned ouster_example repository, as this will confuse the ROS build system.

For each command in the following sections, make sure to first set up the ROS environment in each new terminal by running:

source myworkspace/devel/setup.bash

4.2/ Running ROS Nodes with a Sensor

Make sure the sensor is connected to the network. See "Connecting to the Sensor" in the Software User Manual for instructions and different options for network configuration.

To publish ROS topics from a running sensor, run:

roslaunch ouster_ros ouster.launch sensor_hostname:=<sensor hostname> \
                                   metadata:=<path to metadata json>

where:

    <sensor hostname> can be the hostname (os-99xxxxxxxxxx) or IP of the sensor
    <path to metadata json> is the path you want to save sensor metadata to. You must provide a JSON filename at the end, not just a path to a directory.

Note that by default the working directory of all ROS nodes is set to ${ROS_HOME}, generally $HOME/.ros. If you provide a relative path to metadata, i.e., metadata:=meta.json, it will write to ${ROS_HOME}/meta.json. To avoid this, you can provide an absolute path to metadata, i.e. metadata:=/home/user/meta.json.

You can also optionally specify:

    udp_dest:=<hostname> to specify the hostname or IP to which the sensor should send data
    lidar_mode:=<mode> where mode is one of 512x10, 512x20, 1024x10, 1024x20, or 2048x10, and
    viz:=true to visualize the sensor output, if you have the rviz ROS package installed

4.3/ Recording Data

To record raw sensor output use rosbag record. After starting the roslaunch command above, in another terminal, run:

rosbag record /os_node/imu_packets /os_node/lidar_packets

This will save a bag file of recorded data in the current working directory.

You should copy and save the metadata file alongside your data. The metadata file will be saved at the provided path to roslaunch. If you run the node and cannot find the metadata file, try looking inside your ${ROS_HOME}, generally $HOME/.ros. Regardless, you must retain the metadata file, as you will not be able to replay your data later without it.
      
4.4/ Playing Back Recorded Data

To publish ROS topics from recorded data, specify the replay and metadata parameters when running roslaunch:

roslaunch ouster_ros ouster.launch replay:=true metadata:=<path to metadata json>

And in a second terminal run rosbag play:

rosbag play --clock <bag files ...>

A metadata file is mandatory for replay of data. See Recording Data for how to obtain the metadata file when recording your data.
 
        
4.5/ SLAM Download the ROS node to your target location
Unzip the package source the setup.bash with "--extend" flag 
$ cd ../ ../ .. 
$ mkdir SLAM 
$ cd SLAM 
$ cp ~/Downloads/ROS_node.zip . 
$ unzip ROS_node.zip $ source SLAM/install/setup.bash --extend 
you should now have all the necessary pieces of software on your system. 
For each new terminal, need to source both Ouster and Kudan source files 
$ source/path-to-ouster-ros/devel/setup.bash 
Extend the SLAM ROS_node to work with the ouster_node 
$ source / path-to-ROS_node/install/setup.bash --extend

Playback a saved rosbag, and run lidar SLAM 
$ roslaunch kdlidar_ros kdlidar_ros_ouster_evaluation.launch replay:="true" replay_rate:="0.5" bag_path:="/path/and/filename.bag" metadata:="/path/and/filename.json

save the genirated map as a ply file when you have completed the sequence. 
$rosservice call /path ros_node_pcl/save_ply /path:to:mymap.ply 
once the map is saved you should see a success message success: True
     
     
     
     >[image](image.jpg "titre")
