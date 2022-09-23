# STEREOPOLIS : 

For mass deployment of safe Level 2+ automated driving, carmakers need perception systems with a full suite of high performance lidar sensors.
We can use OS1-128 to bring safe ADAS/ADS vehicles to everyday roads across the globe.
OS1-128 which use laser pulses to reconstruct the environment surrounding a vehicle,allow automated and safe driving for cars, trucks...


## Acquisition STEREOPOLIS : 
The first step is to collect data from your environment. Before running everything on the integrated RC car, we will collect data in a simpler 
and more controlled environment with ROS and we will get a ROSBAG for imu and lidar packets. 
In the first example, an OS-1-128 is mounted to the car of acquisition with a laptop as shown in the image below.



 <p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/191936930-c62f83d6-e4f9-4507-a0f5-f30bf9bbc5d8.png?raw=true" alt="Sublime's custom image"/>
</p>

 <p align="center"> 
 Figure: Lidar OS1-128 sur la voiture d’acquisition
 </p>



## SLAM STEREOPOLIS : 

Simultaneous Localization and Mapping (SLAM) is a method that gives machines the ability to understand their position and orientation 
within an environment. SLAM gives machines spatial awareness by sensing, creating, and constantly updating a representation 
of the world around them.

The second step, we run FAST_LIO_SLAM :
- ROSBAG = 24.6 Go
- Ubuntu 20.04 (Focal) 
- ROS Noetic

<p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/191939306-83ae938f-18ff-4d4f-a5d8-2ded342ed596.png?raw=true" alt="Sublime's custom image"/>
</p>

<p align="center"> 
 Figure A: the trajectory of the acquisition car on google earth
 Figure B : slam result displayed on Rviz
 </p>
