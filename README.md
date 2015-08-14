

# Aerostabile
We aim to develop cubic blimps for artistic performances. They are entirely autonomous, based on IMU, sonars and USB cameras. Localization, control and interaction behaviors are developped in ROS. In the current state of our work, only 2 nodes run on the gumstix: one to handle the Robovero hardware (sonars, motors, battery level) and the other for the USB cameras (PointGrey). More information [here](http://robot.gmc.ulaval.ca/en/research/theme409.html).

## ROS Groundstation (off-board) Nodes

The ROS Nodes used in the Aerostabile project are all developped to work on one individual cubic blimp or on upto four simultaneously.  Therefore the nodes must be able to communicate appropriately and avoid any interference. By using launch files with group parameters, the same node can be executed multiple times while being labeled appropriately with the IP address of the respective cublic blimp the node is run on.  Topic names are chosen to be added after the group label. ( Note: It is important to avoid an inital "/" at the beginning of any topic name for the grouping method to work)

The majority of packages currently have launch files set up in the following format:  "node_name_24#.launch", where node_name is replaced with the respecitive node name and 24# is the respective cubic blimp (1-4),and corresponding IP that the node is intended to be used with. Launch files which follow the format "node_name##.launch" where ## is a combination of numbers (1-4) will launch the node for multiple cubic blimps.

### ROS Nodes for State Feedback

#### Localization 

The MCPTAM and sick_pos packages are used  for localization in the Aerostabile project with Cameras and a SICK laser, respectively. 
 [MCPTAM](https://github.com/aharmat/mcptam) is a package developed by Adam Harmat containing a set of ROS nodes for running Real-time 3D Visual Simultaneous Localization and Mapping (SLAM) using Multi-Camera Clusters. It includes tools for calibrating both the intrinsic and extrinsic parameters of the individual cameras within the rigid camera rig. By running the "mcptam" node, the pose of any platform which has the cameras attached is determined and can be sent to the state estimation for full state feedback. It is important to set the load and group_name parameters with the mcptam.launch file to choose whether to build a map or use a saved map, and what camera group to use respectively.  The mcptam2.launch is used to run two mcptam nodes with different camera_groups using the group function of ROS launch files.

 The sick_pos package contains the "sick_pose_node" which works with the SICK laser to obtain the global x,y coordinates and yaw of the cubic blimps.

#### State Estimation

The above packages can be used with the "state_estimator" node of the state package to obtain full sate feedback of the cubic blimps. The kalman filter based state estimator can use the localization information obtained by sonars, IMU, SICK, MCPTAM or leddartech lasers either fused or individually.  The "state_estimator" node contains an IP address argument  on top of the grouping label it is given from the launch file because of it's use in Gazebo and certain topics that are published that do not follow convention (ie Gumstix and Gumstix_filtered).  

### ROS Nodes for Control

The controls package contains the "control" node which uses either a Discrete PID, LQR or Computed-torque controller to control the cubic blimps for performances and experiments.  The node outputs the force wrench which is then sent to the "force_distribution" node to determine the inidivudal thrust required by each motor on the cubic blimp.  Using rqt, a nice gui for control is available where controller type and gains, predefined paths and global position can be changed.  

### ROS Nodes for User Commands

The ps3 package contains  ps3 nodes which can take user commands through a ps3 controller and control/move the blimp manually.  The "ps3_z" node uses the D-Pad and right and left bumbers to control the x,y,z global position of the cube by outputting a desired pose message that is interpreted by the controls node..   This package is one of the methods used in human robot interaction aspects of the Aerostabile project.

#Changes for Gazebo Model to Real Tryphon

##State Estimation

###state_estimation.cpp
Switch       
-quatMCPTAM,CMCAMpos,GyroOffset    
-IMU Callback function and subscriber topicname (raw_imu, imubuff)   
-In MCPTAM Callback change EulerU::getQuatFromEuler(quattf, 0,0,0);    
-For Gazebo --> q.setEulerZYX(MCPTAMpos(5), MCPTAMpos(4), MCPTAMpos(3));} for pos_src==0 && ==3    
-Other localization transformations    
-Possibly comment out unused sensor subscribers   
###Launch file
Switch    
-pos_src appropriately    
-choose sensors to be used   

##MCPTAM
Switch map in saved Folder (saved found in Gazebo repo) 

###### Acknowledgement
This research is conducted by the Mobile Robotics laboratory of professor Philippe Giguere at University Laval, with the collaboration of the research groups of professor Inna Sharf and professor Gregory Dudek at McGill University and under the artistic direction of Nicolas Reeves, professor at UQAM. All the development is coordinated by David St-Onge, eng. We would like to thanks the FQRNT-Team grant for their support.
