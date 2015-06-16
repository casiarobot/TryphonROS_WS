

# Aerostabile
We aim to develop cubic blimps for artistic performances. They are entirely autonomous, based on IMU, sonars and USB cameras. Localization, control and interaction behaviors are developped in ROS. In the current state of our work, only 2 nodes run on the gumstix: one to handle the Robovero hardware (sonars, motors, battery level) and the other for the USB cameras (PointGrey). More information [here](http://robot.gmc.ulaval.ca/en/research/theme409.html).

## ROS Groundstation (off-board) Nodes

### ROS Nodes for State Feedback

#### Localization

The MCPTAM and sick_pos packages are used  for localization in the Aerostabile project with Cameras and a SICK laser, respectively. 
 [MCPTAM](https://github.com/aharmat/mcptam) is a package developed by Adam Harmat containing a set of ROS nodes for running Real-time 3D Visual Simultaneous Localization and Mapping (SLAM) using Multi-Camera Clusters. It includes tools for calibrating both the intrinsic and extrinsic parameters of the individual cameras within the rigid camera rig.  
 The sick_pos package works with the SICK laser to obtain the global x,y coordinates and yaw of the cubic blimps.

#### State Estimation

The above packages can be used with the state_estimation node of the state package to obtain full sate feedback of the cubic blimps. The kalman filter based state estimator can use the localization information obtained by sonars, IMU, SICK, MCPTAM or leddartech lasers either fused or individually.

### ROS Nodes for Control

The control package contains a Discrete PID, LQR or Computed-torque controller to control the cubic blimps for performances and experiments.

### ROS Nodes for User Commands

The ps3 package contains a ps3 node which can take user commands through a ps3 controller and control/move the blimp manually.  This package is one of the methods used in human robot interaction aspects of the Aerostabile project.

###### Acknowledgement
This research is conducted by the Mobile Robotics laboratory of professor Philippe Giguere at University Laval, with the collaboration of the research groups of professor Inna Sharf and professor Gregory Dudek at McGill University and under the artistic direction of Nicolas Reeves, professor at UQAM. All the development is coordinated by David St-Onge, eng. We would like to thanks the FQRNT-Team grant for their support.
