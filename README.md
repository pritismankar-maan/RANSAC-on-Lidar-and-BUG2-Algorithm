# RANSAC-on-Lidar-and-BUG2-Algorithm
Extract high-level features (boundaries, corners, shapes) from Lidar data using RANSAC Algorithm and use those features with BUG-2 local planner algorithm to drive a simulated robot from Start to Finish.

![hippo](https://github.com/pritismankar-maan/RANSAC-on-Lidar-and-BUG2-Algorithm/blob/main/lab6/ros_ransac_bug2.gif)

# Installation Steps
1. Download the folder into the 'src' folder of your catkin workspace
   > pritismankar@ubuntu-ros:~/catkin_ws/src$ git clone https://github.com/pritismankar-maan/RANSAC-on-Lidar-and-BUG2-Algorithm.git
3. Build the catkin workspace.
   > pritismankar@ubuntu-ros:~/catkin_ws$ catkin_make
4. Source the bash file.
   > pritismankar@ubuntu-ros:~/catkin_ws$ source devel/setup.bash
6. Execute the command 'roslaunch lab6 perception.launch' to view the high-level features such as lines and corners in the RViz window.
   > pritismankar@ubuntu-ros:~/catkin_ws$ roslaunch lab6 perception.launch
7. Execute the command 'roslaunch lab6 bug2.launch' to view the robot moving from start position(fixed) to end position(can be changed) in the ROS-Stage environment. Along with this, you would be able to see the high level feature extraction at each timestep on the RViz window.
   > pritismankar@ubuntu-ros:~/catkin_ws$ roslaunch lab6 bug2.launch

# Configuration
1. Update Goal Pose
   > 'lab6/launch/bug2.launch' script can be updated: goal pose (X, Y, theta) needs to be changed 
2. Update starting Pose
   > 'lab6/World/playground.world' script can be updated: turtlebot pose (X, Y, theta) needs to be changed
