# RANSAC-on-Lidar-and-BUG2-Algorithm
Extract high level features from Lidar data using RANSAC Algorithm and BUG-2 local planner algorithm implementation


# Steps to run the file
1. Download the folder into the 'src' folder of your catkin workspace.
2. Build the catkin workspace.
3. Execute the command 'roslaunch lab6 perception.launch' to view the high level features such as lines and corner in Rviz window.
4. Execute the command 'roslaunch lab6 bug2.launch' to view the robot moving from start position(fixed) to end position(can be changed) in the ROS-Stage environment.        Along with this, you would be able to see the high level feature extraction at each timestep on the Rviz window.
