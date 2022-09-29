
#!/usr/bin/env python
from cmath import pi
from random import random
from time import sleep
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import math
import numpy as np
import time
from tf.transformations import euler_from_quaternion

# intialize velocity publishing object
vel_pub_obj = None
# store robot's current pose, goal threshold, motion type, front wall & left wall bool,point cloud
robot_x = None
robot_y = None
robot_theta = None
goal_threshold = 0.3
motion_type = None
point_cloud = None
front_wall = False
left_wall = False
on_m_line = True


# store robot's pose
def get_curr_pos(odom):
    global robot_x, robot_y,robot_theta

    print('ODOM')
    # store robot pose
    robot_x = odom.pose.pose.position.x
    robot_y = odom.pose.pose.position.y
    orient = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
    _,_,yaw = euler_from_quaternion(orient)
    robot_theta = yaw


# determine distance to goal    
def get_goal_dist(goal_pose,robot_x,robot_y):
    return math.sqrt((goal_pose[0]-robot_x)**2 + (goal_pose[1]-robot_y)**2)

# update 2-D point cloud
def store_update_scan(laser):
    global point_cloud,front_wall,left_wall
    point_cloud = laser.ranges
    print('LASER')

    # find if there is obstruction to the front or to the left
    count = 0
    for i in range(120,240):
        if point_cloud[i] < 1.0:
            count +=1
            if count > 3:
                front_wall = True
                break
        else:
            front_wall = False

    count = 0
    for i in range(0,61):
        if point_cloud[i] < 1.0:
            count +=1
            if count > 20:
                left_wall = True
                break
        else:
            left_wall = False

# compute rotation of robot to face the goal during GOAL SEEK
def compute_goal_seek_rot(goal_angle):
    global robot_theta

    if abs(goal_angle) < pi/50:
        return 0
    else:
        return goal_angle
    
# compute translation vel based on if there is obstacle at front    
def compute_trans_vel():
    global point_cloud,front_wall,left_wall
    if front_wall:
        return 0
    else:
        return 0.6    

# check if obstacle is there in between the robot and the goal (towards the goal direction)
def obstacle_in_way(goal_angle):
    global point_cloud,robot_theta,robot_x,robot_y
    
    const = math.floor((361/pi)*(goal_angle+pi/4))
        
    if const+120 < 0:
            min_index = 0
    else:
            min_index = const+120

    if const+240 > 360:
            max_index = 360
    else:
            max_index = const + 240    
    
    if min_index >= max_index:
        # no obstruction
        return 0
    elif max_index <= min_index:
        # no obstruction
        return 0
    else:        
        # get minimum range from goal-pi/4 to goal+pi/4
        range_min = min(point_cloud[min_index:max_index])
        return range_min < 1.0

# compute rotation velocity during WALL FOLLOW
def compute_rwf_rot():
    global point_cloud,front_wall,left_wall

    if front_wall:
        return 0.5
    elif left_wall:
        return 0
    else:
        return -0.4    

# main control logic for bug2
def bug2_controller():
    global robot_x,robot_y,robot_theta,goal_threshold,motion_type,vel_pub_obj,point_cloud,on_m_line
    # Note - Rate is given higher so that this function get's called after the 2 subscriber's callback - GAVE ME SUCH PAIN
    rate = rospy.Rate(50)
    vel_command = Twist()

    # declare velocity publishing object
    vel_pub_obj = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    #get goal's position from ros params
    goal_pose_x = float(rospy.get_param('goal_pose_x'))
    goal_pose_y = float(rospy.get_param('goal_pose_y'))
    goal_pose_z = float(rospy.get_param('goal_pose_z'))
    goal_pose_deg = float(rospy.get_param('goal_pose_deg'))      
            
    # store them in a list
    goal_pose = [goal_pose_x,goal_pose_y,goal_pose_z,goal_pose_deg]
    # store goal and starting position
    p1 = np.array([-8.00,-2.00])
    p2 = np.array([goal_pose[0],goal_pose[1]])
            

    # start processing
    atGoal = False
    motion_type = 'GOAL SEEK'

    while(not atGoal and not rospy.is_shutdown()):
            global robot_x, robot_y,robot_theta,point_cloud
            if point_cloud != None and robot_x != None and robot_y != None and robot_theta != None:
                
                # determine goal distance and goal angle
                goal_dist = get_goal_dist(goal_pose,robot_x,robot_y)
                goal_angle = math.atan2(goal_pose[1]-p1[1],goal_pose[0]-p1[0]) - robot_theta 

                # compute robot's distance from m-line
                p1 = np.array([-8.00,-2.00])
                p2 = np.array([goal_pose[0],goal_pose[1]])
                p3 = np.array([robot_x,robot_y])
                # step -1 - get slope
                m = (p2[1] - p1[1])/(p2[0]-p1[0])
                # step - 2 - calculate distance
                dist = abs(p3[1]-(m*p3[0])-p1[1]+(m*p1[0]))/np.sqrt(1+(m*m))
                
                print(dist)
                # If it's nearby to m-line break from WALL FOLLOW state and follow the m-line
                if dist > 0.1:
                        on_m_line = False
                else:   
                        on_m_line = True
                
                # check if robot's at goal position or not
                if goal_dist<goal_threshold:
                    print('goal reached')
                    fwd_vel = 0.0
                    rot_vel = 0.0
                    atGoal = True
                    motion_type = 'DONE'
                else:
                    
                    # compute forward velocity
                    fwd_vel = compute_trans_vel()

                    if motion_type == 'GOAL SEEK':
                        # compute rotaional velocity
                        rot_vel = compute_goal_seek_rot(goal_angle)
                        
                        # if rotation velocity is non-zero, keep forward velocity to 0 in order to avoid drift
                        if rot_vel != 0:
                            fwd_vel = 0

                        # if obstruction, switch to WALL FOLLOW    
                        if front_wall:
                            motion_type = 'WALL FOLLOW'

                    elif motion_type == 'WALL FOLLOW':
                        # compute rotation velocity during WALL FOLLOW only
                        rot_vel = -1*compute_rwf_rot()
                        
                        # if robot's on the m-line and no obstruction towards the goal direction, switch to GOAL SEEK
                        if on_m_line: 
                            if not obstacle_in_way(goal_angle):
                                motion_type = 'GOAL SEEK'
                            

                    print(motion_type)

                    # publish velocity command
                    vel_command.linear.x = fwd_vel
                    vel_command.angular.z = rot_vel
                    vel_pub_obj.publish(vel_command)
                    
            rate.sleep()

def initiate_controller():
    # scan subscriber to get point cloud
    rospy.Subscriber("/base_scan", LaserScan, store_update_scan, queue_size= 5)    
    
    # odom subscriber to know robot's pose
    rospy.Subscriber("/odom",Odometry,get_curr_pos, queue_size= 5)

    # control logic
    bug2_controller()

    rospy.spin()

if __name__ == '__main__':
    try:
        # initiate perception node
        rospy.init_node('controller_node', anonymous=True)
        initiate_controller()
    
    except rospy.ROSInterruptException:
        pass
