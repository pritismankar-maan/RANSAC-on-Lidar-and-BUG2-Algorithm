#!/usr/bin/env python
from random import random
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
# from std_msgs.msg import Float64MultiArray

import numpy as np

# number of iteration before choosing a line
iteration = 10 #200
# threshold distance to categories inliers and outliers
thres_dist = 0.1
# number of points after which ransac algorithm needs to stop
thres_points = 150#10
# points array required for marker
pnt_array = []
# change range into coordinates
coordinates = None
# intialize marker and velocity publishing object
marker_pub_obj = None

def range_into_coordinate(laser):
        global coordinates
        
        np_range = np.array(laser.ranges)
        # modify 3.0 to 0 as this is the maximum limit of the range sensor
        np_range[np_range == 3.0] = 0

        # get angle range and their sin and cosine values
        angle_range = np.linspace(laser.angle_min,laser.angle_max,361) 
        sinx = np.sin(angle_range)
        cosx = np.cos(angle_range)
        # find x and y coordinates
        sinx_of_range = np.multiply(np_range,sinx) 
        cosx_of_range = np.multiply(np_range,cosx)
        
        coordinates = np.vstack((cosx_of_range,sinx_of_range))  # x and y coordinates
        print('new laser feed')
        # del_point_array() 
        ransac()


def ransac():
        
        global iteration
        global thres_dist
        global thres_points
        global pnt_array
        global coordinates

        # store the points creating a line, p1 - 1st point, p2 - 2nd point - later to be used to create a line in between p1 and p2
        pnt_array = []
        
        # get total number of points in the point cloud
        total_points = 361
        # initiate the number of points remaining which are not in any lines (outliers)
        rem_points = total_points

        points = range(0,total_points)
        pnt_array = []
        
        no_further_pnts = False
        # until total outliers is greater than the threshold
        while rem_points > thres_points:
            # for each iteration choose a line 
            max_len = 0
            max_inliers = []

            # for 'n'th iteration find the lines and get inliers and outliers
            for i in range(iteration):
                if list(points):
                    # pick 2 points in random
                    random_points = np.random.choice(points,size=2)
                else:
                    no_further_pnts = True
                    break

                # get there specific x and y coordinates
                x1 = coordinates[0,random_points[0]]
                y1 = coordinates[1,random_points[0]]   
                x2 = coordinates[0,random_points[1]]
                y2 = coordinates[1,random_points[1]]
                
                # discard if both the points are 0 (these are range limits)
                if (x1 == 0.0 and y1 == 0.0):
                    points = np.delete(points,np.where(points == random_points[0]))
                    continue

                if (x2 == 0.0 and y2 == 0.0):
                    points = np.delete(points,np.where(points == random_points[1]))    
                    continue
                
                # create a temporary list and traverse through all the point to calculate distance
                temp_points = points
                temp_points = np.delete(points,np.where(points == random_points[0]))
                temp_points = np.delete(points,np.where(points == random_points[1]))

                # these variables will store the index of the x,y points in the coordinate array
                inliers = []

                # current points are inliers which would be send to marker if the line has the most inliers
                inliers = [random_points[0], random_points[1]]

                # store them as points for down the line processing
                p1 = np.array([x1,y1])
                p2 = np.array([x2,y2])

                for j in temp_points:
                    # calculate each points distance to the line
                    p3 = np.array([coordinates[0,j],coordinates[1,j]])
                    dist = np.abs(np.linalg.norm(np.cross(p2-p1,p1-p3)))/np.linalg.norm(p2-p1)

                    if dist <=thres_dist:
                        inliers.append(j)
                    
                curr_len = len(inliers)
                if curr_len > max_len:
                    max_len = curr_len
                    max_inliers = inliers   
                    

            if no_further_pnts == True or not max_inliers :
                break
            # assign p1 and p2 such that p1 is the most left point in the inliers and p2 is the most right point in the inliers
            left_inlier_idx = max(max_inliers)
            right_inlier_idx = min(max_inliers)
                
            left_pnt = np.array([coordinates[0,left_inlier_idx],coordinates[1,left_inlier_idx]])
            rght_pnt = np.array([coordinates[0,right_inlier_idx],coordinates[1,right_inlier_idx]])
            
            #  store these points(p1,p2) need to be sent to the markers
            pnt_array.append(left_pnt)
            pnt_array.append(rght_pnt)
                
            # delete all the inliers from the point cloud to find out the next line
            for j in max_inliers:
                points = np.delete(points,np.where(points == j))

            rem_points = len(points)
        
        print('ransac applied')
        publish_point_array()

def publish_point_array():
    global pnt_array
    global coordinates
    global marker_pub_obj

    marker_lins = Marker()
    # frame is in the local frame of the robot
    marker_lins.header.frame_id = "base_laser_link"
    marker_lins.header.stamp = rospy.Time.now()
    
    # namespace and action of point markers  
    marker_lins.ns = "points_using_ransac"
    marker_lins.action = Marker.ADD
            
    marker_lins.pose.orientation.w = 1.0 
    
    # define id
    marker_lins.id = 0 #1

    # define shape of points and linestrip
    shape = Marker.LINE_LIST
    marker_lins.type = shape
                
    # define only line width of the line strip
    marker_lins.scale.x = 0.01
            
    # Line strip is blue
    marker_lins.color.b = 1.0
    marker_lins.color.a = 1.0
            
    # pass the points to line strip markers
    len_array = len(pnt_array)
    for i in range(len_array):
        curr_pnt = pnt_array[i]
        
        p = Point()
        p.x = curr_pnt[0]
        p.y = curr_pnt[1]
        p.z = 0.0

        marker_lins.points.append(p)

            
    marker_lins.lifetime = rospy.Duration()
    
    if len_array != 0:
        # publish points and line strip markers
        marker_pub_obj.publish(marker_lins)
    # else:
    # marker_lins.action = Marker.DELETEALL
    # marker_pub_obj.publish(marker_lins)

    print('marker published')
            

def del_point_array():
    global pnt_array
    global coordinates
    global marker_pub_obj

    marker_lins = Marker()
    # frame is in the local frame of the robot
    marker_lins.header.frame_id = "base_laser_link"
    marker_lins.header.stamp = rospy.Time.now()
    
    # namespace and action of point markers  
    marker_lins.ns = "points_using_ransac"
    marker_lins.action = Marker.DELETEALL
            
    marker_lins.pose.orientation.w = 1.0 
    
    # define id
    marker_lins.id = 0 #1

    # define shape of points and linestrip
    shape = Marker.LINE_LIST
    marker_lins.type = shape
                
    # define only line width of the line strip
    marker_lins.scale.x = 0.01
            
    # Line strip is blue
    marker_lins.color.b = 1.0
    marker_lins.color.a = 1.0
            
    # pass the points to line strip markers
    len_array = len(pnt_array)
    # for i in range(len_array):
    #     curr_pnt = pnt_array[i]
        
    #     p = Point()
    #     p.x = curr_pnt[0]
    #     p.y = curr_pnt[1]
    #     p.z = 0.0

    #     marker_lins.points.append(p)

            
    marker_lins.lifetime = rospy.Duration()
    
    if len_array != 0:
        marker_lins.action = Marker.DELETEALL
        marker_pub_obj.publish(marker_lins)

    print('marker deleted')
            
def initiate_perception():
    global marker_pub_obj

    # declare marker publishing object
    marker_pub_obj = rospy.Publisher("visualization_marker", Marker, queue_size=1)

    rospy.Subscriber("base_scan", LaserScan,range_into_coordinate)    
    rospy.spin()

if __name__ == '__main__':
    try:
        # initiate perception node
        rospy.init_node('perception_node', anonymous=True)
        initiate_perception()
    
    except rospy.ROSInterruptException:
        pass
