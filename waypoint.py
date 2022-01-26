#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *
# Import LaserScan message from package sensor_msgs.
from sensor_msgs.msg import LaserScan
# Import the needed math functions.
from math import cos, sin, pow, radians, sqrt
# Importing Point message from package geometry_msgs.
from geometry_msgs.msg import Point

import time

drone = gnc_api()

# Function to avoid the obstacle and move the drone at a safe distance till it reaches its waypoint
def obstacle_avoid(scan_value):
    cr_scan = scan_value
    avoid_x = 0.0
    avoid_y = 0.0
    avoid = False

    for i in range(1, len(cr_scan.ranges)):
        d0 = 4
        # k = 0.6
        if cr_scan.ranges[i] < d0 and cr_scan.ranges[i] > 0.35:
            avoid = True
            x = cos(cr_scan.angle_increment * i)
            y = sin(cr_scan.angle_increment * i)
            # u = (-0.5 * k * pow(((1/cr_scan.ranges[i]) - (1/d0)), 2.0))

            avoid_x += x
            avoid_y += y

    # Getting the current_heading of the drone and converting it to radians.
    cr_heading = radians(drone.get_current_heading())
    sign_x = 1
    sign_y = 1
    if (avoid_x * cos(cr_heading)) - (avoid_y * sin(cr_heading)) < 0:
        sign = -1
    if (avoid_x * sin(cr_heading)) + (avoid_y * cos(cr_heading)) < 0:
        sign_y = -1
    factor_x = 0
    # here we are avoiding an object if it's under a limit from the drone
    if avoid:
        dist = sqrt(pow(avoid_x, 2) + pow(avoid_y, 2))
        cur_pose = drone.get_current_location()
        # print("current_location")
        # print(cur_pose)
        X_pos =  factor_x*sign_x + cur_pose.x 
        Y_pos =  3 + cur_pose.y 
        Z_pos = 10
        psi_pos = 0
        drone.set_destination( X_pos, Y_pos, Z_pos, psi_pos)
        time.sleep(3)
        
        

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    drone.set_mode("GUIDED")
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()
    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(10)
    drone.set_speed(2)
    
    # # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)
    # Specify some waypoints
    goals = [[0, 0, 10, 0], [50, 50, 10, 0], [0, 90, 10, 0],[0, 90, 0, 0]]
            #  [0, 0, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0] 
    i = 0
    time.sleep(4)
    # Checking for distance from the objects
    while i < len(goals):
        cr_scan = rospy.wait_for_message("/spur/laser/scan", LaserScan, timeout=None)
        obstacle_avoid(cr_scan)
        move = True
        for l in range(1, len(cr_scan.ranges)):
            d0 = 3
            # print("laser_callbackfunction")
            if cr_scan.ranges[l] < d0 and cr_scan.ranges[l] > 0.35:
                move = False
                # print("laser_callbackfunction")
                # time.sleep(3)
            else:
                move
        # If there is no obstacle the drone can be given the waypoints where it has to reach.
        if move:
            # print("inside the set_destination")
            drone.set_destination(
                x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
            rate.sleep()
            #time.sleep(3)
            current_location = drone.get_current_location()
            # print("current_location")
            # print(current_location)

            #Checking if the drone has reached the waypoint
            dx = abs(goals[i][0] - current_location.x)
            dy = abs(goals[i][1]- current_location.y)
            dMag = sqrt(pow(dx, 2) + pow(dy, 2))
            if dMag < 1.0: 
                rospy.loginfo("waypoint" + chr(i+97)+" reached ")
                i += 1
                
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)
    

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
