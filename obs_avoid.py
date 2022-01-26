#! /usr/bin/env python
# Import ROS.
import rospy
# Import LaserScan message from package sensor_msgs.
from sensor_msgs.msg import LaserScan
# Import the API.
from iq_gnc.py_gnc_functions import *
# Import the needed math functions.
from math import cos, sin, pow, radians, sqrt
# Importing Point message from package geometry_msgs.
from geometry_msgs.msg import Point

import time
# Create an object for the API and making it a global variable.
drone = gnc_api()


def laser_cb(msg):
    # Callback function of the subscriber.
    cr_scan = LaserScan()
    cr_scan = msg
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

    if avoid:
        dist = sqrt(pow(avoid_x, 2) + pow(avoid_y, 2))

        cur_pose = Point()
        # Getting the current location from the drone.
        cur_pose = drone.get_current_location()
        print("current_location")
        print(cur_pose)
        X_pos = 0.4*sign_x  + cur_pose.x
        Y_pos = 3*sign_y + cur_pose.y
        Z_pos = cur_pose.z
        psi_pos = 10
        # print("current_X_pos: ",X_pos)
        # print("current_Y_pos: ",Y_pos)
        # print("current_Z_pos: ",Z_pos)
        # print("current_psi_pos: ",psi_pos)
        # Sending the goal.
        drone.set_destination( X_pos, Y_pos, Z_pos, psi_pos)

        time.sleep(3)

def main():
    # Initializing the ROS node.
    rospy.init_node("obs_avoider", anonymous=True)
    # Creating a subscriber for the topic '/spur/laser/scan'.
    rospy.Subscriber(name="/spur/laser/scan",
                     data_class=LaserScan,
                     queue_size=1,
                     callback=laser_cb)

    # Used to keep the node running.
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
