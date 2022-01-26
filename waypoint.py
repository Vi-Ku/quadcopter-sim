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
# Importing to
import time

move = True

def distance_check(msg):
    # Callback function of the subscriber.
    cr_scan = LaserScan()
    cr_scan = msg
    avoid = False

    for i in range(1, len(cr_scan.ranges)):
        d0 = 3
        print("laser_callbackfunction")
        if cr_scan.ranges[i] < d0 and cr_scan.ranges[i] > 0.35:
            move = False
            print("laser_callbackfunction")
        else:
            move = True


def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)
    rospy.Subscriber(name="/spur/laser/scan",
                     data_class=LaserScan,
                     queue_size=1,
                     callback=distance_check)

    # Create an object for the API. 
    drone = gnc_api()
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

    while i < len(goals):
        if move:
            print("inside the set_destination")
            drone.set_destination(
                x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
            rate.sleep()
            time.sleep(3)
            current_location = drone.get_current_location()
            print("current_location")
            print(current_location)

            dx = abs(goals[i][0] - current_location.x)
            dy = abs(goals[i][1]- current_location.y)
            dMag = sqrt(pow(dx, 2) + pow(dy, 2))
            if dMag < 1.0: 
                i += 1
                rospy.loginfo("waypoint reached ")
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
