#!/usr/bin/env python
from math import sqrt
from math import atan2

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

box_angle = 0
box_distance = 0
box_id = "0"
time_secs = 0
time_nsecs = 0

# global variables for fake scan parameters

def cb_laserScan(data):
    global box_angle, box_distance, box_id, time_secs, time_nsecs

    # populate with fake scan parameters
    box_angle = atan2(data.pose.position.y, data.pose.position.x)
    box_distance = sqrt((data.pose.position.x * data.pose.position.x) + (data.pose.position.y * data.pose.position.y))
    box_id = "0"
    time_secs = data.header.stamp.secs
    time_nsecs = data.header.stamp.nsecs


def camera_to_laser_scan():
    # initialize node
    rospy.init_node('camera_to_laser_scan_1', anonymous=True)

    rospy.Subscriber('visulization_marker/ArUco_Location_1', Marker, cb_laserScan)

    # publisher to publish fake scan information
    publisher = rospy.Publisher('camera_to_laser_scan_1', LaserScan, queue_size=50) 

    rate = rospy.Rate(10) # 10 Hz refresh rate

    rate = rospy.Rate(10)       # 10 Hz refresh rate

    while not rospy.is_shutdown():
        laserScan = LaserScan() # fake scan object

        # populate with fake scan parameters
        laserScan.header.stamp.secs = time_secs
        laserScan.header.stamp.nsecs = time_nsecs
        laserScan.header.frame_id = 'odom'
        laserScan.angle_min = box_angle - 0.01
        laserScan.angle_max = box_angle + 0.01
        laserScan.range_min = 0.0
        laserScan.range_max = 20
        laserScan.angle_increment = 0.0001
        laserScan.time_increment = 0.0001
        laserScan.ranges = [box_distance, (box_distance + 0.05), (box_distance + 0.05*2), (box_distance + 0.05*3)]

        publisher.publish(laserScan)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_to_laser_scan()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
