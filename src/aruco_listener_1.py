#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker

flag = False

def create_marker(trans, rot):
    # publisher = rospy.Publisher('visualization_marker/ArUco_Location', Marker, queue_size = 50)

    marker = Marker()

    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 1
    marker.id = 1

    # Set the scale of the marker
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = trans[0] 
    marker.pose.position.y = trans[1]
    marker.pose.position.z = trans[2]
    marker.pose.orientation.x = rot[0]
    marker.pose.orientation.y = rot[1]
    marker.pose.orientation.z = rot[2]
    marker.pose.orientation.w = 1.0

    return marker

    # publisher.publish(marker)

def locate_marker():
    rospy.init_node('locate_marker_1', anonymous=True)
    listener = tf.TransformListener()


    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/fiducial_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        publisher = rospy.Publisher('visulization_marker/ArUco_Location_1', Marker, queue_size=50)

        marker = create_marker(trans, rot)

        publisher.publish(marker)


        rate.sleep()

if __name__ == '__main__':
    try:
        locate_marker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
