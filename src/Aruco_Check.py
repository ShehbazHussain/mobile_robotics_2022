#!/usr/bin/env python

import rospy
import actionlib

from fiducial_msgs.msg import FiducialTransformArray

arucoID = 0
oldID = 0

def callback(data):
    global arucoID
    global oldID
    arucoID = data.transforms[2]
    rospy.loginfo("Current ID Seen: %s", arucoID)
    if(arucoID > oldID):
        oldID = arucoID
    

if __name__ == '__main__':

    try:   
        rospy.init_node('checking_aruco', anonymous=False)
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, callback) 
        rate = rospy.Rate(3)
        print("We in the mains")

        while not rospy.is_shutdown():
            print("Current ArucoID: ", arucoID)
#            print("#IDs registered: ", oldID)

            if(oldID+1 == 2):
                print("done searching for markers!")

        rate.sleep()



    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
