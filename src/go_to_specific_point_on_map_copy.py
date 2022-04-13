#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import roslaunch
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID
from fiducial_msgs.msg import FiducialTransformArray
import tf
import aruco_listener_0 
import aruco_listener_1 



finished_map = False
cancelled_map = False
aruco1 = False
aruco2 = False
stop_locate_marker = False
robo_clean_run = False

#function to cycle through transform arrays and set flags when found
def callbackAruco(data):
    global aruco1
    global aruco2
    for transform in data.transforms:
       arucoID = transform.fiducial_id
       #print(arucoID)
       if (arucoID == 0):
           aruco1 = True
           #print("found aruco1")
       elif (arucoID == 1):
           aruco2 = True
           #print("found aruco 2")

def callback(data):
    mapping_status = data.status.status
    rospy.loginfo("I heard %s", mapping_status)
    if (mapping_status == 3):
        rospy.loginfo("Done Mapping!")
        global finished_map
        finished_map = True

def callback2(data2):
    urCancelled = data2.id
    rospy.loginfo("I heard %s", urCancelled)
    if (urCancelled == ''):
        rospy.loginfo("Cancelled Mapping...")
        global cancelled_map
        cancelled_map = True

class GoToPose():
    
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
                # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

def locate_marker_1():
    
    listener = tf.TransformListener()


    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (trans_1,rot_1) = listener.lookupTransform('/map', '/fiducial_1', rospy.Time(0))
            return trans_1, rot_1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()

def locate_marker_0():
    
    listener = tf.TransformListener()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (trans_0,rot_0) = listener.lookupTransform('/map', '/fiducial_0', rospy.Time(0))
            return trans_0, rot_0
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()



if __name__ == '__main__':

    try:   

        rospy.init_node('nav_test', anonymous=False)
        rospy.Subscriber("move_base/result", MoveBaseActionResult, callback) 
        rospy.Subscriber("move_base/cancel", GoalID, callback2) 
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, callbackAruco) 

        while not rospy.is_shutdown():

            print("finished_map: ", finished_map," cancelled_map: ", cancelled_map," aruco1: ", aruco1," aruco2: ", aruco2)
            
            if stop_locate_marker == False:
                trans_0, rot_0 = locate_marker_0()
                trans_1, rot_1 = locate_marker_1()



            if((aruco2 or aruco1)):   


                #Turn off robo clean if its active
                if robo_clean_run == True:
                    robo_clean.stop()

                    print("robo clean has stopped as aruco marker has been found")

                aruco_marker_location = GoToPose()

                stop_locate_marker = True

                if aruco1:

                    x = trans_0[0]
                    y = trans_0[1]

                    x_angle = rot_0[0]
                    y_angle = rot_0[1]
                    z_angle = rot_0[2]
                    
                    # Customize the following values so they are appropriate for your location
                    position_aruco_marker = {'x': x, 'y' : y}
                    quaternion_aruco_marker = {'r1' : x_angle, 'r2' : y_angle, 'r3' : z_angle, 'r4' : 1.000}

                if aruco2:

                    x = trans_1[0]
                    y = trans_1[1]

                    x_angle = rot_1[0]
                    y_angle = rot_1[1]
                    z_angle = rot_1[2]

                    position_aruco_marker = {'x': x, 'y' : y}
                    quaternion_aruco_marker = {'r1' : x_angle, 'r2' : y_angle, 'r3' : z_angle, 'r4' : 1.000}


                rospy.loginfo("Go to (%s, %s) pose", position_aruco_marker['x'], position_aruco_marker['y'])
            
                success_reach_aruco = aruco_marker_location.goto(position_aruco_marker, quaternion_aruco_marker)

                if success_reach_aruco:
                    rospy.loginfo("Hooray, reached the Aruco Marker")

                    #command to execute arm grab stuff


                    navigator = GoToPose()

                    # Customize the following values so they are appropriate for your location
                    position = {'x': 0, 'y' : 0}
                    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

                    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                
                    success = navigator.goto(position, quaternion)
                    
                    if success:
                        rospy.loginfo("Hooray, reached the desired pose")
                        exit()
                    else:
                        rospy.loginfo("The base failed to reach the desired pose")
                    # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)



                    exit()
                else:
                    rospy.loginfo("The base failed to reach the Aruco Marker")


            elif((finished_map or cancelled_map) and not (aruco2 or aruco1)):
                #Execute code to do roomba sweep

                package = 'clean_robot'
                executable = 'clean_work'
                node = roslaunch.core.Node(package, executable)

                robo_clean_run = True

                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()
                #Start running clean robot?

                robo_clean = launch.launch(node)

                



    
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
