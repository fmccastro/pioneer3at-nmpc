#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, tf

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from std_msgs.msg import Float64MultiArray, Float64, Twist, Int32

from pioneer3at_control.msg import poseRef, pose3D

from classes import Common

if __name__ == '__main__':

    common = Common()
    
    rospy.init_node( 'dataOnlineProc', anonymous = True )

    rospy.Subscriber( '/pioneer3at/robotPose', pose3D, common._poseCallback, 2 )                                #   '/pioneer3at/robotPose' -> topic for pose
    rospy.Subscriber( '/pioneer3at/currentStep', Int32, common._Int32Callback, 0 )                              #   '/pioneer3at/currentStep' -> topic for cycle step
    rospy.Subscriber( '/pioneer3at/currentSol', Float64MultiArray, common._Float64MultiArrayCallback, 0 )       #   '/pioneer3at/currentSol' -> topic for states horizon
    rospy.Subscriber( '/pioneer3at/currentRef', Float64MultiArray, common._optPathSequence, 1 )                 #   '/pioneer3at/currentRef' -> topic for current reference to follow
    rospy.Subscriber( '/pioneer3at/clock', Float64, common._Float64allback, 0 )                                 #   '/pioneer3at/clock' -> topic for simulation time counting ( since the robot starts moving )
    rospy.Subscriber( '/pioneer3at/cycleTime', Float64, common._Float64Callback, 1 )                            #   '/pioneer3at/cycleTime' -> topic for cycle iteration timing count
    rospy.Subscriber( '/pioneer3at/optTime', Float64, common._Float64Callback, 2 )                              #   '/pioneer3at/optTime' -> topic for optimization time
    rospy.Subscriber( '/pioneer3at/cmd_vel', Twist, common._TwistCallback )                                     #   '/pioneer3at/cmd_vel' -> topic for pose
    rospy.Subscriber( '/pioneer3at/distance', Float64, common._Float64Callback, 3 )                             #   '/pioneer3at/distance' -> topic for distance made by the robot 
    rospy.Subscriber( '/pioneer3at/error', Float64MultiArray, common._optPathSequence, 2 )                      #   '/pioneer3at/error' -> topic to publish error of the robot (distance to desired reference)

    #   Resample topics
    pubResRobotPose = rospy.Publisher( '/pioneer3at/resample/robotPose', pose3D )                               #   '/pioneer3at/resample/robotPose'    -> topic for pose
    pubResCurrentStep = rospy.Publisher( '/pioneer3at/resample/currentStep', Int32 )                            #   '/pioneer3at/resample/currentStep'  -> topic for cycle step
    pubResCurrentSol = rospy.Publisher( '/pioneer3at/resample/currentSol', Float64MultiArray )                  #   '/pioneer3at/resample/currentSol'   -> topic for states horizon
    pubResCurrentRef = rospy.Publisher( '/pioneer3at/resample/currentRef', Float64MultiArray )                  #   '/pioneer3at/resample/currentRef'   -> topic for current reference to follow
    pubResClock = rospy.Publisher( '/pioneer3at/resample/clock', Float64 )                                      #   '/pioneer3at/resample/clock'        -> topic for simulation time counting ( since the robot starts moving )
    pubResCycleTime = rospy.Publisher( '/pioneer3at/resample/cycleTime', Float64 )                              #   '/pioneer3at/resample/cycleTime'    -> topic for cycle iteration timing count
    pubResOptTime = rospy.Publisher( '/pioneer3at/resample/optTime', Float64 )                                  #   '/pioneer3at/resample/optTime'      -> topic for optimization time    
    pubResCmd_Vel = rospy.Publisher( '/pioneer3at/resample/cmd_vel', Twist )                                    #   '/pioneer3at/resample/cmd_vel'      -> topic for pose
    pubResDistance = rospy.Publisher( '/pioneer3at/resample/distance', Float64 )                                #   '/pioneer3at/resample/distance'     -> topic for distance made by the robot 
    pubResError = rospy.Publisher( '/pioneer3at/resample/error', Float64MultiArray )                            #   '/pioneer3at/resample/error'        -> topic to publish error of the robot (distance to desired reference)

    while( not rospy.is_shutdown() ):
        try:



        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            print( "[data.py] Something went wrong!" )

    rospy.spin()