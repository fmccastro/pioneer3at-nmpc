#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, tf

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from std_msgs.msg import Float64MultiArray, Float64, Int32
from geometry_msgs.msg import Twist

from pioneer3at_control.msg import pose3D, resampleData

from classes import Common

if __name__ == '__main__':

    common = Common()
    
    rospy.init_node( 'dataOnlineProc', anonymous = True )

    rospy.Subscriber( '/pioneer3at/robotPose', pose3D, common._poseCallback, 2 )                                #   '/pioneer3at/robotPose'     -> topic for pose
    rospy.Subscriber( '/pioneer3at/currentStep', Int32, common._Int32Callback, 0 )                              #   '/pioneer3at/currentStep'   -> topic for cycle step
    rospy.Subscriber( '/pioneer3at/currentSol', Float64MultiArray, common._Float64MultiArrayCallback, 0 )       #   '/pioneer3at/currentSol'    -> topic for states horizon
    rospy.Subscriber( '/pioneer3at/currentRef', Float64MultiArray, common._Float64MultiArrayCallback, 1 )       #   '/pioneer3at/currentRef'    -> topic for current reference to follow
    rospy.Subscriber( '/pioneer3at/clock', Float64, common._Float64Callback, 0 )                                #   '/pioneer3at/clock'         -> topic for simulation time counting ( since the robot starts moving )
    rospy.Subscriber( '/pioneer3at/cycleTime', Float64, common._Float64Callback, 1 )                            #   '/pioneer3at/cycleTime'     -> topic for cycle iteration timing count
    rospy.Subscriber( '/pioneer3at/optTime', Float64, common._Float64Callback, 2 )                              #   '/pioneer3at/optTime'       -> topic for optimization time
    rospy.Subscriber( '/pioneer3at/cmd_vel', Twist, common._TwistCallback, 0 )                                  #   '/pioneer3at/cmd_vel'       -> topic for pose
    rospy.Subscriber( '/pioneer3at/distance', Float64, common._Float64Callback, 3 )                             #   '/pioneer3at/distance'      -> topic for distance made by the robot 
    rospy.Subscriber( '/pioneer3at/error', Float64MultiArray, common._Float64MultiArrayCallback, 2 )            #   '/pioneer3at/error'         -> topic to publish error of the robot (distance to desired reference)

    #   Resample topics
    pubResample = rospy.Publisher( '/pioneer3at/resample', resampleData, queue_size = 1 )                       #   '/pioneer3at/resample'    -> topic for resampling data

    #   Waiting for permission to initialize node
    while( rospy.get_param("/init") > common.dataProc ):
        continue
    
    print( "[dataOnlineProc.py] It's active." )
    
    resData = resampleData()

    while( not rospy.is_shutdown() ):
        try:

            pose = common.robotPose
            step = common.step
            currentSol = common.pathSequence
            currentRef = common.refSequence
            clock = common.clock
            cycleTime = common.cycleTime
            optTime = common.optTime
            cmdVel = common.actuation
            distance = common.distance
            error = common.error

            resData.pose = pose
            resData.step = step.data
            resData.solution = currentSol.data
            resData.reference = currentRef.data
            resData.clock = clock.data
            resData.cycleTime = cycleTime.data
            resData.optTime = optTime.data
            resData.actuation = cmdVel
            resData.distance = distance.data
            resData.error = error.data

            pubResample.publish( resData )

            if( cycleTime.data == 0 ):
                continue
            
            else:
                freq = abs( 1.0/cycleTime.data )

                rate = rospy.Rate( freq )

                rate.sleep()

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            print( "[data.py] Something went wrong!" )

    rospy.spin()