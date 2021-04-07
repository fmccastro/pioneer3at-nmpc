#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, tf, math
import numpy as np

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from std_msgs.msg import Float64MultiArray, Float64

from pioneer3at_control.msg import poseRef, pose3D

from classes import Common

if __name__ == '__main__':

    common = Common()
    
    rospy.init_node( 'data', anonymous = True )

    rospy.Subscriber( '/pioneer3at/pRef', poseRef, common._poseRefCallback )
    rospy.Subscriber( '/pioneer3at/robotPose', pose3D, common._poseCallback, 2 )                #   '/pioneer3at/robotPose' -> topic for pose

    errorPub = rospy.Publisher( '/pioneer3at/error', Float64MultiArray, queue_size = 1 )        #   '/pioneer3at/error' -> topic to publish error of the robot (distance to desired reference)  
    distancePub = rospy.Publisher( '/pioneer3at/distance', Float64, queue_size = 1 )            #   '/pioneer3at/distance' -> topic for distance made by the robot            

    #   Waiting for permission to initialize node
    while( rospy.get_param("/init") > common.data ):
        continue

    print( "[data.py] It's active." )

    index = 0

    error = Float64MultiArray()
    distance = Float64()

    distance.data = 0
    pose = common.robotPose

    distancePub.publish( distance )

    while( not rospy.is_shutdown() ):
        try:

            if( index == 1 ):
                #   Change "/init" parameter in order to allow the following node to initialize
                rospy.set_param( "/init", rospy.get_param("/init") - 1 )
            
            prevPose = pose
            
            pose = common.robotPose
            
            poseRef = common.poseRef
            
            ###     Error is computed with respect to the desired referece in the reference frame, not the global frame
            errorXY = np.array( [ [ math.cos( poseRef.reference[2] ), -math.sin( poseRef.reference[2] ) ],\
                                    [ math.sin( poseRef.reference[2] ), math.cos( poseRef.reference[2] ) ] ] ).\
                                        dot( np.array( [ poseRef.reference[0] - poseRef.pose.x, poseRef.reference[1] - poseRef.pose.y ] ) )
            
            errorX = errorXY[0]
            errorY = errorXY[1]
            errorYaw = common._shortestAngle( poseRef.pose.yaw, poseRef.reference[2] )

            #   Assign error data to publish it afterwards
            error.data = [ errorX, errorY, errorYaw ]

            #   Sum distance covered by the robot
            distance.data += common._distanceBtwPoints( prevPose, pose, 1 )

            errorPub.publish( error )                                                           #   Publish error with respective to the current reference
            distancePub.publish( distance )                                                     #   Publish increasing distance covered by the robot
        
            index += 1

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            print( "[data.py] Something went wrong!" )

    rospy.spin()