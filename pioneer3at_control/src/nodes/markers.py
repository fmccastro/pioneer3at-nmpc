#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, tf
import numpy as np

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from classes import Common, Planner

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int32, Bool, Float64, Float64MultiArray

from pioneer3at_control.msg import pose3D
from pioneer3at_control.srv import getPath, getPathResponse

if __name__ == '__main__':

    common = Common()

    goalPoint = common.goalPoint * ( common.mapLength / common.img.shape[0] ) - common.mapLength/2
    
    pathPlanning = Planner( Path = common.path, NbStates = common.NbStates, NbControls = common.NbControls, Image = common.img, costMap = common.costMap, heightPx = common.heightProportion,\
                                paramVel = common.parameterSpeed, paramVelFM = common.parameterSpeed_FM, length = common.mapLength, option = common.refType,\
                                    samplingTime = common.Ts, goal = goalPoint )

    rospy.init_node( 'markers', anonymous = True )
    
    rospy.Subscriber( '/pioneer3at/robotPose', pose3D, common._poseCallback, 2 )                    #   '/pioneer3at/robotPose' -> topic for pose
    rospy.Subscriber( '/pioneer3at/currentStep', Int32, common._Int32Callback, 0 )                  #   '/pioneer3at/currentStep' -> topic for cycle step
    rospy.Subscriber( '/pioneer3at/currentSol', Float64MultiArray, common._optPathSequence, 0 )     #   '/pioneer3at/currentSol' -> topic for states horizon
    rospy.Subscriber( '/pioneer3at/currentRef', Float64MultiArray, common._optPathSequence, 1 )     #   '/pioneer3at/currentRef' -> topic for current reference to follow
    rospy.Subscriber( '/pioneer3at/clock', Float64, common._clockCallback, 0 )                      #   '/pioneer3at/clock' -> topic for simulation time counting ( since the robot starts moving )
    rospy.Subscriber( '/pioneer3at/cycleTime', Float64, common._clockCallback, 1 )                  #   '/pioneer3at/cycleTime' -> topic for cycle iteration timing count

    refPub = rospy.Publisher( '/pioneer3at/viz/reference', Marker, queue_size = 1 )                     #   '/pioneer3at/reference' -> topic for current reference which goes into the cost function   
    horPub = rospy.Publisher( '/pioneer3at/viz/horizon', Marker, queue_size = 1 )                       #   '/pioneer3at/horizon' -> topic for optimization sucessive states solution throughout the horizon  
    robotPathPub = rospy.Publisher( '/pioneer3at/viz/robotPath', Marker, queue_size = 1 )               #   '/pioneer3at/robotPath' -> topic for robot's pose history visualization
    pathPub = rospy.Publisher( '/pioneer3at/viz/path', Path, queue_size = 1 )                           #   '/pioneer3at/path' -> topic for publishing fixed path from starting to goal

    if( common.refType == 0 ):  #   Generate points from pre-definied path for visualization purposes of a path-tracking reference
        
        ref = pathPlanning._pathSlicing()
        ref = np.hstack( ( ref, pathPlanning._pathOrientation( ref ) ) )

    elif( common.refType == 1 ):    #   Generate points from trajectory for visualization purposes

        refPts, ref = pathPlanning._pathSlicing()
    
    elif( common.refType == 2 ):    #   Generate gradient with fast marching algorithm for reference visualization purposes

        grad_X, grad_Y = pathPlanning._fastMarching()

    #   markersRef -> Refereces points that are put into the NMPC cost function at each iteration
    markersRef = Marker()

    #   markersHor -> Horizon points that comes from the solution at each iteration
    markersHor = Marker()

    #   markersPath -> Path history from the robot
    markersPath = Marker()

    #   horizon -> Pathway from solution
    horizon = Float64MultiArray()
    
    #   path -> Full reference for the robot to follow
    if( common.refType != 2 ):
        path = Path()                                   #   full reference for the robot to follow from start to end
        path.header.seq = 0
        path.header.stamp = rospy.Time.now()

    if( common.poseType == 0 ):

        if( common.refType != 2 ):
            path.header.frame_id = "base_link"
        
        markersRef.header.frame_id = "base_link"
        markersHor.header.frame_id = "base_link"
        markersPath.header.frame_id = "base_link"

    elif( common.poseType == 1 ):

        if( common.refType != 2 ):
            path.header.frame_id = "odom"
        
        markersRef.header.frame_id = "odom"
        markersHor.header.frame_id = "odom"
        markersPath.header.frame_id = "odom"
        
    #   Blue Ref Markers
    markersRef.header.stamp = rospy.Time.now()
    markersRef.ns = "refPoints"
    markersRef.action = 0
    markersRef.id = 0
    markersRef.type = 7
    markersRef.scale.x = 0.1
    markersRef.scale.y = 0.1
    markersRef.scale.z = 0.1
    markersRef.color.r = 0.0
    markersRef.color.g = 0.0
    markersRef.color.b = 1.0
    markersRef.color.a = 1.0

    #   Red Horizon Markers
    markersHor.header.stamp = rospy.Time.now()
    markersHor.ns = "horizonPoints"
    markersHor.action = 0
    markersHor.id = 0
    markersHor.type = 7
    markersHor.scale.x = 0.1
    markersHor.scale.y = 0.1
    markersHor.scale.z = 0.1
    markersHor.color.r = 1.0
    markersHor.color.g = 0.0
    markersHor.color.b = 0.0
    markersHor.color.a = 1.0

    #   Yellow Path history markers
    markersPath.header.stamp = rospy.Time.now()
    markersPath.ns = "robotPathPoints"
    markersPath.action = 0
    markersPath.id = 0
    markersPath.type = 4
    markersPath.scale.x = 0.01
    markersPath.scale.y = 0.01
    markersPath.scale.z = 0.01
    markersPath.color.r = 0.960
    markersPath.color.g = 0.921 
    markersPath.color.b = 0.360
    markersPath.color.a = 1.0
    markersPath.points = []

    if( common.refType == 0 ):

        ref = ref.tolist()
        ref = sum( ref, [] )

        index = 0
        
        while( common.NbStates * index < len( ref ) ):

            path.poses += [ PoseStamped() ]

            if( common.poseType == 0 ):
                path.poses[ index ].header.frame_id = "base_link"
            
            elif( common.poseType == 1 ):
                path.poses[ index ].header.frame_id = "odom"

            path.poses[ index ].header.stamp = rospy.Time.now()

            path.poses[ index ].header.seq = index
            path.poses[ index ].pose.position.x = ref[ index * common.NbStates ]
            path.poses[ index ].pose.position.y = ref[ index * common.NbStates + 1 ]

            index += 1
    
    elif( common.refType == 1 ):

        refPts = refPts.tolist()
        refPts = sum( refPts, [] )

        index = 0
        
        while( common.NbStates * index < len( refPts ) ):

            path.poses += [ PoseStamped() ]

            if( common.poseType == 0 ):
                path.poses[ index ].header.frame_id = "base_link"
            
            elif( common.poseType == 1 ):
                path.poses[ index ].header.frame_id = "odom"

            path.poses[ index ].header.stamp = rospy.Time.now()

            path.poses[ index ].header.seq = index
            path.poses[ index ].pose.position.x = refPts[ index * common.NbStates ]
            path.poses[ index ].pose.position.y = refPts[ index * common.NbStates + 1 ]

            index += 1

    index = 0
    r = rospy.Rate( 1 / common.Ts )

    #   Waiting for simulation to start in order to publish visualization markers
    while( rospy.get_param( "/init" ) != 0 ):
        continue

    print( "[markers.py] It's active." )

    while( not rospy.is_shutdown() ):
        try:
            
            pose = common.robotPose
            step = common.step
            clock = common.clock
            Ts = common.cycleTime
            horizon = common.pathSequence
            reference = common.refSequence

            markersRef.points = []
            markersHor.points = []
            markersPath.points += [ Point( pose.x, pose.y, 0 ) ]

            for j in range( common.N ):

                markersRef.points += [ Point( reference.data[ j * common.NbStates ], reference.data[ j * common.NbStates + 1 ], 0 ) ]
                markersHor.points += [ Point( horizon.data[ j * common.NbStates ], horizon.data[ j * common.NbStates + 1 ], 0 ) ]

            robotPathPub.publish( markersPath )
            horPub.publish( markersHor )
            refPub.publish( markersRef )

            if( common.refType != 2 ):
                pathPub.publish( path )

            #r.sleep()
        
            index += 1

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            print( "[markers.py] Something went wrong!" )

    rospy.spin()