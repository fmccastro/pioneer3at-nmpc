#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, math, rospy, tf, time

import numpy as np

from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation
from pioneer3at_control.msg import pose3D

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from classes import Common

if __name__ == '__main__':

    common = Common()

    rospy.init_node( 'poseSelectorTrainGP', anonymous = True )

    rospy.Subscriber( '/gazebo/link_states', LinkStates, common._poseCallback, 0 )                  #   '/gazebo/link_states' -> topic which collects perfect sensors data 
    rospy.Subscriber( '/filtered/odometry', Odometry, common._poseCallback, 1 )                     #   '/filtered/odometry' -> topic which collects fusioned data from the sensors of the robot 

    pubPose = rospy.Publisher( '/pioneer3at/robotPose', pose3D, queue_size = 1 )                    #   '/pioneer3at/robotPose' -> topic for pose
    
    common._pauseFunction( "[poseSelector.py] Press [ENTER] to activate node." )

    print( "[poseSelector.py] It's active." )

    #   Change "/init" parameter in order to allow the following node to initialize
    rospy.set_param( "/init", rospy.get_param("/init") - 1 )

    if( common.terraOnOff ):

        rospy.wait_for_service( '/pioneer3at/triggerTerramechanics' )
        triggerTerramechanics = rospy.ServiceProxy( '/pioneer3at/triggerTerramechanics', Trigger )

        res = triggerTerramechanics()

        if( res.success ):
            print( "[poseSelector.py] Terramechanics sim is ON." + res.message )
    
    pose = pose3D()

    while( not rospy.is_shutdown() ):
        try:

            pose = common._selectPoseType( common.poseType )

            pubPose.publish( pose )

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            print( "Something went wrong!" )
    
    rospy.spin()