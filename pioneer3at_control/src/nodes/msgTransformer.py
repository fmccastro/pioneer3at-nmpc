#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, sys, tf

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from classes import Common

if __name__ == '__main__':

    common = Common()

    rospy.init_node( 'msgTransformer', anonymous = True )

    rospy.Subscriber( '/filtered/odometry', Odometry, common._poseCallback, 1 )
    pose_pub = rospy.Publisher( '/filtered/pose', PoseWithCovarianceStamped, queue_size = 1 )

    odometry = Odometry()
    stampedPose = PoseWithCovarianceStamped()

    r = rospy.Rate(50)

    while( not rospy.is_shutdown() ):
        try:
            odometry = common.fusedPose

            stampedPose.header = odometry.header
            stampedPose.pose = odometry.pose

            pose_pub.publish( stampedPose )
        
        except( tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException ):
            pass
            print( "Something went wrong!" )
        
        #r.sleep()