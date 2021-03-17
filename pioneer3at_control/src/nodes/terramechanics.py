#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, tf, math, sys, time
import numpy as np

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from std_srvs.srv import Trigger, TriggerResponse
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetPhysicsProperties, ApplyBodyWrench, BodyRequest
from geometry_msgs.msg import Twist, Wrench, Point
from classes import Common
from casadi import*

joint = JointState()
framesTf = TFMessage()

def tfCallback( msg ):

    global framesTf
    framesTf = msg

def jointsCallback( msg ):
    
    global joint
    joint = msg

def srvCall( option, torque = None, horizontalForce = None, link = None, signal = None, wheelOrientation = None ):

    if( option == 0 ):

        rospy.wait_for_service('/gazebo/get_physics_properties')

        try:
            physicsProp = rospy.ServiceProxy( '/gazebo/get_physics_properties', GetPhysicsProperties )

            resp = physicsProp()

            return resp.gravity.z

        except rospy.ServiceException as e:
            print("[terramechanics.py] Service call failed: %s"%e)
    
    elif( option == 1 ):
        
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        rospy.wait_for_service('/gazebo/clear_body_wrenches')

        wrench = Wrench()

        pitch = wheelOrientation[1]

        if( pitch >= 0  and pitch < math.pi/2 ):
            wrench.force.x = horizontalForce * signal * math.cos( pitch )
            wrench.force.y = 0
            wrench.force.z = -horizontalForce * signal * math.sin( pitch )
            wrench.torque.x = 0
            wrench.torque.y = torque * signal
            wrench.torque.z = 0
        
        elif( pitch >= math.pi/2 and pitch < math.pi ):
            wrench.force.x = -horizontalForce * signal * math.sin( pitch - math.pi/2 )
            wrench.force.y = 0
            wrench.force.z = -horizontalForce * signal * math.cos( pitch - math.pi/2 )
            wrench.torque.x = 0
            wrench.torque.y = torque * signal
            wrench.torque.z = 0
        
        elif( pitch >= -math.pi and pitch < -math.pi/2 ):
            wrench.force.x = -horizontalForce * signal * math.cos( pitch + math.pi )
            wrench.force.y = 0
            wrench.force.z = horizontalForce * signal * math.sin( pitch + math.pi )
            wrench.torque.x = 0
            wrench.torque.y = torque * signal
            wrench.torque.z = 0
        
        elif( pitch >= -math.pi/2 and pitch < 0 ):
            wrench.force.x = horizontalForce * signal * math.sin( pitch + math.pi/2 )
            wrench.force.y = 0
            wrench.force.z = horizontalForce * signal * math.cos( pitch + math.pi/2 )
            wrench.torque.x = 0
            wrench.torque.y = torque * signal
            wrench.torque.z = 0

        try:
            applyWrench = rospy.ServiceProxy( '/gazebo/apply_body_wrench', ApplyBodyWrench )
            clearWrench = rospy.ServiceProxy( '/gazebo/clear_body_wrenches', BodyRequest )

            clearWrench( body_name = "pioneer3at_robot::" + link[:-6] )

            applyWrench( body_name = "pioneer3at_robot::" + link[:-6],\
                         reference_frame = "pioneer3at_robot::" + link[:-6],\
                         reference_point = Point( x = 0 , y = 0 , z = 0 ),\
                         wrench = wrench,\
                         start_time = rospy.Time(1000),\
                         duration = rospy.Duration(1000) )

        except rospy.ServiceException as e:
            print("[terramechanics.py] Service call failed: %s"%e)
    
def triggerCall( request ):

    return TriggerResponse( success = True, message = "Forces sim started.")

if __name__ == '__main__':

    flag = True

    common = Common()

    if( common.terraOnOff ):

        entry = Float32MultiArray()
        slip = Float32MultiArray()
        T = Float32MultiArray()
        DP = Float32MultiArray()

        rospy.init_node( 'terramechanics', anonymous = True )

        slipPub = rospy.Publisher( '/pioneer3at/slip', Float32MultiArray, queue_size = 1 )
        torquePub = rospy.Publisher( '/pioneer3at/torque', Float32MultiArray, queue_size = 1 )
        dpPub = rospy.Publisher( '/pioneer3at/dp', Float32MultiArray, queue_size = 1 )
        entryPub = rospy.Publisher( '/pioneer3at/entryAngle', Float32MultiArray, queue_size = 1 )

        rospy.Subscriber( '/twistPose', Twist, common._poseCallback, 2 )
        rospy.Subscriber( '/joint_states', JointState, jointsCallback )
        rospy.Subscriber( '/gazebo/link_states', LinkStates, common._poseCallback, 0 )
        rospy.Subscriber( '/tf', TFMessage, tfCallback )
        
        theta_1 = SX.sym( 'theta_1' )
        w = SX.sym( 'w' )
        theta_m = theta_1 / 2

        sigma_m = ( common.k_c / common.b + common.k_phi ) * ( common.r * ( cos( theta_m ) - cos( theta_1 ) ) )**common.n

        f_sigma_m = Function( 'f_sigma_m', [ theta_1 ], [ sigma_m ] )

        service = rospy.Service( '/pioneer3at/triggerTerramechanics', Trigger, triggerCall )

        gravity = srvCall( 0 )

        index = 0

        while( not rospy.is_shutdown() ):
            try:
                
                start = time.time()

                if( index > 0 ):
                    pose_i_prev = pose_i
                    velocity_i_prev = velocity_i

                pose_i = common.twistPose

                if( common.poseType == 0 ):
                    velocity_i = common.truePose.twist[1].linear
                
                else:
                    velocity_i = common.fusedPose.twist.twist.linear

                if( flag ):
                    time.sleep( 1 )
                    flag = False

                else:

                    if( index > 0 ):

                        velocity = np.array( [ velocity_i_prev.x, velocity_i_prev.y, velocity_i_prev.z ] )
                        velocityMod = np.linalg.norm( velocity )
                        #print( "Velocity: ", velocity )
                        #print( "Velocity Modulus: ", velocityMod )
                        
                        if( abs( pose_i_prev.angular.x ) > math.pi / 2 or abs( pose_i_prev.angular.y ) > math.pi / 2 ):
                            pass

                        elif( velocityMod < 0.1 ):
                            pass

                        else:

                            orientationVector = np.array( [ math.cos( pose_i_prev.angular.z ), math.sin( pose_i_prev.angular.z ), math.sin( pose_i_prev.angular.y ) ] )
                            #print( "Orientation Vector: ", orientationVector )

                            velocity_X = np.dot( velocity, orientationVector ) * orientationVector / math.pow( np.linalg.norm( orientationVector ), 2 )
                            #print( "Velocity X_body: ", velocity_X )

                            velocity_X_Sign = np.linalg.norm( velocity_X ) * np.sign( np.dot( velocity, orientationVector ) )
                            #print( "Velocity X_Body_Sign: ", velocity_X_Sign )

                            jointStates = joint
                            #print( joint )

                            slipRatio = []
                            dp = []
                            torque = []
                            entryAngle = []
                            refs = []

                            for i in range( common.nbWheels ):

                                if( abs( common.r * jointStates.velocity[i] ) >= abs( np.linalg.norm( velocity_X_Sign ) ) ):

                                    slipRatio += [ ( common.r * jointStates.velocity[i] - velocity_X_Sign ) / ( common.r * jointStates.velocity[i] ) ]

                                else:

                                    slipRatio += [ ( common.r * jointStates.velocity[i] - velocity_X_Sign ) / velocity_X_Sign ]

                                tau_m = ( common.c + sigma_m * tan( common.phi ) ) * ( 1 - exp( -common.r * ( theta_1 - theta_m - ( 1 - slipRatio[-1] ) * ( sin( theta_1 ) - sin( theta_m ) ) ) / common.k ) )

                                g0 = w - fabs( ( common.r * common.b / ( theta_m * ( theta_1 - theta_m ) ) ) * ( tau_m * ( theta_1 * sin( theta_m ) - theta_m * sin( theta_1 ) )\
                                        + sigma_m * ( theta_1 * cos( theta_m ) - theta_m * cos( theta_1 ) - theta_1 ) +
                                            common.c * ( theta_m * sin( theta_m ) - theta_1 * sin( theta_m ) + theta_1 * theta_m - theta_m**2 ) ) )
                                
                                f_tau_m = Function( 'f_tau_m', [ theta_1 ], [ tau_m ] )
                                g = Function( 'g', [ theta_1, w ], [ g0 ] )

                                G = rootfinder( 'G', 'newton', g )

                                entryAngle += [ G( 0.5, common.m * abs( gravity ) * cos( pose_i_prev.angular.x ) * cos( pose_i_prev.angular.y ) / 4 ) ]

                                torque += [ ( math.pow( common.r, 2 ) * common.b / 2 ) * ( f_tau_m( entryAngle[-1] ) * entryAngle[-1] + common.c * entryAngle[-1] / 2 ) ]
                                
                                dp1 = common.c * math.sin( entryAngle[-1] / 2 ) + ( f_tau_m( entryAngle[-1] / 2 ) - common.c )\
                                        * ( math.cos( entryAngle[-1] / 2 ) + entryAngle[-1] / 2 * math.sin( entryAngle[-1] / 2 ) - 1 ) / ( entryAngle[-1] / 2 )
                                
                                dp2 = ( f_tau_m( entryAngle[-1] / 2 ) / ( entryAngle[-1] / 2 ) ) * ( -entryAngle[-1] / 2 * math.sin( entryAngle[-1] / 2 ) + math.cos( entryAngle[-1] / 2 ) - cos( entryAngle[-1] ) )

                                dp3 = f_sigma_m( entryAngle[-1] / 2 ) * ( math.sin( entryAngle[-1] / 2 ) - entryAngle[-1] / 2 * math.cos( entryAngle[-1] / 2 ) ) / ( entryAngle[-1] / 2 )

                                dp4 = f_sigma_m( entryAngle[-1] / 2 ) / ( entryAngle[-1] / 2 ) * ( math.cos( entryAngle[-1] / 2 ) * ( entryAngle[-1] / 2 ) + math.sin( entryAngle[-1] / 2 ) - math.sin( entryAngle[-1] ) )

                                dp += [ common.r * common.b * ( dp1 + dp2 - dp3 - dp4 ) ]

                            for i in range( common.nbWheels ):

                                if( i == 0 ):

                                    rpy = common._quaternionToRPY( framesTf.transforms[2].transform.rotation )
                                
                                elif( i == 1 ):

                                    rpy = common._quaternionToRPY( framesTf.transforms[3].transform.rotation )
                                
                                elif( i == 2 ):

                                    rpy = common._quaternionToRPY( framesTf.transforms[0].transform.rotation )
                                
                                elif( i == 3 ):

                                    rpy = common._quaternionToRPY( framesTf.transforms[1].transform.rotation )

                                if( velocity_X_Sign >= 0 ):
                                    srvCall( 1, torque = torque[i], horizontalForce = dp[i], link = str( jointStates.name[i] ), signal = -1, wheelOrientation = rpy )
                                
                                else:
                                    srvCall( 1, torque = torque[i], horizontalForce = dp[i], link = str( jointStates.name[i] ), signal = 1, wheelOrientation = rpy )

                            entry.data = entryAngle
                            slip.data = slipRatio
                            T.data = torque
                            DP.data = dp

                            entryPub.publish( entry )
                            slipPub.publish( slip )
                            torquePub.publish( T )
                            dpPub.publish( DP )
                
                index += 1

            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print( "[terramechanics.py] Something went wrong!" )
    
    else:
        print( "[terramechanics.py] No terramechanics sim. Node is shuting down." )