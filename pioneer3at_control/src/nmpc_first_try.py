#!/usr/bin/env python
import rospy
import math
import tf
import threading
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.optimize import minimize
from scipy.optimize import Bounds
from scipy.spatial.transform import Rotation

current_position = Odometry()
flag = 1
acquire_tf = threading.Semaphore(1)

# This script was made before coding NMPC with ACADO

def get_pose(msg):
    
    global flag
    global current_position 
    
    if flag == 1:
        current_position = msg
        flag = 0
    elif flag == 0:
        pass

def process_model(pose, time_step, rot_matrix, command):
    
    actuation_world_frame = [ rot_matrix[0][0] * command[0], 
                                rot_matrix[1][0] * command[0], 
                                    rot_matrix[2][0] * command[0],
                                        rot_matrix[0][2] * command[1],
                                            rot_matrix[1][2] * command[1], 
                                                rot_matrix[2][2] * command[1] ] 

    return list( np.array( pose ) + time_step * np.array( actuation_world_frame ) )

def cost_function(commands, *fixed_args): #= (pose, hor, ctrl_weight, rot_matrix, time_step)):

    args = []
    for element in fixed_args:
        args.append(element)

    pose = []

    pose.append( list ( np.array( args[0] ) ) )
    index = 0
    final_result = 0

    while( index < args[1] ):

        print(index)

        if( index == 1 ):
            pose.append( process_model( pose[index - 1], args[4], 
                            args[3], [ commands[ index * 2 ], commands[ index * 2 + 1 ] ] ) )
        elif( index > 1 ):
            # compute rotation matrix
            
            rotation_matrix_aux = Rotation.from_rotvec( [ pose[index - 1][3], 
                                                          pose[index - 1][4], 
                                                          pose[index - 1][5] ] ) 

            rotation_matrix = rotation_matrix_aux.as_dcm()
            
            pose.append( process_model( pose[ index - 1 ], args[ 4 ], 
                            rotation_matrix, [ commands[ index * 2 ], commands[ index * 2 + 1 ] ] ) )
        else:
            pass
        
        final_result = np.linalg.norm( pose[index] )**2 + \
                                    args[2] * \
                        np.linalg.norm( [ commands[ index * 2 ], commands[ index * 2 + 1 ] ] )**2          
        index = index + 1
        
    return final_result

if __name__ == '__main__':

    rospy.init_node('nmpc', anonymous = True)       # node initialization
    
    listener = tf.TransformListener()               # listening to TF topic

    actuation = Twist()

    horizon_length = 10                             # horizon length definition
    control_weight = 0.2                            # weight of the NMPC cost function
    time_step = 5                                   # time step of the NMPC prior process model

    # Control constraints #
    vel_lower_bd = -0.7
    ang_vel_lower_bd = -140 * ( math.pi / 180 )
    vel_upper_bd = 0.7
    ang_vel_upper_bd = 140 * ( math.pi / 180 )
    #######################
    
    index = 0
    lower_bounds = []
    upper_bounds = []
    
    while ( index < horizon_length ):
        lower_bounds.append( vel_lower_bd )
        lower_bounds.append( ang_vel_lower_bd )
        upper_bounds.append( vel_upper_bd )
        upper_bounds.append( ang_vel_upper_bd )
        index = index + 1

    bounds = Bounds( lower_bounds, upper_bounds )                                   # Control constraints initialization
    
    rospy.Subscriber('/pioneer3at/odom', Odometry, get_pose)                        # Subscription to odometry topic
    next_action = rospy.Publisher('/pioneer3at/cmd_vel', Twist, queue_size = 1)     # Control Publisher initilization 

    rate = rospy.Rate( 5 )

    # NMPC cycle
    while not rospy.is_shutdown():
        try:
            # Retrieval of rotation matrix w.r.t the world frame origin at the respective time 
            (trans,rot) = listener.lookupTransform('/base_link', '/odom', 
                rospy.Time(secs = current_position.header.stamp.secs,
                            nsecs = current_position.header.stamp.nsecs))
            ##################################################################################
            
            R_aux = Rotation.from_quat(rot)     

            R = R_aux.as_dcm()                      # Conversion of the rotation quaternion to rotation matrix format
            orientation_angles = R_aux.as_rotvec()  # Conversion of the rotation quaternion to pitch-roll-yaw format
            initial_pose = [ trans[0], trans[1], trans[2], 
                                orientation_angles[0], orientation_angles[1], orientation_angles[2] ]   # Initial pose definition

            index = 0
            ini_guess = []

            while ( index < horizon_length ):
                ini_guess.append( 0 )
                ini_guess.append( 0 )
                index = index + 1

            print("Beginning of minimization")
            
            # Cost function minimization #
            result = minimize( cost_function, ini_guess, 
                                args = ( initial_pose, horizon_length, 
                                    control_weight, R, time_step ), 
                                        method = 'trust-constr', options = {'verbose': 1}, bounds = bounds )
            ##############################

            print("End of minimization")
            
            print(result)

            actuation.linear.x = ( result[ "x" ] )[0] 
            actuation.linear.y = 0
            actuation.linear.z = 0
            actuation.angular.x = 0
            actuation.angular.y = 0
            actuation.angular.z = ( result[ "x" ] )[1]

            next_action.publish( actuation )        # First control input of the minimization is published for 10 seconds
            rospy.sleep(10)                          

            actuation.linear.x = 0 
            actuation.linear.y = 0
            actuation.linear.z = 0
            actuation.angular.x = 0
            actuation.angular.y = 0
            actuation.angular.z = 0

            next_action.publish( actuation )        # After 10 seconds, the control input is set to zero, so that the next NMPC
                                                    # cycle begins
            flag = 1                                # When flag is set to 1, the position of the robot (which is being published
                                                    # continuously) is saved        

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            #print( "Something went wrong!" )