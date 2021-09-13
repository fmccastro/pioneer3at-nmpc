#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, tf, sys, time, pickle, os

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

from scipy.io import savemat
from multiprocessing import Process, Queue
from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Float64, Float64MultiArray
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from classes import Common, Model, LGP, SOGP

from pioneer3at_control.msg import pose3D
from pioneer3at_control.srv import getPath

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/scripts")

gamepad = Joy()

def joyCallback( msg ):

    global gamepad
    gamepad = msg

if __name__ == '__main__':

    common = Common()

    #   LGP
    if( common.gpType == 0 ):
        gp = LGP( common )

    #   SOGP
    elif( common.gpType == 1 ):
        gp = SOGP( common )

    model = Model( common, gp )

    rospy.init_node( 'getTrainingDataGP', anonymous = True )

    rospy.Subscriber( '/pioneer3at/robotPose', pose3D, common._poseCallback, 2 )                    #   '/pioneer3at/robotPose' -> topic for pose
    rospy.Subscriber( '/pioneer3at/robotVel', Twist, common._TwistCallback, 1 )                     #   '/pioneer3at/robotVel' -> topic for robot velocity
    rospy.Subscriber( '/pioneer3at/cmd_vel', Twist, common._TwistCallback, 0 )                      #   '/pioneer3at/cmd_vel' -> topic to listen actuation inputs
    rospy.Subscriber( '/joy', Joy, joyCallback )

    while( rospy.get_param("/init") > 1  ):
        continue

    common._pauseFunction( "[trainGP.py] Press [ENTER] to activate node." )
    print( "[trainGP.py] It's active." )

    r = rospy.Rate(10)

    index = 0

    while( not rospy.is_shutdown() and index < common.nbTrainingPoints + 2 ):

        try:
            if( gamepad.buttons[5] == 1 ):

                start = time.time()

                if( index >= 2 ):
                    prevPrevPose = previousPose
                    prevPrevVel = previousVel

                    prevPrevActuation = previousActuation

                    ###

                    previousPose = pose
                    previousVel = vel

                    previousActuation = actuation

                    ###

                    pose = common.robotPose
                    vel = common.velocity
                    
                    actuation = common.actuation

                    #   { u_k, u_{k-1}, v_{k-1} }
                    if( common.gpModel == 1 ):
                        rawInput = np.hstack( ( gp._getOrientation( previousPose ),\
                                                gp._getVelocity( prevPrevVel ),\
                                                gp._getControls( previousActuation ),\
                                                gp._getControls( prevPrevActuation ) ) )

                    auxPose = np.array( [ [ previousPose.x, previousPose.y, previousPose.yaw ] ] ).T
                    auxCtrl = np.array( [ [ previousActuation.linear.x, previousActuation.angular.z ] ] ).T

                    predictionPose = model._ODE( auxPose, auxCtrl, cycleTime )

                    rawOutput = np.array( [ [ pose.x - predictionPose[0], pose.y - predictionPose[1], common._shortestAngle( predictionPose[2], pose.yaw ) ] ] )

                    print(index)
                    gp._updateRawInData( rawInput )
                    gp._updateRawOutData( rawOutput )

                elif( index >= 1 ):
                    previousPose = pose
                    previousVel = vel
                    
                    previousActuation = actuation

                    ###

                    pose = common.robotPose
                    vel = common.velocity

                    actuation = common.actuation
                
                else:
                    pose = common.robotPose
                    vel = common.velocity

                    actuation = common.actuation
                
                index += 1
                r.sleep()
                cycleTime = time.time() - start

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            print( "[trainGP.py] Something went wrong!" )

    print("[trainGP.py] GP training ended.")

    input, output = gp._trainingData()

    ###
    os.remove(common.pathInputTrainingData_np)
    os.remove(common.pathOutputTrainingData_np) 

    # Creates a new file
    with open(common.pathInputTrainingData_np, 'w') as input_np:
        pass

    with open(common.pathOutputTrainingData_np, 'w') as output_np:
        pass

    np.save(common.pathInputTrainingData_np, input)
    np.save(common.pathOutputTrainingData_np, output)

    ###
    os.remove(common.pathInputTrainingData_mat)
    os.remove(common.pathOutputTrainingData_mat)

    # Creates a new file
    with open(common.pathInputTrainingData_mat, 'w') as input_mat:
        pass

    with open(common.pathOutputTrainingData_mat, 'w') as output_mat:
        pass

    matInput = {"input": input, "label": "experiment"}
    matOutput = {"output": output, "label": "experiment"}

    savemat(common.pathInputTrainingData_mat, matInput)
    savemat(common.pathOutputTrainingData_mat, matOutput)