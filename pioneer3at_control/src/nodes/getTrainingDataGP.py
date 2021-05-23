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
from classes import Common, Model, LGP, localModel, LGP_Record

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

    model = Model( Nx = common.NbStates, Nu = common.NbControls,\
                    ode = common._ode, J = common._costFunction, controlIntervals = common.N,\
                        weightMatrix_1 = common.Q, weightMatrix_2 = common.R, samplingTime = common.Ts, intAcc = common.intAccuracy, spaceSetLowerBounds = common.X_lb, spaceSetUpperBounds = common.X_ub,\
                             controLowerBounds = common.U_lb, controlUpperBounds = common.U_ub, transMethod = common.transMet, optimization = common.optType, gpOnOff = common.gpOnOff )

    gp = LGP( inputDimension = common.LGP.nbInputs, outputDimension = common.LGP.nbOutputs, limitValue = common.LGP.limitValue, maxNbDataPts = common.LGP.maxDataPts )

    rospy.init_node( 'getTrainingDataGP', anonymous = True )

    rospy.Subscriber( '/pioneer3at/robotPose', pose3D, common._poseCallback, 2 )                    #   '/pioneer3at/robotPose' -> topic for pose
    rospy.Subscriber( '/pioneer3at/cmd_vel', Twist, common._TwistCallback, 0 )                      #   '/pioneer3at/cmd_vel' -> topic to listen actuation inputs
    rospy.Subscriber( '/joy', Joy, joyCallback )

    while( rospy.get_param("/init") > 1  ):
        continue

    common._pauseFunction( "[trainGP.py] Press [ENTER] to activate node." )
    print( "[trainGP.py] It's active." )

    r = rospy.Rate(10)

    index = 0

    while( not rospy.is_shutdown() and index < common.LGP.nbTrainingPoints + 2 ):

        try:
            if( gamepad.buttons[5] == 1 ):

                start = time.time()

                if( index >= 2 ):
                    prevPrevPose = previousPose
                    prevPrevActuation = previousActuation

                    previousPose = pose
                    previousActuation = actuation

                    pose = common.robotPose
                    actuation = common.actuation

                    rawInput = np.hstack( ( gp._getVelocity( previousPose, prevPrevPose, cycleTime ),\
                                            gp._getControls( previousActuation ),\
                                            gp._getControls( prevPrevActuation )\
                                                                                    ) )

                    auxPose = np.array( [ previousPose.x, previousPose.y, previousPose.yaw ] ).reshape( common.NbStates, 1 )
                    auxCtrl = np.array( [ previousActuation.linear.x, previousActuation.angular.z ] ).reshape( common.NbControls, 1 )

                    predictionPose = model._ODE( auxPose, auxCtrl, cycleTime )

                    rawOutput = np.array( [ [ pose.x - predictionPose[0], pose.y - predictionPose[1], common._shortestAngle( predictionPose[2], pose.yaw ) ] ] )

                    print(index)
                    gp._updateRawInData( rawInput )
                    gp._updateRawOutData( rawOutput )

                elif( index >= 1 ):
                    previousPose = pose
                    previousActuation = actuation

                    pose = common.robotPose
                    actuation = common.actuation
                
                else:
                    pose = common.robotPose
                    actuation = common.actuation
                
                index += 1
                r.sleep()
                cycleTime = time.time() - start

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            print( "[trainGP.py] Something went wrong!" )

    print("[trainGP.py] GP training ended.")

    common._pauseFunction( "[trainGP.py] Press [ENTER] to start hyperparameters optimization." )
    print( "[trainGP.py] Optimization starting." )

    input, output = gp._trainingData()

    ###
    os.remove(common.LGP.pathInputTrainingData_np)
    os.remove(common.LGP.pathOutputTrainingData_np) 

    # Creates a new file
    with open(common.LGP.pathInputTrainingData_np, 'w') as input_np:
        pass

    with open(common.LGP.pathOutputTrainingData_np, 'w') as output_np:
        pass

    np.save(common.LGP.pathInputTrainingData_np, input)
    np.save(common.LGP.pathOutputTrainingData_np, output)

    ###
    os.remove(common.LGP.pathInputTrainingData_mat)
    os.remove(common.LGP.pathOutputTrainingData_mat)

    # Creates a new file
    with open(common.LGP.pathInputTrainingData_mat, 'w') as input_mat:
        pass

    with open(common.LGP.pathOutputTrainingData_mat, 'w') as output_mat:
        pass

    matInput = {"input": input, "label": "experiment"}
    matOutput = {"output": output, "label": "experiment"}

    savemat(common.LGP.pathInputTrainingData_mat, matInput)
    savemat(common.LGP.pathOutputTrainingData_mat, matOutput)