#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, tf, time, os

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

from multiprocessing import Process, Queue
from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from std_msgs.msg import Int32, Float64, Float64MultiArray
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from classes import Model, Planner, Common, LGP, SOGP

from pioneer3at_control.msg import pose3D, poseRef
from pioneer3at_control.srv import getPath

if __name__ == '__main__':

    common = Common()

    ###     With gaussian processes (GP)
    if( common.gpOnOff ):

        ###     Local Gaussian Processes (LGP)
        if( common.gpType == 0 ):
            gp = LGP( common )

            rawInput = np.load( common.pathInputTrainingData_np )
            rawOutput = np.load( common.pathOutputTrainingData_np )

            gp._saveTrainingData( rawInput[ :, common.cutVar: ], rawOutput )
            gp._loadPickle( common.pathKernel, common.pathLocalModels, 0 )
            gp._loadPickle( common.pathKernel, common.pathLocalModels, 1 )

        ###     Sparse Online Gaussian Processes (SOGP)
        elif( common.gpType == 1 ):
            gp = SOGP( common )

        model = Model( common = common, gp = gp )

    ###     No GP
    else:
        model = Model( common = common )

    pathPlanning = Planner( common )

    rospy.init_node( 'nmpc', anonymous = True )

    rospy.Subscriber( '/pioneer3at/robotPose', pose3D, common._poseCallback, 2 )                #   '/pioneer3at/robotPose' -> topic for pose
    rospy.Subscriber( '/pioneer3at/robotVel', Twist, common._TwistCallback, 1 )                 #   '/pioneer3at/robotVel' -> topic for robot velocity

    nextCommand = rospy.Publisher( '/pioneer3at/cmd_vel', Twist, queue_size = 1 )               #   '/pioneer3at/cmd_vel' -> topic for pose
    stepPub = rospy.Publisher( '/pioneer3at/currentStep', Int32, queue_size = 1 )               #   '/pioneer3at/currentStep' -> topic for cycle step
    horPub = rospy.Publisher( '/pioneer3at/currentSol', Float64MultiArray, queue_size = 1 )     #   '/pioneer3at/currentSol' -> topic for states horizon
    refPub = rospy.Publisher( '/pioneer3at/currentRef', Float64MultiArray, queue_size = 1 )     #   '/pioneer3at/currentRef' -> topic for current reference getting into the cost function
    cycleTimePub = rospy.Publisher( '/pioneer3at/cycleTime', Float64, queue_size = 1 )          #   '/pioneer3at/cycleTime' -> topic for cycle iteration timing count
    clockPub = rospy.Publisher( '/pioneer3at/clock', Float64, queue_size = 1 )                  #   '/pioneer3at/clock' -> topic for simulation time counting ( since the robot starts moving )
    optTimePub = rospy.Publisher( '/pioneer3at/optTime', Float64, queue_size = 1 )              #   '/pioneer3at/optTime' -> topic for optimization time
    pRefPub = rospy.Publisher( '/pioneer3at/pRef', poseRef, queue_size = 1 )                    #   '/pioneer3at/error' -> topic for error elements to publish 

    while( rospy.get_param("/init") > common.nmpc ):
        continue
    
    common._pauseFunction( "[nmpc.py] Press [ENTER] to activate node." )
    print( "[nmpc.py] It's active." )
    
    k = Int32(0)                                                                        #   Number of optimizations counter
    cycleTime = Float64(0)                                                              #   Loop time 
    actuation = Twist()                                                                 #   Robot actuation
    horizon = Float64MultiArray()                                                       #   Sequences of states determined in the optimization
    reference = Float64MultiArray()                                                     #   Sequences of states to act as reference on a given iteration
    pRef = poseRef()

    if( common.refType == 2 ):
        pose = common.robotPose

        grad_X, grad_Y = pathPlanning._fastMarching()

        pathPlanning._showMap_FM( grad_X, grad_Y, pose.x, pose.y )

    elif( common.refType == 1 ):
        refPts, ref = pathPlanning._pathSlicing()
        #refPts = np.hstack( ( refPts, pathPlanning._pathOrientation( refPts ) ) )

        pathPlanning._showPath( refPts )

    else:
        ref = pathPlanning._pathSlicing()
        ref = np.hstack( ( ref, pathPlanning._pathOrientation( ref ) ) )

        pathPlanning._showPath( ref )
    
    common._pauseFunction("[nmpc.py] About to start moving.")

    pose = common.robotPose
    velocity = common.velocity

    ### Beginning of first optimization ###########################################################################################################################

    if( common.gpOnOff ):
        out = gp._fullPrediction( pose, k.data )

    #   Path or Trajectory tracking
    if( common.refType == 0 ):
        refList = ref.tolist()
        refList = sum( refList, [] )

        #   Direct Multiple Shooting
        if( common.transMet == 0 ):
            reference.data = refList[ k.data * common.NbStates: ( k.data + common.N ) * common.NbStates ]

            if( common.gpOnOff ):
                solution, optTime = model._solveDMS( [0] * common.N * ( common.NbStates + common.NbControls ),\
                                                ( common.U_lb + common.X_lb ) * common.N, ( common.U_ub + common.X_ub ) * common.N,\
                                                [0] * common.N * common.NbStates, [0] * common.N * common.NbStates,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data +\
                                                        [0] * common.N * common.nbOutputs, k.data )

            else:
                solution, optTime = model._solveDMS( [0] * common.N * ( common.NbStates + common.NbControls ),\
                                                ( common.U_lb + common.X_lb ) * common.N, ( common.U_ub + common.X_ub ) * common.N,\
                                                [0] * common.N * common.NbStates, [0] * common.N * common.NbStates,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data, k.data )

        #   Direct Single Shooting
        elif( common.transMet == 1 ):
            reference.data = refList[ k.data * common.NbStates: ( k.data + common.N ) * common.NbStates ]

            if( common.gpOnOff ):
                solution, optTime = model._solveDSS( [0] * common.N * ( common.NbControls ),\
                                                common.U_lb * common.N, common.U_ub * common.N,\
                                                common.X_lb * common.N, common.X_ub * common.N,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data +\
                                                        [0] * common.N * common.nbOutputs, k.data )

            else:
                solution, optTime = model._solveDSS( [0] * common.N * ( common.NbControls ),\
                                                common.U_lb * common.N, common.U_ub * common.N,\
                                                common.X_lb * common.N, common.X_ub * common.N,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data, k.data )

    elif( common.refType == 1 ):
        refList = refPts.tolist()
        refList = sum( refList, [] )

        #   Direct Multiple Shooting
        if( common.transMet == 0 ):
            reference.data = refList[ k.data * common.NbStates: ( k.data + common.N ) * common.NbStates ]

            if( common.gpOnOff ):
                solution, optTime = model._solveDMS( [0] * common.N * ( common.NbStates + common.NbControls ),\
                                                ( common.U_lb + common.X_lb ) * common.N, ( common.U_ub + common.X_ub ) * common.N,\
                                                [0] * common.N * common.NbStates, [0] * common.N * common.NbStates,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data +\
                                                        [0] * common.N * common.nbOutputs, k.data )
            
            else:
                solution, optTime = model._solveDMS( [0] * common.N * ( common.NbStates + common.NbControls ),\
                                                ( common.U_lb + common.X_lb ) * common.N, ( common.U_ub + common.X_ub ) * common.N,\
                                                [0] * common.N * common.NbStates, [0] * common.N * common.NbStates,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data, k.data )

        #   Direct Single Shooting
        elif( common.transMet == 1 ):
            reference.data = refList[ k.data * common.NbStates: ( k.data + common.N ) * common.NbStates ]

            if( common.gpOnOff ):
                solution, optTime = model._solveDSS( [0] * common.N * ( common.NbControls ),\
                                                common.U_lb * common.N, common.U_ub * common.N,\
                                                common.X_lb * common.N, common.X_ub * common.N,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data +\
                                                        [0] * common.N * common.nbOutputs, k.data )
            
            else:
                solution, optTime = model._solveDSS( [0] * common.N * ( common.NbControls ),\
                                            common.U_lb * common.N, common.U_ub * common.N,\
                                            common.X_lb * common.N, common.X_ub * common.N,\
                                                    [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data, k.data )

    #   Path Planning with Fast Marching
    elif( common.refType == 2 ):
        ref = pathPlanning._computeFM_Path( grad_X, grad_Y, pose.x, pose.y, common.N )

        ref = ref.tolist()
        ref = sum( ref, [] )

        reference.data = ref

        #   Direct Multiple Shooting
        if( common.transMet == 0 ):

            if( common.gpOnOff ):
                solution, optTime = model._solveDMS( [0] * common.N * ( common.NbStates + common.NbControls ), 
                                                ( common.U_lb + common.X_lb ) * common.N, ( common.U_ub + common.X_ub ) * common.N,\
                                                [0] * common.N * common.NbStates, [0] * common.N * common.NbStates,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + ref +\
                                                        [0] * common.N * common.nbOutputs, k.data )
            
            else:
                solution, optTime = model._solveDMS( [0] * common.N * ( common.NbStates + common.NbControls ), 
                                                ( common.U_lb + common.X_lb ) * common.N, ( common.U_ub + common.X_ub ) * common.N,\
                                                [0] * common.N * common.NbStates, [0] * common.N * common.NbStates,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + ref, k.data )

        #   Direct Single Shooting
        elif( common.transMet == 1 ):

            if( common.gpOnOff ):
                solution, optTime = model._solveDSS( [0] * common.N * common.NbControls,\
                                                common.U_lb * common.N, common.U_ub * common.N,\
                                                common.X_lb * common.N, common.X_ub * common.N,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + ref +\
                                                        [0] * common.N * common.nbOutputs, k.data )
            
            else:
                solution, optTime = model._solveDSS( [0] * common.N * common.NbControls,\
                                                common.U_lb * common.N, common.U_ub * common.N,\
                                                common.X_lb * common.N, common.X_ub * common.N,\
                                                        [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + ref, k.data )

    #   solutionX -> optimized variables solution
    solutionX = solution['x'].elements()

    #   solutionG -> inequality (DSS) or continuity (DMS) constraint horizon sequence
    solutionG = solution['g'].elements()

    ### End of first optimization ####################################################################################################################################

    """ Publish states sequences from optimization for visualization on RViz """
    
    #   Direct Multiple Shooting
    if( common.transMet == 0 ):
        horizon.data = [ solutionX[ i + common.NbControls : i + common.NbStates + common.NbControls ] for i in range( 0, len( solutionX ), common.NbStates + common.NbControls ) ]
        horizon.data = sum( horizon.data, [] )

        if( common.gpOnOff ):
            controls = [ solutionX[ i : i + common.NbControls ] for i in range( 0, len( solutionX ), common.NbStates + common.NbControls ) ]
            controls = sum( controls, [] )

    #   Direct Single Shooting
    elif( common.transMet == 1 ):
        horizon.data = solutionG

        if( common.gpOnOff ):
            controls = solutionX

    ######

    actuation = model._selectCommand( solutionX )
    
    stepPub.publish( k )                    #   Number of optimizations counter // Number of optimizations = k + 1
    horPub.publish( horizon )               #   Publish optimized variables sequence
    refPub.publish( reference )             #   Publish current reference which gets into the NMPC
    nextCommand.publish( actuation )        #   Publish actuation to be applied to the robot
    clockPub.publish( 0 )                   #   Start clock simulation counter after first actuation
    optTimePub.publish( optTime )           #   Save time spent on each optimization

    start = time.time()

    #   Set "/init" parameter to zero to anounce simulation started ( if "/init" = 0 it means the simulation started )
    rospy.set_param( "/init", rospy.get_param("/init") - 1 )

    lastLookAheadPoint = 0
    lastFracIndex = 0
    #factor = 0

    flag = True

    while( not rospy.is_shutdown() and common._distanceBtwPoints( pose, common.goalPoint, 0 ) > common.minDistToGoal ):
        try:

            cycleStart = time.time()

            k.data += 1

            prevPose = pose
            pose = common.robotPose

            preVelocity = velocity
            velocity = common.velocity

            #   Publish pose and respective reference for node [data.py] to compute error
            pRef.reference = reference.data[ :common.NbStates ]
            pRef.pose = pose

            pRefPub.publish( pRef )

            if( flag ):
                #   Decrease "/init" parameter to enable [data.py] node working mode
                rospy.set_param( "/init", rospy.get_param("/init") - 1 )
                flag = False

            ### Optimization beginning ###############################################################################################################################

            a = time.time()

            if( common.gpOnOff ):
                predictionInputs = gp._getPredictionInputs( prevPose, horizon.data, controls, common.gpModel )
                print(predictionInputs)
                out = gp._fullPrediction( pose, k.data, predictionInputs )

            print( time.time() - a )

            #   Path-tracking
            if( common.refType == 0 ):
                lastLookAheadPoint, lastFracIndex = pathPlanning._lookAheadPoint( ref, lastLookAheadPoint, lastFracIndex, pose, common.radiusLookAhead, common.maxCycles )

                #   Direct Multiple Shooting
                if( common.transMet == 0 ):
                    initialGuess = ca.vertcat( solutionX[ common.NbStates + common.NbControls: ], solutionX[ -( common.NbStates + common.NbControls ): ] )

                    #   In case the the reference sequence surpasses the array length, add more poses to the reference equal to the last one
                    if( ref.shape[0] - lastLookAheadPoint < common.N ):
                        addPoints = ref.shape[0] - lastLookAheadPoint
                        reference.data = refList[ lastLookAheadPoint * common.NbStates: ( lastLookAheadPoint + addPoints ) * common.NbStates ]\
                                                    + refList[ -common.NbStates:  ] * ( common.N - addPoints )

                    else:
                        reference.data = refList[ lastLookAheadPoint * common.NbStates: ( lastLookAheadPoint + common.N ) * common.NbStates ]
                    
                    solution, optTime = model._solveDMS( initialGuess,\
                                                    ( common.U_lb + common.X_lb ) * common.N, ( common.U_ub + common.X_ub ) * common.N,\
                                                    [0] * common.N * common.NbStates, [0] * common.N * common.NbStates,\
                                                            [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data, k.data )

                #   Direct Single Shooting
                elif( common.transMet == 1 ):
                    initialGuess = ca.vertcat( solutionX[ common.NbControls: ], solutionX[ -( common.NbControls ): ] )

                    #   In case the the reference sequence surpasses the array length, add more poses to the reference equal to the last one
                    if( ref.shape[0] - lastLookAheadPoint < common.N ):
                        addPoints = ref.shape[0] - lastLookAheadPoint
                        reference.data = refList[ lastLookAheadPoint * common.NbStates: ( lastLookAheadPoint + addPoints ) * common.NbStates ]\
                                                    + refList[ -common.NbStates:  ] * ( common.N - addPoints )
                    
                    else:
                        reference.data = refList[ lastLookAheadPoint * common.NbStates: ( lastLookAheadPoint + common.N ) * common.NbStates ]

                    solution, optTime = model._solveDSS( initialGuess,\
                                                    common.U_lb * common.N, common.U_ub * common.N,\
                                                    common.X_lb * common.N, common.X_ub * common.N,\
                                                            [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data, k.data )

            #   Trajectory tracking
            elif( common.refType == 1 ):
                timeNow = time.time() - start

                refList = [ ref.eval( timeNow + i * cycleTime.data ) for i in range( common.N ) ]
                refList = sum( refList, [] )
                reference.data = refList

                #   Direct Multiple Shooting
                if( common.transMet == 0 ):
                    initialGuess = ca.vertcat( solutionX[ common.NbStates + common.NbControls: ], solutionX[ -(common.NbStates + common.NbControls): ] )

                    solution, optTime = model._solveDMS( initialGuess,\
                                                    ( common.U_lb + common.X_lb ) * common.N, ( common.U_ub + common.X_ub ) * common.N,\
                                                    [0] * common.N * common.NbStates, [0] * common.N * common.NbStates,\
                                                            [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data, k.data )

                #   Direct Single Shooting
                elif( common.transMet == 1 ):
                    initialGuess = ca.vertcat( solutionX[ common.NbControls: ], solutionX[ -( common.NbControls ): ] )

                    solution, optTime = model._solveDSS( initialGuess,\
                                                    common.U_lb * common.N, common.U_ub * common.N,\
                                                    common.X_lb * common.N, common.X_ub * common.N,\
                                                            [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + reference.data, k.data )

            #   Path tracking with Fast Marching
            elif( common.refType == 2 ):
                ref = pathPlanning._computeFM_Path( grad_X, grad_Y, pose.x, pose.y, common.N )

                ref = ref.tolist()
                ref = sum( ref, [] )

                reference.data = ref

                #   Direct Multiple Shooting
                if( common.transMet == 0 ):
                    initialGuess = ca.vertcat( solutionX[ common.NbStates + common.NbControls: ], solutionX[ -(common.NbStates + common.NbControls): ] )

                    
                    solution, optTime = model._solveDMS( initialGuess,\
                                                ( common.U_lb + common.X_lb ) * common.N, ( common.U_ub + common.X_ub ) * common.N,\
                                                [0] * common.N * common.NbStates, [0] * common.N * common.NbStates,\
                                                    [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + ref, k.data )
                
                #   Direct Single Shooting
                elif( common.transMet == 1 ):
                    initialGuess = ca.vertcat( solutionX[ common.NbControls: ], solutionX[ -( common.NbControls ): ] )

                    solution, optTime = model._solveDSS( initialGuess,\
                                                ( common.U_lb ) * common.N, ( common.U_ub ) * common.N,\
                                                common.X_lb * common.N, common.X_ub * common.N,\
                                                    [ pose.x, pose.y, pose.yaw ] + [0] * common.NbControls + ref, k.data )

            #   solutionX -> optimized variables solution
            solutionX = solution['x'].elements()

            #   solutionG -> inequality (DSS) or continuity (DMS) constraint horizon sequence
            solutionG = solution['g'].elements()

            ### End of optimization ######################################################################################################################################

            """ Publish states sequences from optimization for visualization on RViz """
            
            #   Direct Multiple Shooting
            if( common.transMet == 0 ):
                horizon.data = [ solutionX[ i + common.NbControls : i + common.NbStates + common.NbControls ] for i in range( 0, len( solutionX ), common.NbStates + common.NbControls ) ]
                horizon.data = sum( horizon.data, [] )

                if( common.gpOnOff ):
                    controls = [ solutionX[ i : i + common.NbControls ] for i in range( 0, len( solutionX ), common.NbStates + common.NbControls ) ]
                    controls = sum( controls, [] )

            #   Direct Single Shooting
            elif( common.transMet == 1 ):
                horizon.data = solutionG

                if( common.gpOnOff ):
                    controls = solutionX

            ######

            actuation = model._selectCommand( solutionX )

            #   Retrieving the loop duration from its beginning till the end
            cycleEnd = time.time()
            cycleTime.data = time.time() - cycleStart                                    

            #   Checking the clock timing since the beginning of simulation
            simClock = cycleEnd - start

            print( cycleTime.data )
            
            stepPub.publish( k )                                        #   Number of optimizations counter // Number of optimizations = k + 1 
            horPub.publish( horizon )                                   #   Publish optimized variables sequence
            refPub.publish( reference )                                 #   Publish current reference
            nextCommand.publish( actuation )                            #   Publish actuation to be applied to the robot
            clockPub.publish( simClock )                                #   Start clock simulation counter after first actuation
            cycleTimePub.publish( cycleTime )                           #   Publish loop time from beginning till the end
            optTimePub.publish( optTime )                               #   Publish time spent on each optimization

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            print( "[nmpc.py] Something went wrong!" )

    actuation.linear.x = 0
    actuation.angular.z = 0

    nextCommand.publish(actuation)

    print("[nmpc.py] Simulation ended.")

    nodes = os.popen("rosnode list").readlines()

    for i in range( len(nodes) ):
        nodes[i] = nodes[i].replace( "\n", "" )

    for node in nodes:
        os.system( "rosnode kill " + node )