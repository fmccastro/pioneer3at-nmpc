#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import tf, math, os, sys, time

import cv2 as cv
import casadi as ca
import numpy as np

from scipy.spatial.transform import Rotation as R
from pioneer3at_control.msg import pose3D, poseRef
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Int32, Float64, Float64MultiArray

os.chdir("/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_gazebo/models")

class Common:

    """ Nodes Initialization """
    poseSelector = 2                                                    #   Node [poseSelector] initialization order
    nmpc = 1                                                            #   Node [nmpc] initialization order
    markers = 0                                                         #   Node [markers] initialization order
    data = -1                                                           #   Node [data] initialization order
    dataProc = -2                                                       #   Node [dataProc] initialization order

    """ System Parameters """
    Ts = 0.15                                                           #   Sampling Time
    fixedTs = True                                                      #   Variable sampling time
    N = 20                                                              #   Control Intervals
    maxCycles = 10                                                      #   Maximum number of cycles for look ahead point finder
    intAccuracy = 4                                                     #   Integrator Accuracy
    NbStates = 3                                                        #   Number of States
    NbControls = 2                                                      #   Number of Controls
    mapLength = 50.0                                                    #   Square Map dimension
    
    """ Robot's admissible space set """
    X_lb = [ -mapLength/2.0, -mapLength/2.0, -math.pi ]
    X_ub = [ mapLength/2.0, mapLength/2.0, math.pi ]
    
    """ Limits on the Robot's Controls """
    U_lb = [ -0.7, -140 * math.pi / 180 ]                               #   Controls lower bound                           
    U_ub = [  0.7,  140 * math.pi / 180 ]                               #   Controls upper bound

    """ Penalty Matrices """
    Q = ca.diag( ca.SX( [ 0.8, 0.8, 0.8 ] ) )                           #   States Matrix
    R = ca.diag( ca.SX( [ 0.1, 0.1 ] ) )                                #   Controls Matrix
    P = ca.diag( ca.SX( [ 0.8, 0.8 ] ) )                                #   Penalty matrix for controls variation

    """ Cost Function choice """
    costFunction = 1                                                    #   choose cost function

    """ Goal Point on [Px] units """
    goalPoint = np.array( [ 8.13840156, -13.791423 ] )

    """ Reference Path Points on [m] units """
    path = [    [ -6.09161793, -17.20272904],
                [-12.91423002, -14.60360624],
                [-12.86549708,  -7.45614035],
                [-12.81676413,  -0.56851852],
                [ -6.48148148,  10.13966862],
                [ -3.3625731 ,   8.47631579],
                [ -0.92592593,   2.6542885 ],
                [  4.04483431,   4.14230019],
                [  8.23586745,  10.28265107],
                [ 13.98635478,   0.82846004],
                [ 14.08382066,  -6.87134503],
                [  8.13840156, -13.791423  ]    ]
    
    radiusLookAhead = 0.5
    
    """ Grayscale image from the terrain """
    img = cv.imread( "test_field_one/materials/textures/heightmap.jpg", cv.IMREAD_GRAYSCALE )
    
    """ Height-Pixel proportion """
    heightProportion = 2.0/255
    
    """ Parameter parameterization factor for path tracking """
    parameterSpeed = 0.7 * Ts                                   #   For path tracking
    parameterSpeed_FM = 0.7 * Ts                                #   For fast marching

    """
        Reference Pose: 0 -> true pose (from Gazebo) 
                        1 -> fused pose ( IMU + Odometry) from robot_localization package
    """
    poseType = 0

    """
        Tracking Reference Type: 0 -> path
                                 1 -> trajectory
                                 2 -> fast marching path
    """
    refType = 2

    """
        Cost map for fast marching gradient computation
    """
    costMap = cv.imread( "test_field_one/costMap_for_FM/costMap_1025_12.jpg", cv.IMREAD_GRAYSCALE )
    
    """
        Minimum Distance the robot must be from the goal for the cycle to end
    """
    minDistToGoal = 0.25

    """ Transcription method """
    transMet = 1                    #   0   -> Direct Multiple Shooting (DMS)
                                    #   1   -> Direct Single Shooting (DSS)
    
    """ Optimization type """
    optType = 4                     #   0   -> SQP method
                                    #   1   -> IPOPT method
                                    #   2   -> QRSQP method
                                    #   3   -> QRSQP method + jit
                                    #   4   -> SQP method + jit
    
    """ Enable/Disable Gaussian Processes """
    gpOnOff = True                                                      #   On (True), Off (False)

    """ Hyper-parameters optimization """
    hyperOpt = 0                                                        #   0   ->  NLL
                                                                        #   1   ->  MSE

    """ Gaussian Process Model  """
    gpType = 0                                                          #   0   ->  LGP
                                                                        #   1   ->  SOGP

    #   LGP
    if( gpType == 0 ):

        #   Bins features
        angleDiscretization = 40
        velDiscretization = 7

        c_roll = 2
        c_pitch = 2
        c_vel  = 1

        cutVar = 2

        maxDataPts = 4

    #   SOGP
    elif( gpType == 1 ):
        v = 0.1
        R_max = 20
        maxSize = 10

    """ Prediction variables set """
    gpModel = 1                                                         #   1   ->  u_{k}, u_{k-1}, v_{k-1}

    """ Number of training points """
    nbTrainingPoints = 1000
    trainingSet = 10                                                    #   Set of training points to be taken into account

    """ Prediction variables input dimension """
    nbInputs = 6
    nbOutputs = 3

    pathInputTrainingData_np = "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/trainingData/trainingField1/gp_model_" + str(gpModel) + "/" + str(nbTrainingPoints) + "_training_points/input.npy"
    pathOutputTrainingData_np = "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/trainingData/trainingField1/gp_model_" + str(gpModel) + "/" + str(nbTrainingPoints) + "_training_points/output.npy"

    pathInputTrainingData_mat = "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/trainingData/trainingField1/gp_model_" + str(gpModel) + "/" + str(nbTrainingPoints) + "_training_points/input.mat"
    pathOutputTrainingData_mat = "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/trainingData/trainingField1/gp_model_" + str(gpModel) + "/" + str(nbTrainingPoints) + "_training_points/output.mat"

    pathKernel = "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/trainingData/trainingField1/gp_model_" + str(gpModel) + "/" + str(nbTrainingPoints) + "_training_points/kernel.pickle"
    pathLocalModels = "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/trainingData/trainingField1/gp_model_" + str(gpModel) + "/" + str(nbTrainingPoints) + "_training_points/localModels.pickle"

    """
        Terramechanics parameters
    """
    terraOnOff = False                                                  #   Terramechanics forces simulation On (True) or Off (False)
    n = 1.1                                                             #   Sinkage exponent
    c = 1.0 * math.pow( 10, 3 )                                         #   Soil shear cohesion constant
    phi = 30.0 * math.pi / 180                                          #   Angle of internal friction
    k_c = 0.9 * math.pow( 10, 3 )                                       #   k_c, k_{phi} are soil cohesion frictional constants
    k_phi = 1523.4 * math.pow( 10, 3 )                                                  
    k = 0.025                                                           #   shear deformation modulus
    
    """
        Robot features
    """
    r = 0.222/2                                                         #   wheel radius
    b = 0.086                                                           #   wheel width
    nbWheels = 4                                                        #   number of wheels
    m = 22.61 + 1.2 * 4                                                 #   robot mass

    def __init__( self ):

        self.truePose = LinkStates()
        self.fusedPose = Odometry()
        self.robotPose = pose3D()

        self.actuation = Twist()

        self.step = Int32()
        self.initCounter = Int32()

        self.clock = Float64()
        self.cycleTime = Float64()

        self.pathSequence = Float64MultiArray()
        self.refSequence = Float64MultiArray()

        self.poseRef = poseRef()

    def _poseCallback( self, msg, poseType ):

        #   Type gazebo_msgs/LinkStates.msg, from Gazebo     
        if( poseType == 0 ):
            self.truePose = msg

        #   Type nav_msgs/Odometry.msg, fused Pose (IMU + Odometry) 
        elif( poseType == 1 ):
            self.fusedPose = msg
        
        #   Type pioneer3at_control/pose3D.msg, robot pose expressed on aircraft angles (roll, pitch, yaw)
        elif( poseType == 2 ):
            self.robotPose = msg

    #   Type geometry_msgs/Twist.msg
    def _TwistCallback( self, msg, opt ):

        #   Actuation ( with velocities )
        if( opt == 0 ):
            self.actuation = msg

        #   Velocity ( with velocities )
        if( opt == 1 ):
            self.velocity = msg

    #   Type std_msgs/Int32.msg
    def _Int32Callback( self, msg, opt ):

        #   Number of cycles counter
        if( opt == 0 ):
            self.step = msg

    #   Type std_msgs/Float64.msg
    def _Float64Callback( self, msg, opt ):

        #   Simulator clock counter
        if( opt == 0 ):
            self.clock = msg

        #   Cycle time counter
        elif( opt == 1 ):
            self.cycleTime = msg
        
        elif( opt == 2 ):
            self.optTime = msg

        elif( opt == 3 ):
            self.distance = msg

    #   Type std_msgs/Float64MultiArray.msg
    def _Float64MultiArrayCallback( self, msg, opt ):

        if( opt == 0 ):
            self.pathSequence = msg
        
        elif( opt == 1 ):
            self.refSequence = msg
        
        elif( opt == 2 ):
            self.error = msg
    
    #   Type pioneer3at_control/poseRef.msg, pose & reference
    def _poseRefCallback( self, msg ):

        self.poseRef = msg

    def _pauseFunction( self, string ):

        print( string, " Press [ENTER] to carry on." )
        sys.stdin.read(1)

        print("GO.")

        time.sleep(2)
    
    def _quaternionToRPY( self, quaternion ):

        r = R.from_quat( [ quaternion.x, quaternion.y, quaternion.z, quaternion.w ] )

        return r.as_rotvec()
    
    def _selectPoseType( self, poseType ):

        pose = pose3D()

        if( poseType == 0 ):

            pose.x = self.truePose.pose[1].position.x
            pose.y = self.truePose.pose[1].position.y
            pose.z = self.truePose.pose[1].position.z

            rpy = self._quaternionToRPY( self.truePose.pose[1].orientation )

            pose.roll = rpy[0]
            pose.pitch = rpy[1]
            pose.yaw = rpy[2]

        elif( poseType == 1 ):

            pose.x = self.fusedPose.pose.pose.position.x
            pose.y = self.fusedPose.pose.pose.position.y
            pose.z = self.fusedPose.pose.pose.position.z

            rpy = self._quaternionToRPY( self.fusedPose.pose.pose.orientation )

            pose.roll = rpy[0]
            pose.pitch = rpy[1]
            pose.yaw = rpy[2]
        
        return pose
    
    def _selectVelType( self, poseType ):

        vel = Twist()

        if( poseType == 0 ):
            vel = self.truePose.twist[1]
        
        elif( poseType == 1 ):
            vel = self.fusedPose.twist.twist

        return vel

    def _ode( self, x, u, gp = None ):

        if( gp == None ):
            
            dxdt = [
                        ca.cos( x[2] ) * u[0],
                        ca.sin( x[2] ) * u[0],
                        u[1]
                    ]
        
        else:

            dxdt = [
                        ca.cos( x[2] ) * u[0] + gp[0],
                        ca.sin( x[2] ) * u[0] + gp[1],
                        u[1] + gp[2]
                    ]

        return ca.vertcat(*dxdt)

    def _costFunction( self, x, xRef, u, prevU, statesMatrix, controlsMatrix, deltaCtrlMatrix, option ):

        A = xRef - x
        deltaU = u - prevU

        A[2] = ca.if_else( A[2] > math.pi, A[2] - 2 * math.pi, ca.if_else( A[2] <= -math.pi, A[2] + 2 * math.pi, A[2] ) )

        # Only weighting tracking and controls
        if( option == 0 ):
            J = ca.mtimes( ca.mtimes( A.T, statesMatrix ), A ) + ca.mtimes( ca.mtimes( u.T, controlsMatrix ), u )

        #   Weighting tracking, controls and controls increment
        elif( option == 1 ):
            J = ca.mtimes( ca.mtimes( A.T, statesMatrix ), A ) + ca.mtimes( ca.mtimes( u.T, controlsMatrix ), u ) + ca.mtimes( ca.mtimes( deltaU.T, deltaCtrlMatrix ), deltaU )
        
        elif( option == 2 ):
            J = ca.mtimes( ca.mtimes( A.T, statesMatrix ), A ) + ca.mtimes( ca.mtimes( deltaU.T, deltaCtrlMatrix ), deltaU )
            
        return J

    def _distanceBtwPoints( self, point1, point2, opt ):

        """

            #   Inputs:
                    point1: receives the robot's pose
                    point2: it receives the robot's goal point
                    poseType: it sets point1 message type

            #   Output:
                    Distance between two points

        """

        if( opt == 0 ):
            return math.sqrt( ( point1.x - point2[0] )**2 + ( point1.y - point2[1] )**2 )
        
        elif( opt == 1 ):
            return math.sqrt( ( point1.x - point2.x )**2 + ( point1.y - point2.y )**2 )

        elif( opt == 2 ):
            return math.sqrt( math.pow( point1[0] - point2[0], 2 ) + math.pow( point1[1] - point2[1], 2 ) )
        
        elif( opt == 3 ):
            return math.sqrt( math.pow( point1[0] - point2.x, 2 ) + math.pow( point1[1] - point2.y, 2 ) )

    def _shortestAngle( self, angle1, angle2 ):

        """
            inputs:
                    angle1: scalar in radians
                    angle2: scalar in radians
            
            output:
                    diff: scalar in radians
        """

        diff = angle2 - angle1

        if( diff > math.pi ):
        
            diff = diff - 2 * math.pi
        
        elif( diff <= - math.pi ):

            diff = diff + 2 * math.pi

        return diff

    def _kernel( self, lengthscales, variance, xp, xq ):

        W = np.linalg.matrix_power( np.diag(lengthscales), -2 )

        res = variance * math.exp( -0.5 * ( ( xp - xq ) @ W ) @ ( xp - xq ).T )

        return res