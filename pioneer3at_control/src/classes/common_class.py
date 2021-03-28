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

class LGP_Common:

    select = True
    nbTrainingPoints = 50
    nbInputs = 5
    nbOutputs = 3
    maxDataPts = 50                                                     #   Maximum Data Points per Local Model
    limitValue = [ 0.01, 0.01, 0.05 ]
    pred_limitValue = [ 0.5, 0.5, 0.7 ]

class Common:

    """ Nodes Initialization """
    nmpc = 1                                                            #   Node [nmpc] initialization order                                                         
    poseSelector = 2                                                    #   Node [poseSelector] initialization order
    data = 0                                                            #   Node [data] initialization order

    """ System Parameters """
    Ts = 0.13                                                           #   Sampling Time
    N = 10                                                              #   Control Intervals
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
    Q = ca.diag( ca.MX( [ 0.8, 0.8, 0.8 ] ) )                           #   States Matrix
    R = ca.diag( ca.MX( [ 0.1, 0.1 ] ) )                                #   Controls Matrix

    """ Goal Point on [Px] units """
    goalPoint = np.array( [ 340, 115 ] )

    """ Reference Path Points on [Px] units """
    path = [
                [ 194, 80 ],
                [ 124, 106.667 ],
                [ 124.5, 180.0 ],
                [ 125, 250.667 ],
                [ 190, 360.533 ],
                [ 222, 343.467 ],
                [ 247, 283.733 ],
                [ 298, 299 ],
                [ 341, 362 ],
                [ 400, 265 ],
                [ 401, 186 ],
                [ 340, 115 ]
                                    ]
    
    """ Grayscale image from the terrain """
    img = cv.imread( "marsYard/materials/textures/heightmap.jpg", cv.IMREAD_GRAYSCALE )
    
    """ Height-Pixel proportion """
    heightProportion = 2.0/255
    
    """ Parameter parameterization factor for path tracking """
    parameterSpeed = 0.001                                     #   For path tracking
    parameterSpeed_FM = 0.1                                    #   For fast marching

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
    refType = 0

    """
        Cost map for fast marching gradient computation
    """

    costMap = cv.imread( "marsYard/costMap_for_FM/heightmap_FM_1025_1.jpg", cv.IMREAD_GRAYSCALE )
    
    """
        Minimum Distance the robot must be from the goal for the cycle to end
    """
    minDistToGoal = 0.25

    """ Transcription method """
    transMet = 1                    #   0   -> Direct Multiple Shooting (DMS)
                                    #   1   -> Direct Single Shooting (DSS)
    
    """ Optimization type """
    optType = 0                     #   0   -> SQP method
                                    #   1   -> IPOPT method
                                    #   2   -> QRSQP method
                                    #   3   -> QRSQP method + jit
    
    """ Enable/Disable Gaussian Processes """
    gpOnOff = False                                                      #   On (True), Off (False)

    """
        Local Gaussian Processes (LGP)
    """
    LGP = LGP_Common()

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
    def _TwistCallback( self, msg ):

        #   Actuation ( with velocities )
        self.actuation = msg
    
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

    def _costFunction( self, x, xRef, u, statesMatrix, controlsMatrix, option, obstacles = None ):

        A = xRef - x

        A[2] = ca.if_else( A[2] > math.pi, A[2] - 2 * math.pi, ca.if_else( A[2] <= -math.pi, A[2] + 2 * math.pi, A[2] ) )

        # Only weighting tracking and controls
        if( option == 0 ):
            J = ca.mtimes( ca.mtimes( A.T, statesMatrix ), A ) + ca.mtimes( ca.mtimes( u.T, controlsMatrix ), u )

        #   Weighting tracking, controls and found obstacles by a point-repulsive technique
        elif( option == 1 ):

            P = 1 / ( ( x[0] )**2 + ( x[1] )**2 + 0.01 )

            J = ca.mtimes( ca.mtimes( A.T, statesMatrix ), A ) + ca.mtimes( ca.mtimes( u.T, controlsMatrix ), u ) + P

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
        
        if( opt == 1 ):
            return math.sqrt( ( point1.x - point2.x )**2 + ( point1.y - point2.y )**2 )

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