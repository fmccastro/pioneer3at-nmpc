#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy
import math

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import tensorflow as tf

from geometry_msgs.msg import Twist

class Model:
    def __init__( self, common, gp = None ):
        
        """

            #   Arguments:
                    Nx:                 Number of states
                    Nu:                 Number of controls
                    ode:                ode( x, u )
                    J:                  Cost Function
                    controlIntervals:   Number of Control Intervals
                    weightMatrix_1:     First weight matrix
                    weightMatrix_2:     Second weight matrix
                    lowerBounds:        Constraints Lower Bounds
                    upperBounds:        Constraints Upper Bounds 
                    samplingTime:       Sampling Time

        """
        
        self.__Nx = common.NbStates                
        self.__Nu = common.NbControls
        self.__dt = common.Ts
        self.__N = common.N
        self.__M = common.intAccuracy
        self.__x_lb = common.X_lb
        self.__x_ub = common.X_ub
        self.__u_lb = common.U_lb                      
        self.__u_ub = common.U_ub
        self.__ode = common._ode
        self.__transMethod = common.transMet
        self.__opt = common.optType
        self.__J = common._costFunction
        self.__weightMatrix_1 = common.Q
        self.__weightMatrix_2 = common.R

        self.__gp = gp

        states = ca.SX.sym( 'states', self.__Nx )
        controls = ca.SX.sym( 'controls', self.__Nu )
        deltaTime = ca.SX.sym( 'deltaTime', 1 )
        statesRef = ca.SX.sym( 'statesRef', self.__Nx )
        
        """ Discrete Time Dynamics Formulation """
        DT = self.__dt / self.__M

        X0 = ca.SX.sym( 'X0', self.__Nx )
        X_Ref = ca.SX.sym( 'X0_Ref', self.__Nx )
        U = ca.SX.sym( 'U', self.__Nu )

        if( self.__opt == 0 ):
            self.__optOptions = { 'qpsol': 'qpoases', 'qpsol_options': { 'printLevel': 'none' }, 'expand': True }
            self.__optSolver = 'sqpmethod'
    
        elif( self.__opt == 1 ):
            self.__optOptions = { 'verbose': False, 'verbose_init': False, 'print_time': True }
            self.__optSolver = 'ipopt'

        elif( self.__opt == 2 ):
            self.__optOptions = { 'qpsol': 'qpoases', 'qpsol_options': { 'printLevel': 'none' }, 'expand': True }
            self.__optSolver = 'qrsqp'
        
        elif( self.__opt == 3 ):
            self.__optOptions = {

                    'qpsol': 'qpoases',

                    #'max_iter': 2,

                    'print_iteration': False,

                    'print_header': False,

                    'qpsol_options': { "jit": True,

                            'jit_options': {
                            
                            'compiler': 'ccache gcc',

                            'flags': ["-O3", "-pipe"] },

                            'compiler': 'shell',

                            'print_time': True,

                            'printLevel': 'none'
                    },

                    'expand': True
                }

            self.__optSolver = 'qrsqp'
        
        elif( self.__opt == 4 ):
            self.__optOptions = {

                    'qpsol': 'qpoases',

                    #'max_iter': 2,

                    'print_iteration': False,

                    'print_header': False,

                    'qpsol_options': { "jit": True,

                            'jit_options': {
                            
                            'compiler': 'ccache gcc',

                            'flags': ["-O3", "-pipe"] },

                            'compiler': 'shell',

                            'print_time': True,

                            'printLevel': 'none'
                    },

                    'expand': True
                }

            self.__optSolver = 'sqpmethod'

        if( self.__gp != None ):

            self.__paramGP_def = common.LGP.paramGP_def
            self.__predGP_def = common.LGP.predGP_def
            self.__paramGP = common.LGP.paramGP
            self.__predGP = common.LGP.predGP

            self.__nbLocalModels = common.LGP.maxLocalModels
            self.__nbLGPOutput = common.LGP.nbOutputs

            indexLocalModels_def  = ca.SX.sym( 'indexLocalModels_def', 1, self.__nbLocalModels * self.__nbLGPOutput )
            indexLocalModels  = ca.SX.sym( 'indexLocalModels', 1, self.__nbLocalModels * self.__nbLGPOutput )

            #############################################################################################################################
            #############################################################################################################################

            X = X0
            Q = 0

            ###     With gaussian processes     ######
            f_gpOn = ca.Function( 'f_gpOn', [ states, statesRef, controls, self.__predGP_def, indexLocalModels_def ],\
                                            [ self.__ode( states, controls ) + self.__gp._fullPrediction( self.__predGP_def, indexLocalModels_def ),\
                                                self.__J( states, statesRef, controls, self.__weightMatrix_1, self.__weightMatrix_2, 0 ) ] )

            for j in range( self.__M ):
                k1, k1_q = f_gpOn( X,             X_Ref, U, self.__predGP, indexLocalModels )
                k2, k2_q = f_gpOn( X + DT/2 * k1, X_Ref, U, self.__predGP, indexLocalModels )
                k3, k3_q = f_gpOn( X + DT/2 * k2, X_Ref, U, self.__predGP, indexLocalModels )
                k4, k4_q = f_gpOn( X + DT * k3,   X_Ref, U, self.__predGP, indexLocalModels )
                X = X + DT / 6 * ( k1 + 2 * k2 + 2 * k3 + k4 )
                Q = Q + DT / 6 * ( k1_q + 2 * k2_q + 2 * k3_q + k4_q )
            self.__F_gpOn = ca.Function( 'F_gpOn', [ X0, X_Ref, U, self.__predGP, indexLocalModels ], [ X, Q ] )
            ######

            #############################################################################################################################
            #############################################################################################################################

            """
                NMPC Direct Multiple Shooting Formulation
            """

            #   Start with an empty NLP
            w = []
            G = []

            _costFunction = 0
            
            """ Parameters """
            _initPose = ca.SX.sym( '_initPose', self.__Nx )
            _reference = ca.SX.sym( '_reference', self.__N * self.__Nx )
            _listLocalModels = ca.SX.zeros( 1, self.__N * self.__nbLocalModels * self.__nbLGPOutput )
            
            param_s = ca.vertcat( _initPose, _reference, self.__paramGP, _listLocalModels )

            for k in range( self.__N ):
                
                #   New NLP variable for the control
                if( k > 0 ):
                    prevUk = Uk
                
                Uk = ca.SX.sym( 'U_' + str(k), self.__Nu )
                w += [ Uk ]

                currentRef = ca.vertcat( _reference[ k * self.__Nx ], _reference[ k * self.__Nx + 1 ], _reference[ k * self.__Nx + 2 ] )

                if( k == 0 ):
                    predInput = ca.vertcat( self.__paramGP[:4], Uk )
                    Fk = self.__F_gpOn( _initPose, currentRef, Uk, predInput,\
                                            _listLocalModels[ k * ( self.__nbLocalModels * self.__nbLGPOutput ) :\
                                                k * ( self.__nbLocalModels * self.__nbLGPOutput ) + ( self.__nbLocalModels * self.__nbLGPOutput ) ] )

                elif( k == 1 ):
                    predInput = ca.vertcat( self.__paramGP[4:], Uk, prevUk )
                    Fk = self.__F_gpOn( Xk, currentRef, Uk, predInput,\
                                            _listLocalModels[ k * ( self.__nbLocalModels * self.__nbLGPOutput ) :\
                                                k * ( self.__nbLocalModels * self.__nbLGPOutput ) + ( self.__nbLocalModels * self.__nbLGPOutput ) ] )

                else:
                    yawDiff = Xk[2] - prevXk[2]
                    yawDiff = ca.if_else( yawDiff > math.pi, yawDiff - 2 * math.pi, ca.if_else( yawDiff <= -math.pi, yawDiff + 2 * math.pi, yawDiff ) )
                    preVel = ca.vertcat( ca.sqrt( ( Xk[0] - prevXk[0] )**2 + ( Xk[1] - prevXk[1] )**2 ) / self.__dt, yawDiff / self.__dt )

                    predInput = ca.vertcat( preVel, Uk, prevUk )
                    Fk = self.__F_gpOn( Xk, currentRef, Uk, predInput,\
                                            _listLocalModels[ k * ( self.__nbLocalModels * self.__nbLGPOutput ) :\
                                                k * ( self.__nbLocalModels * self.__nbLGPOutput ) + ( self.__nbLocalModels * self.__nbLGPOutput ) ] )

                Xk_end = Fk[0]
                _costFunction += Fk[1]

                #   New NLP variable for state at end of interval
                if( k > 0 ):
                    prevXk = Xk

                Xk = ca.SX.sym( 'X_' + str(k + 1), self.__Nx )
                w += [ Xk ]
                
                #   Continuity constraint
                G += [ Xk_end - Xk ]
            
            #   Create NLP solver
            prob = { 'f': _costFunction, 'x': ca.vertcat(*w), 'g': ca.vertcat(*G), 'p': param_s }
            self.__solverDMS = ca.nlpsol( 'solver', self.__optSolver, prob, self.__optOptions )

            #############################################################################################################################
            #############################################################################################################################
        
            """
                NMPC Direct Single Shooting Formulation
            """

            #   Start with an empty NLP
            w = []
            G = []

            _costFunction = 0
            
            for k in range( self.__N ):

                print(k)
                
                #   New NLP variable for the control
                Uk = ca.SX.sym( 'U_' + str(k), self.__Nu )
                w += [ Uk ]

                currentRef = ca.vertcat( _reference[ k * self.__Nx ], _reference[ k * self.__Nx + 1 ], _reference[ k * self.__Nx + 2 ] )

                if( k == 0 ):
                    Fk = self.__F_gpOn( _initPose, currentRef, Uk )

                else:
                    Fk = self.__F_gpOn( Xk, currentRef, Uk )

                Xk = Fk[0]
                _costFunction += Fk[1]
                
                #   Inequality constraint
                G += [ Xk ]
            
            #   Create NLP solver
            prob = { 'f': _costFunction, 'x': ca.vertcat( *w ), 'g': ca.vertcat( *G ), 'p': param_s }
            self.__solverDSS = ca.nlpsol('solver', self.__optSolver, prob, self.__optOptions )

        else:

            #############################################################################################################################
            #############################################################################################################################

            X = X0
            Q = 0

            ###     Without gaussian processes      ######
            f = ca.Function( 'f', [ states, statesRef, controls ],\
                                        [ self.__ode( states, controls ),\
                                            self.__J( states, statesRef, controls, self.__weightMatrix_1, self.__weightMatrix_2, 0 ) ] )
            
            for j in range( self.__M ):
                k1, k1_q = f( X,             X_Ref, U )
                k2, k2_q = f( X + DT/2 * k1, X_Ref, U )
                k3, k3_q = f( X + DT/2 * k2, X_Ref, U )
                k4, k4_q = f( X + DT * k3,   X_Ref, U )
                X = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
                Q = Q + DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)
            self.__F = ca.Function( 'F', [ X0, X_Ref, U ], [ X, Q ] )
            ######

            #############################################################################################################################
            #############################################################################################################################

            """
                NMPC Direct Multiple Shooting Formulation
            """

            #   Start with an empty NLP
            w = []
            G = []

            _costFunction = 0
            
            """ Parameters """
            _initPose = ca.SX.sym( '_initPose', self.__Nx )
            _reference = ca.SX.sym( '_reference', self.__N * self.__Nx )
            
            param_s = ca.vertcat( _initPose, _reference )

            for k in range( self.__N ):
                
                #   New NLP variable for the control
                Uk = ca.SX.sym( 'U_' + str(k), self.__Nu )
                w += [ Uk ]

                currentRef = ca.vertcat( _reference[ k * self.__Nx ], _reference[ k * self.__Nx + 1 ], _reference[ k * self.__Nx + 2 ] )

                if( k == 0 ):
                    Fk = self.__F( _initPose, currentRef, Uk )

                else:
                    Fk = self.__F( Xk, currentRef, Uk )

                Xk_end = Fk[0]
                _costFunction += Fk[1]

                #   New NLP variable for state at end of interval
                Xk = ca.SX.sym( 'X_' + str(k + 1), self.__Nx )
                w += [ Xk ]
                
                #   Continuity constraint
                G += [ Xk_end - Xk ]
            
            #   Create NLP solver
            prob = { 'f': _costFunction, 'x': ca.vertcat(*w), 'g': ca.vertcat(*G), 'p': param_s }
            self.__solverDMS = ca.nlpsol( 'solver', self.__optSolver, prob, self.__optOptions )

            #############################################################################################################################
            #############################################################################################################################
        
            """
                NMPC Direct Single Shooting Formulation
            """

            #   Start with an empty NLP
            w = []
            G = []

            _costFunction = 0
            
            for k in range( self.__N ):
                
                #   New NLP variable for the control
                Uk = ca.SX.sym( 'U_' + str(k), self.__Nu )
                w += [ Uk ]

                currentRef = ca.vertcat( _reference[ k * self.__Nx ], _reference[ k * self.__Nx + 1 ], _reference[ k * self.__Nx + 2 ] )

                if( k == 0 ):
                    Fk = self.__F( _initPose, currentRef, Uk )

                else:
                    Fk = self.__F( Xk, currentRef, Uk )

                Xk = Fk[0]
                _costFunction += Fk[1]
                
                #   Inequality constraint
                G += [ Xk ]
            
            #   Create NLP solver
            prob = { 'f': _costFunction, 'x': ca.vertcat( *w ), 'g': ca.vertcat( *G ), 'p': param_s }
            self.__solverDSS = ca.nlpsol('solver', self.__optSolver, prob, self.__optOptions )

        ###     A-priori model      ######
        deltaT = ca.SX.sym( 'deltaT', 1 )
        f_ode = ca.Function( 'f_ode', [ states, controls, deltaTime ], [ self.__ode( states, controls ) ] )

        X = X0

        for j in range( self.__M ):
            k1 = f_ode( X            , U, deltaT )
            k2 = f_ode( X + deltaT/2 * k1, U, deltaT )
            k3 = f_ode( X + deltaT/2 * k2, U, deltaT )
            k4 = f_ode( X + deltaT * k3  , U, deltaT )
            X = X + deltaT / 6 * ( k1 + 2 * k2 + 2 * k3 + k4 )
        self.__F_ode = ca.Function( 'F_ode', [ X0, U, deltaT ], [ X ] )
        ######
        
    #   Direct Multiple Shooting
    def _solveDMS( self, initialGuess, optVarLB, optVarUB, ineConsLB, ineConsUB, parameters, optNumber ):

        if( optNumber == 0 ):
            w0 = ca.vertcat( *initialGuess )
        
        else:
            w0 = initialGuess

        lbw = ca.vertcat( *optVarLB )
        ubw = ca.vertcat( *optVarUB )
        lbg = ca.vertcat( *ineConsLB )
        ubg = ca.vertcat( *ineConsUB )

        param = ca.vertcat( *parameters )

        #   Solve the NLP
        sol = self.__solverDMS( x0 = w0, lbx = lbw, ubx = ubw, lbg = lbg, ubg = ubg, p = param )

        return sol

    #   Direct Single Shooting
    def _solveDSS( self, initialGuess, optVarLB, optVarUB, ineConsLB, ineConsUB, parameters, optNumber ):

        if( optNumber == 0 ):
            w0 = ca.vertcat( *initialGuess )

        else:
            w0 = initialGuess
            
        lbw = ca.vertcat( *optVarLB )
        ubw = ca.vertcat( *optVarUB )
        lbg = ca.vertcat( *ineConsLB )
        ubg = ca.vertcat( *ineConsUB )

        param = ca.vertcat( *parameters )

        #   Solve the NLP
        sol = self.__solverDSS( x0 = w0, lbx = lbw, ubx = ubw, lbg = lbg, ubg = ubg, p = param )

        return sol
    
    def _selectCommand( self, optSolution ):

        actuation = Twist()

        for i in range( self.__Nu ):

            if( i == 0 ):
                actuation.linear.x = optSolution[ i ]
            
            elif( i == 1 ):
                actuation.angular.z = optSolution[ i ]
        
        return actuation

    def _ODE(self, pose, actuation, time):

        """
            Return discretized ODE
        """

        return self.__F_ode( pose, actuation, time )