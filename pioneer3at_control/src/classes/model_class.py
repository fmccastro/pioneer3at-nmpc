#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, math, time

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
        self.__Jopt = common.costFunction

        self.__weightMatrix_1 = common.Q
        self.__weightMatrix_2 = common.R
        self.__weightMatrix_3 = common.P

        self.__gp = gp
        self.__gpOnOff = common.gpOnOff

        states = ca.SX.sym( 'states', self.__Nx )
        controls = ca.SX.sym( 'controls', self.__Nu )
        prevControls = ca.SX.sym( 'prevControls', self.__Nu )
        deltaTime = ca.SX.sym( 'deltaTime', 1 )
        statesRef = ca.SX.sym( 'statesRef', self.__Nx )
        
        """ Discrete Time Dynamics Formulation """
        DT = self.__dt / self.__M

        X0 = ca.SX.sym( 'X0', self.__Nx )
        X_Ref = ca.SX.sym( 'X0_Ref', self.__Nx )
        U = ca.SX.sym( 'U', self.__Nu )
        prevU = ca.SX.sym( 'prevU', self.__Nu )

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

        #############################################################################################################################
        #############################################################################################################################

        X = X0
        Q = 0

        if( self.__gpOnOff ):
            self.__nbOutputs = common.nbOutputs

            gp_def = ca.SX.sym( 'gp_def', self.__nbOutputs )
            gp = ca.SX.sym( 'gp', self.__nbOutputs )

            ###     Explicit Runge-Kutta Integrator     ######
            f = ca.Function( 'f', [ states, statesRef, controls, prevControls, gp_def ],\
                                        [ self.__ode( states, controls, gp = gp_def ),\
                                            self.__J( states, statesRef, controls, prevControls, self.__weightMatrix_1, self.__weightMatrix_2, self.__weightMatrix_3, self.__Jopt ) ] )
            
            for _ in range( self.__M ):
                k1, k1_q = f( X, X_Ref, U, prevU, gp )
                k2, k2_q = f( X + DT/2 * k1, X_Ref, U, prevU, gp )
                k3, k3_q = f( X + DT/2 * k2, X_Ref, U, prevU, gp )
                k4, k4_q = f( X + DT * k3, X_Ref, U, prevU, gp )

                X = X + DT / 6 * ( k1 + 2 * k2 + 2 * k3 + k4 )
                Q = Q + DT / 6 * ( k1_q + 2 * k2_q + 2 * k3_q + k4_q )
            self.__F = ca.Function( 'F', [ X0, X_Ref, U, prevU, gp ], [ X, Q ] )
        
        else:
            ###     Explicit Runge-Kutta Integrator     ######
            f = ca.Function( 'f', [ states, statesRef, controls, prevControls ],\
                                        [ self.__ode( states, controls ),\
                                            self.__J( states, statesRef, controls, prevControls, self.__weightMatrix_1, self.__weightMatrix_2, self.__weightMatrix_3, self.__Jopt ) ] )
            
            for _ in range( self.__M ):
                k1, k1_q = f( X, X_Ref, U, prevU )
                k2, k2_q = f( X + DT/2 * k1, X_Ref, U, prevU )
                k3, k3_q = f( X + DT/2 * k2, X_Ref, U, prevU )
                k4, k4_q = f( X + DT * k3, X_Ref, U, prevU )

                X = X + DT / 6 * ( k1 + 2 * k2 + 2 * k3 + k4 )
                Q = Q + DT / 6 * ( k1_q + 2 * k2_q + 2 * k3_q + k4_q )
            self.__F = ca.Function( 'F', [ X0, X_Ref, U, prevU ], [ X, Q ] )

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
        _prevCtrl = ca.SX.sym( '_prevCtrl', self.__Nu )
        
        if( self.__gpOnOff ):
            _gp = ca.SX.sym( '_gp', self.__nbOutputs * self.__N )

            param_s = ca.vertcat( _initPose, _prevCtrl, _reference, _gp )
        
        else:
            param_s = ca.vertcat( _initPose, _prevCtrl, _reference )

        for k in range( self.__N ):
            
            #   New NLP variable for the control
            if( k > 0 ):
                prevUk = Uk

            Uk = ca.SX.sym( 'U_' + str(k), self.__Nu )
            w += [ Uk ]

            currentRef = ca.vertcat( _reference[ k * self.__Nx ], _reference[ k * self.__Nx + 1 ], _reference[ k * self.__Nx + 2 ] )

            if( k == 0 ):
                if( self.__gpOnOff ):
                    Fk = self.__F( _initPose, currentRef, Uk, _prevCtrl, ca.vertcat( _gp[ k * self.__nbOutputs ], _gp[ k * self.__nbOutputs + 1 ], _gp[ k * self.__nbOutputs + 2 ] ) )

                else:
                    Fk = self.__F( _initPose, currentRef, Uk, _prevCtrl )
                    
            else:
                if( self.__gpOnOff ):
                    Fk = self.__F( Xk, currentRef, Uk, prevUk, ca.vertcat( _gp[ k * self.__nbOutputs ], _gp[ k * self.__nbOutputs + 1 ], _gp[ k * self.__nbOutputs + 2 ] ) )

                else:
                    Fk = self.__F( Xk, currentRef, Uk, prevUk )

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
            if( k > 0 ):
                prevUk = Uk

            Uk = ca.SX.sym( 'U_' + str(k), self.__Nu )
            w += [ Uk ]

            currentRef = ca.vertcat( _reference[ k * self.__Nx ], _reference[ k * self.__Nx + 1 ], _reference[ k * self.__Nx + 2 ] )

            if( k == 0 ):
                if( self.__gpOnOff ):
                    Fk = self.__F( _initPose, currentRef, Uk, _prevCtrl, ca.vertcat( _gp[ k * self.__nbOutputs ], _gp[ k * self.__nbOutputs + 1 ], _gp[ k * self.__nbOutputs + 2 ] ) )
                
                else:
                    Fk = self.__F( _initPose, currentRef, Uk, _prevCtrl )

            else:
                if( self.__gpOnOff ):
                    Fk = self.__F( Xk, currentRef, Uk, prevUk, ca.vertcat( _gp[ k * self.__nbOutputs ], _gp[ k * self.__nbOutputs + 1 ], _gp[ k * self.__nbOutputs + 2 ] ) )
                
                else:
                    Fk = self.__F( Xk, currentRef, Uk, prevUk )

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
        optStart = time.time()
        sol = self.__solverDMS( x0 = w0, lbx = lbw, ubx = ubw, lbg = lbg, ubg = ubg, p = param )
        optTime = time.time() - optStart

        return sol, optTime

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
        optStart = time.time()
        sol = self.__solverDSS( x0 = w0, lbx = lbw, ubx = ubw, lbg = lbg, ubg = ubg, p = param )
        optTime = time.time() - optStart

        return sol, optTime
    
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