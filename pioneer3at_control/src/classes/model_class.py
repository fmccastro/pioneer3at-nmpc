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
    def __init__( self, Nx, Nu, ode, J, controlIntervals, weightMatrix_1, weightMatrix_2, samplingTime, spaceSetLowerBounds, spaceSetUpperBounds, controLowerBounds, controlUpperBounds, transMethod, optimization, gpOnOff ):
        
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
        
        self.__Nx = Nx                
        self.__Nu = Nu
        self.__dt = samplingTime
        self.__N = controlIntervals
        self.__x_lb = spaceSetLowerBounds
        self.__x_ub = spaceSetUpperBounds
        self.__u_lb = controLowerBounds                       
        self.__u_ub = controlUpperBounds
        self.__ode = ode
        self.__gpOnOff = gpOnOff
        self.__transMethod = transMethod
        self.__opt = optimization

        states = ca.MX.sym( 'states', self.__Nx )
        controls = ca.MX.sym( 'controls', self.__Nu )
        gpDisturbances = ca.MX.sym( 'gpDisturbances', self.__Nx )

        statesRef = ca.MX.sym( 'statesRef', self.__Nx )
        
        """ Discrete Time Dynamics Formulation """
        self.__M = 4
        DT = self.__dt / self.__M

        X0 = ca.MX.sym( 'X0', self.__Nx )
        X_Ref = ca.MX.sym( 'X0_Ref', self.__Nx )
        U = ca.MX.sym( 'U', self.__Nu )

        X = X0
        Q = 0

        ###     Without gaussian processes      ######
        f = ca.Function( 'f', [ states, statesRef, controls ],\
                                    [ self.__ode( states, controls ),\
                                        J( states, statesRef, controls, weightMatrix_1, weightMatrix_2, 0 ) ] )
        
        for j in range( self.__M ):
            k1, k1_q = f( X,             X_Ref, U )
            k2, k2_q = f( X + DT/2 * k1, X_Ref, U )
            k3, k3_q = f( X + DT/2 * k2, X_Ref, U )
            k4, k4_q = f( X + DT * k3,   X_Ref, U )
            X = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            Q = Q + DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)
        self.__F = ca.Function( 'F', [ X0, X_Ref, U ], [ X, Q ] )
        ######

        ###     With gaussian processes     ######
        f_gpOn = ca.Function( 'f_gpOn', [ states, statesRef, controls, gpDisturbances ],\
                                    [ self.__ode( states, controls, gpDisturbances ),\
                                        J( states, statesRef, controls, weightMatrix_1, weightMatrix_2, 0 ) ] )

        GP = ca.MX.sym( 'GP', self.__Nx )
        
        for j in range( self.__M ):
            k1, k1_q = f_gpOn( X,             X_Ref, U, GP )
            k2, k2_q = f_gpOn( X + DT/2 * k1, X_Ref, U, GP )
            k3, k3_q = f_gpOn( X + DT/2 * k2, X_Ref, U, GP )
            k4, k4_q = f_gpOn( X + DT * k3,   X_Ref, U, GP )
            X = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            Q = Q + DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)
        self.__F_gpOn = ca.Function( 'F_gpOn', [ X0, X_Ref, U, GP ], [ X, Q ] )
        ######

        ###     A-priori model      ######
        f_ode = ca.Function( 'f_ode', [ states, controls ], [ self.__ode( states, controls ) ] )

        X = X0

        for j in range( self.__M ):
            k1 = f_ode( X            , U )
            k2 = f_ode( X + DT/2 * k1, U )
            k3 = f_ode( X + DT/2 * k2, U )
            k4 = f_ode( X + DT * k3  , U )
            X = X + DT / 6 * ( k1 + 2 * k2 + 2 * k3 + k4 )
        self.__F_ode = ca.Function( 'F_ode', [ X0, U ], [ X ] )
        ######

        if( self.__opt == 0 ):
            self.__optOptions = { 'qpsol': 'qpoases', 'qpsol_options': { 'printLevel': 'none' } }
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
                    }

                    #'expand': True
                }

            self.__optSolver = 'qrsqp'
        
        """
            NMPC Direct Multiple Shooting Formulation
        """

        #   Start with an empty NLP
        w = []
        G = []

        _costFunction = 0
        
        """ Parameters """
        _initPose = ca.MX.sym( '_initPose', self.__Nx )
        _reference = ca.MX.sym( '_reference', self.__N * self.__Nx )
        
        param_s = ca.vertcat( _initPose, _reference )

        for k in range( self.__N ):
            
            #   New NLP variable for the control
            Uk = ca.MX.sym( 'U_' + str(k), self.__Nu )
            w += [ Uk ]

            currentRef = ca.vertcat( _reference[ k * self.__Nx ], _reference[ k * self.__Nx + 1 ], _reference[ k * self.__Nx + 2 ] )

            if( k == 0 ):
                Fk = self.__F( _initPose, currentRef, Uk )

            else:
                Fk = self.__F( Xk, currentRef, Uk )

            Xk_end = Fk[0]
            _costFunction += Fk[1]

            #   New NLP variable for state at end of interval
            Xk = ca.MX.sym( 'X_' + str(k + 1), self.__Nx )
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
            Uk = ca.MX.sym( 'U_' + str(k), self.__Nu )
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

    def _ODE(self, pose, actuation):

        """
            Return discretized ODE
        """

        return self.__F_ode( pose, actuation )

    """def _solveDMS( self, _measurement, _step, _lastSolution = None, _reference = None, _refType = None, _warmStart = False, _samplingTime = None, _gaussianProcess = None ):

        
            NMPC discretised with Direct Multiple Shooting and integration is performed with Runge Kutta 4

            #   Inputs:
                    _measurement:
                    _lastSolution:
                    trajectory:
                    _step:
                    type: 
                    _warmStart:

            #   Output:

        if( _samplingTime == None ):
            pass

        else:
            self.__dt = self.__dt

        Start with an empty NLP
        w = []
        w0 = []
        lbw = []
        ubw = []
        
        G = []
        lbg = []
        ubg = []

        J = 0

        Initial conditions
        Xk = ca.MX.sym( 'X_0', self.__Nx )
        w += [Xk]

        lbw += [ _measurement.linear.x, _measurement.linear.y, _measurement.angular.z ]
        ubw += [ _measurement.linear.x, _measurement.linear.y, _measurement.angular.z ]

        if( _warmStart is False ):
            w0 = [0, 0, 0]

        else:
            w0 = _lastSolution[ self.__Nx + self.__Nu :: ]
            w0 = ca.vertcat( w0, _lastSolution[ -(self.__Nx + self.__Nu) :: ] ) 
        
        for k in range( self.__N ):
            
            #   New NLP variable for the control
            Uk = ca.MX.sym( 'U_' + str(k), self.__Nu )
            w += [ Uk ]
            lbw += self.__u_lb
            ubw += self.__u_ub
            
            if( _warmStart is False ):
                w0 += [0, 0]

            if( _refType == 0 ):

                if( k + _step > _reference.shape[0] - 1 ):
                    currentRef = ca.vertcat( _reference[ -1, 0 ], _reference[ -1, 1 ], _reference[ -1, 2 ] )
                
                else:
                    currentRef = ca.vertcat( _reference[ k + _step, 0 ], _reference[ k + _step, 1 ], _reference[ k + _step, 2 ] )
            
            elif( _refType == 1 ):

                #   Integrate till the end of the interval
                #dtraj_dt = reference.deriv( step * self.__dt + k * self.__dt )
                #Uref = [ math.sqrt( dtraj_dt[0]**2 + dtraj_dt[1]**2 ), dtraj_dt[2] ]

                currentRef = np.array( _reference.eval( _step + k * self.__dt ) )
            
            else:
                currentRef = ca.vertcat( _reference[ k, 0 ], _reference[ k, 1 ], _reference[ k, 2 ] )

            if( self.__gpOnOff is False ):
                Fk = self.__F( Xk, currentRef, Uk )

            elif( self.__gpOnOff == True and _gaussianProcess == None ):
                Fk = self.__F( Xk, currentRef, Uk )
            
            elif( self.__gpOnOff == True and _gaussianProcess != None ):
                gp_Pred = ca.vertcat( _gaussianProcess._prediction( 0, ca.transpose( ca.vertcat( Xk, Uk ) ) ),\
                                      _gaussianProcess._prediction( 1, ca.transpose( ca.vertcat( Xk, Uk ) ) ),\
                                      _gaussianProcess._prediction( 2, ca.transpose( ca.vertcat( Xk, Uk ) ) ) )
                Fk = self.__F_gpOn( Xk, currentRef, Uk, gp_Pred )

            Xk_end = Fk[0]

            J += Fk[1]

            #   New NLP variable for state at end of interval
            Xk = ca.MX.sym( 'X_' + str(k + 1), self.__Nx )
            w += [ Xk ]
            lbw += self.__x_lb
            ubw += self.__x_ub

            if( _warmStart is False ):
                w0 += [0, 0, 0]
            
            #   Continuity constraint
            G += [ Xk_end - Xk ]
            lbg += [ 0, 0, 0 ]
            ubg += [ 0, 0, 0 ]
        
        #   Create NLP solver
        prob = { 'f': J, 'x': ca.vertcat(*w), 'g': ca.vertcat(*G) }
        solver = ca.nlpsol('solver', self.__optSolver, prob, self.__optOptions )

        if( _warmStart is False ):
            w0 = ca.vertcat( *w0 )
        
        lbw = ca.vertcat( *lbw )
        ubw = ca.vertcat( *ubw )
        lbg = ca.vertcat( *lbg )
        ubg = ca.vertcat( *ubg )
        
        #   Solve the NLP
        sol = solver( x0 = w0, lbx = lbw, ubx = ubw, lbg = lbg, ubg = ubg )

        # Plot the solution
        return sol"""

    """def _solveDSS( self, _measurement, _step, _lastSolution, _reference, _refType, _warmStart, _samplingTime, _gaussianProcess ):

            NMPC discretised with Direct Single Shooting and integration is performed with Runge Kutta 4

            #   Inputs:
                    _measurement:
                    lastSolution:
                    trajectory:
                    _step:
                    type: 
                    warmStart:

            #   Output:

        if( _samplingTime == None ):
            pass

        else:
            self.__dt = self.__dt

        Start with an empty NLP
        w = []
        w0 = []
        lbw = []
        ubw = []
        
        G = []
        lbg = []
        ubg = []

        J = 0

        Initial conditions
        Xk = ca.vertcat( _measurement.linear.x, _measurement.linear.y, _measurement.angular.z )

        if( _warmStart ):
            w0 = _lastSolution[ self.__Nu:: ] 
            w0 = ca.vertcat( w0, _lastSolution[ -self.__Nu:: ] ) 
        
        for k in range( self.__N ):
            
            #   New NLP variable for the control
            Uk = ca.MX.sym( 'U_' + str(k), self.__Nu )
            w += [ Uk ]
            lbw += self.__u_lb
            ubw += self.__u_ub
            
            if( _warmStart is False ):
                w0 += [0, 0]

            if( _refType == 0 ):

                if( k + _step > _reference.shape[0] - 1 ):
                    currentRef = ca.vertcat( _reference[ -1, 0 ], _reference[ -1, 1 ], _reference[ -1, 2 ] )
                
                else:
                    currentRef = ca.vertcat( _reference[ k + _step, 0 ], _reference[ k + _step, 1 ], _reference[ k + _step, 2 ] )
            
            elif( _refType == 1 ):

                #   Integrate till the end of the interval
                #dtraj_dt = reference.deriv( step * self.__dt + k * self.__dt )
                #Uref = [ math.sqrt( dtraj_dt[0]**2 + dtraj_dt[1]**2 ), dtraj_dt[2] ]

                currentRef = np.array( _reference.eval( _step + k * self.__dt ) )
            
            else:
                currentRef = ca.vertcat( _reference[ k, 0 ], _reference[ k, 1 ], _reference[ k, 2 ] )

            if( self.__gpOnOff is False ):
                Fk = self.__F( Xk, currentRef, Uk )

            elif( self.__gpOnOff == True and _gaussianProcess == None ):
                Fk = self.__F( Xk, currentRef, Uk )
            
            elif( self.__gpOnOff == True and _gaussianProcess != None ):
                gp_Pred = ca.vertcat( _gaussianProcess._prediction( 0, ca.transpose( ca.vertcat( Xk, Uk ) ) ),\
                                      _gaussianProcess._prediction( 1, ca.transpose( ca.vertcat( Xk, Uk ) ) ),\
                                      _gaussianProcess._prediction( 2, ca.transpose( ca.vertcat( Xk, Uk ) ) ) )
                Fk = self.__F_gpOn( Xk, currentRef, Uk, gp_Pred )

            Xk = Fk[0]

            J += Fk[1]
            
            #   Inequality constraint
            G += [ Xk ]
            lbg += self.__x_lb
            ubg += self.__x_ub
        
        #   Create NLP solver
        prob = { 'f': J, 'x': ca.vertcat(*w), 'g': ca.vertcat(*G) }
        solver = ca.nlpsol('solver', self.__optSolver, prob, self.__optOptions )

        if( _warmStart is False ):
            w0 = ca.vertcat( *w0 )
        
        lbw = ca.vertcat( *lbw )
        ubw = ca.vertcat( *ubw )
        lbg = ca.vertcat( *lbg )
        ubg = ca.vertcat( *ubg )
        
        #   Solve the NLP
        sol = solver( x0 = w0, lbx = lbw, ubx = ubw, lbg = lbg, ubg = ubg )

        return sol"""