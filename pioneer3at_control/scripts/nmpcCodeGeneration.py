import numpy as np

from sys import path
path.append(r"/home/fmccastro/Thesis_RoverNavigation/ROS_workspaces/ws_1/src/casadi-linux-py27-v3.5.1-64bit")
from casadi import *

if __name__ == '__main__':
    """
    x = MX.sym("x")             #   States
    y = MX.sym("y")
    theta = MX.sym("theta")

    states = vertcat(x, y, theta)

    V_x = MX.sym("V_x")         #   Controls
    W_z = MX.sym("W_z")

    controls = vertcat(V_x, W_z)

    W = diag( SX( [0.8, 0.8, 0.8, 0.8, 0.8] ) )

    WN = diag( SX( [0.1, 0.1, 0.1] ) )    

    #   A-priori model
    #f = Function( 'f', [ theta, V_x, W_z ], [ cos(theta) * V_x, sin(theta) * V_x, W_z ] )
    xdot = vertcat( cos(theta) * V_x, sin(theta) * V_x, W_z )

    #   Cost function
    L = 

    #############################################################################################
    """
    Ts = 0.1            #   Sampling Time
    N = 60              #   Control Intervals

    T = N * Ts          #   Time Horizon


    """
    #   Integrator to discretize the system
    intg_options = { 'tf': Ts, 'simplify': true, 'number_of_finite_elements': 4 }                                            }
    
    #   DAE problem structure
    dae = { 'x': x, 'y': y, 'V_x': V_x, 'W_z': W_z, 'ode': f }
    
    intg = integrator( 'intg', 'rk', dae, intg_options)

    #############################################################################################
    """

    opti = casadi.Opti()

    states = opti.variable( 3, N + 1 )      #   
    controls = opti.variable( 2, N )
    initialState = opti.parameter( 3, 1 )

    #opti.minimize( 0.8 * states +      )

    opti.subject_to()
    opti.subject_to( -0.7 <= controls( 0, :) <= 0.7 )
    opti.subject_to( -140*pi/180 <= controls( 1, :) <= 140 * pi / 180 )




    


