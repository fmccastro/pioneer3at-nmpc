#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import skfmm
import math
import numpy as np
import cv2 as cv
import sys

from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt

from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

from PIL import Image
from casadi import *

from scipy.spatial.transform import Rotation as R

import os
os.chdir("/home/fmccastro/Desktop")

class MyCallback(Callback):
    def __init__( self, name, h, img, opts = {} ):
        Callback.__init__(self)
        self.height = h
        self.image = img
        self.construct(name, opts)
    
    #   Number of inputs and outputs
    def get_n_in( self ):   return 2
    def get_n_out( self ):  return 1

    #   Initialize the object
    def init( self ):
        print('initializing_object')
    
    #   Evaluate numerically
    def eval( self, arg ):
        f = self.image[ arg[0], arg[1] ] * self.height
        return [f]

#mapHeight = img[ ca.floor( contactsInertialFrame[row, 0] ), ca.floor( contactsInertialFrame[row, 1] ) ] * heightProportion

if __name__ == '__main__':
    
    a = MX.sym('a')
    b = MX.sym('b')
    
    img = cv.imread("heightmap.png", cv.IMREAD_GRAYSCALE)

    f = MyCallback( 'f', 15.0/255, img )

    print( f(a, b) )

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    heightProportion = 15.0/255

    contactsRobotFrame = MX( np.array( [ 
                                            [ 0.0, 0.0, 0.0 ],
                                            [ 143.0, 151.5, -138.5 ],
                                            [ 143.0, 248.5, -138.5 ],
                                            [ 143.0, -151.5, -138.5 ],
                                            [ 143.0, -248.5, -138.5 ],
                                            [ -143.0, 151.5, -138.5 ],
                                            [ -143.0, 248.5, -138.5 ],
                                            [ -143.0, -151.5, -138.5 ],
                                            [ -143.0, -248.5, -138.5 ],       
                                                                        ] ) * 0.001 )
    
    print( contactsRobotFrame )
    
    X = range( img.shape[0] )
    Y = range( img.shape[1] )
    Y, X = np.meshgrid(X, Y)

    Z = img[ X, Y ] * ( 15.0 / 255 )

    print( X )
    print( Y )

    print(Z)

    z = MX.sym( 'z' )
    roll = MX.sym( 'roll' )
    pitch = MX.sym( 'pitch' )
    yaw = MX.sym( 'yaw' )

    yawMatrix = MX.sym( 'yawMatrix', 3, 3 )

    yawMatrix[0, 0] = cos( yaw )
    yawMatrix[0, 1] = -sin(yaw)
    yawMatrix[0, 2] = 0
    yawMatrix[1, 0] = sin(yaw)
    yawMatrix[1, 1] = cos(yaw)
    yawMatrix[1, 2] = 0
    yawMatrix[2, 0] = 0
    yawMatrix[2, 1] = 0
    yawMatrix[2, 2] = 1

    pitchMatrix = MX.sym( 'pitchMatrix', 3, 3 )

    pitchMatrix[0, 0] = cos(pitch)
    pitchMatrix[0, 1] = 0
    pitchMatrix[0, 2] = sin(pitch)
    pitchMatrix[1, 0] = 0
    pitchMatrix[1, 1] = 1
    pitchMatrix[1, 2] = 0
    pitchMatrix[2, 0] = -sin(pitch)
    pitchMatrix[2, 1] = 0
    pitchMatrix[2, 2] = cos(pitch)

    rollMatrix = MX.sym( 'rollMatrix', 3, 3 )

    rollMatrix[0, 0] = 1
    rollMatrix[0, 1] = 0
    rollMatrix[0, 2] = 0
    rollMatrix[1, 0] = 0
    rollMatrix[1, 1] = cos(roll)
    rollMatrix[1, 2] = -sin(roll)
    rollMatrix[2, 0] = 0
    rollMatrix[2, 1] = sin(roll)
    rollMatrix[2, 2] = cos(roll)

    A = mtimes( mtimes( yawMatrix, pitchMatrix ), rollMatrix )

    contactsInertialFrame = MX( np.zeros( ( 9, 3 ) ) )
    print( contactsInertialFrame )

    grade = atan( 35/100 )

    #print( grade )

    #for i in range( img.shape[0] ):
    #    for j in range( img.shape[1] ):

    i = 267
    j = 451

    w = []
    w0 = []

    lbw = []
    ubw = []

    displacement = MX.sym( 'displacement', 3, 1 )
    displacement[0, 0] = i
    displacement[1, 0] = j
    displacement[2, 0] = z
    
    for row in range( contactsRobotFrame.shape[0] ):

        contactsInertialFrame[ row, : ] = mtimes( A, contactsRobotFrame[ row , : ].T ) + displacement
        
        mapHeight = f( floor( contactsInertialFrame[row, 0] ), floor( contactsInertialFrame[row, 1] ) )

        w0 += [0]
        w += [ contactsInertialFrame[row, 2] - mapHeight ]

        lbw += [ 0 ]
        ubw += [ np.inf ]

    #print( contactsInertialFrame )

    w0 += [ 0, 0 ]

    w += [ roll, pitch, yaw ]

    lbw = [ - grade, -grade, -np.inf ]
    ubw = [ grade, grade, np.inf ]

    prob = { 'f': z, 'x': w }
    solver = nlpsol('solver', 'ipopt', prob)
    
    #   Solve the NLP
    sol = solver( x0 = w0, lbx = lbw, ubx = ubw )
    w_opt = sol['x']

    print( w_opt )