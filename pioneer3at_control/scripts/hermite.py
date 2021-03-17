#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import sys, math, os, skfmm
import cv2 as cv
import matplotlib.cm as cm

from klampt.model import trajectory
from klampt import vis
from scipy import interpolate, signal

if __name__ == '__main__':

    os.chdir("/home/fmccastro/Desktop")

    img = cv.imread( "heightmap.png", cv.IMREAD_GRAYSCALE )
    img = cv.flip( img, 0 )

    heightProportion = 2.0/255

    grade = math.atan(35/100)

    X, Y = np.meshgrid( np.linspace( 0, img.shape[0] - 1, img.shape[0] ), np.linspace( 0, img.shape[1] - 1, img.shape[1] ) )

    path = [
                [ 194, 80 ],
                [ 124, 106.667 ],
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

    print( path )
    
    traj_aux = trajectory.Trajectory( milestones = path )

    traj = trajectory.HermiteTrajectory()
    traj.makeSpline( traj_aux )

    #traj_timed = trajectory.path_to_trajectory( traj,  )

    endTime = traj.endTime()

    print( endTime )

    sys.stdin.read( 1 )

    i = 0

    param = []

    while( i < endTime ):

        print(i)

        points = traj.eval(i)

        param.append( list( points[ :2 ] ) )

        i += 0.05

    print( param )

    param = np.array( param )
    path = np.array( path )

    cmap = cm.get_cmap('jet')
    plt.plot( param[:, 0], param[:, 1], color = "white" )
    plt.plot( path[:, 0], path[:, 1], 'ro' )
    plt.contourf( X, Y, img, 100, cmap = cmap)
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
    plt.gca().xaxis.set_major_locator(plt.NullLocator())
    plt.gca().yaxis.set_major_locator(plt.NullLocator())
    plt.colorbar()
    plt.show()
