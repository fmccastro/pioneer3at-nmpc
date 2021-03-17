import os, sys, skfmm, math, random
import numpy as np
import cv2 as cv
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import pylab as plt

from scipy.optimize import minimize, Bounds
from scipy import interpolate
from scipy import signal

os.chdir("/home/fmccastro/Desktop")

def vectorOrientation( array ):

    angle = 0

    angle = math.acos( np.dot( array, [ 1, 0 ] ) / np.linalg.norm( array )  )

    if( array[1] < 0 ):
        angle = -angle
    
    return angle

def costFunction( w ):

    return w[0]

def rotationMatrix( w, i = 0 ):

    yawMatrix = np.array( [ 
                            [ math.cos( w[3 - i] ), -math.sin( w[3 - i] ), 0 ],
                            [ math.sin( w[3 - i] ), math.cos( w[3 - i] ), 0 ],
                            [ 0, 0, 1 ]
                                                                            ] )
    
    pitchMatrix = np.array( [
                              [ math.cos( w[1 - i] ), 0, math.sin( w[1 - i] ) ],
                              [ 0, 1, 0 ],
                              [ -math.sin( w[1 - i] ), 0, math.cos( w[1 - i] ) ]
                                                                            ] )

    rollMatrix = np.array( [
                                [ 1, 0, 0 ],
                                [ 0, math.cos( w[2 - i] ), -math.sin( w[2 - i] ) ],
                                [ 0, math.sin( w[2 - i] ), math.cos( w[2 - i] ) ]
                                                                            ] )
    
    return np.matmul( np.matmul( yawMatrix, pitchMatrix ), rollMatrix )

if __name__ == '__main__':

    img = cv.imread("marsHeightmap2_teste.jpg", cv.IMREAD_GRAYSCALE)

    heightProportion = 2.0/255

    grade = math.atan(35/100)

    i = 0
    j = 0

    contactsRobotFrame = np.array( [
                                        [ 0.0, 0.0, 0.0 ],
                                        [ 143.0, 200.0, -138.5 ],
                                        [ 143.0, -200.0, -138.5 ],
                                        [ -143.0, 200.0, -138.5 ],
                                        [ -143.0, -200.0, -138.5 ]
                                                                    ] ) * 0.001
    
    #   z, pitch, roll, yaw
    bounds = Bounds( [ -np.inf, -grade, -grade, 0 ], [ np.inf, grade, grade, 2 * np.pi ] )
    
    K = 10
    
    X, Y = np.meshgrid( np.linspace( 0, img.shape[0] - 1, img.shape[0] ), np.linspace( 0, img.shape[1] - 1, img.shape[1] ) )

    x, y = np.arange( img.shape[0] ), np.arange( img.shape[1] )

    f = interpolate.RectBivariateSpline( x, y, img * heightProportion, kx = 5, ky = 5 )

    phi = ( X - 20 )**2 + ( Y - 10 )**2 - 4

    costMap = np.ones( ( img.shape[0], img.shape[1] ) )
    
    for i in range( img.shape[0] ):
        for j in range( img.shape[1] ):

            print( i, j )

            w0 = np.array( [ 0, 0, 0, 0 ] )

            ineqCons = { 'type': 'ineq',
                         'fun' : lambda w: np.array( [ (np.matmul( rotationMatrix(w), contactsRobotFrame[0,:] ) + np.array( [ i, j, w[0] ] ) )[2] - f( (np.matmul( rotationMatrix(w), contactsRobotFrame[0,:] ) + np.array( [ i, j, w[0] ] ) )[0], (np.matmul( rotationMatrix(w), contactsRobotFrame[0,:] ) + np.array( [ i, j, w[0] ] ) )[1] )[0, 0],
                                                       (np.matmul( rotationMatrix(w), contactsRobotFrame[1,:] ) + np.array( [ i, j, w[0] ] ) )[2] - f( (np.matmul( rotationMatrix(w), contactsRobotFrame[1,:] ) + np.array( [ i, j, w[0] ] ) )[0], (np.matmul( rotationMatrix(w), contactsRobotFrame[1,:] ) + np.array( [ i, j, w[0] ] ) )[1] )[0, 0],
                                                       (np.matmul( rotationMatrix(w), contactsRobotFrame[2,:] ) + np.array( [ i, j, w[0] ] ) )[2] - f( (np.matmul( rotationMatrix(w), contactsRobotFrame[2,:] ) + np.array( [ i, j, w[0] ] ) )[0], (np.matmul( rotationMatrix(w), contactsRobotFrame[2,:] ) + np.array( [ i, j, w[0] ] ) )[1] )[0, 0],
                                                       (np.matmul( rotationMatrix(w), contactsRobotFrame[3,:] ) + np.array( [ i, j, w[0] ] ) )[2] - f( (np.matmul( rotationMatrix(w), contactsRobotFrame[3,:] ) + np.array( [ i, j, w[0] ] ) )[0], (np.matmul( rotationMatrix(w), contactsRobotFrame[3,:] ) + np.array( [ i, j, w[0] ] ) )[1] )[0, 0],
                                                       (np.matmul( rotationMatrix(w), contactsRobotFrame[4,:] ) + np.array( [ i, j, w[0] ] ) )[2] - f( (np.matmul( rotationMatrix(w), contactsRobotFrame[4,:] ) + np.array( [ i, j, w[0] ] ) )[0], (np.matmul( rotationMatrix(w), contactsRobotFrame[4,:] ) + np.array( [ i, j, w[0] ] ) )[1] )[0, 0]
                                                            ] ) }
            
            res = minimize( costFunction, w0, method = 'SLSQP',
                            constraints = [ ineqCons ], options = {'ftol': 1e-9, 'disp': True},
                            bounds = bounds )

            if( res.x[2] == 0 ):
                v1 = np.array( [ 0, 1, 0 ] )
            elif( res.x[2] == np.pi/2 ):
                v1 = np.array( [ 0, 0, 1 ] )
            else:
                v1 = np.array( [ 0, math.cos( res.x[2] ), math.sin( res.x[2] ) ] )

            if( res.x[1] == 0 ):
                v2 = np.array( [ 1, 0, 0 ] )
            elif( res.x[1] == np.pi/2 ):
                v2 = np.array( [ 0, 0, 1 ] )
            else:
                v2 = np.array( [ math.cos( res.x[1] ), 0, math.sin( res.x[1] ) ] )
            
            v3 = v1 + v2

            aux = [ ( np.matmul( rotationMatrix( [ res.x[1], res.x[2], res.x[3] ], i = 1 ), contactsRobotFrame[0,:] ) + np.array( [ i, j, res.x[0] ] ) ),
                    ( np.matmul( rotationMatrix( [ res.x[1], res.x[2], res.x[3] ], i = 1 ), contactsRobotFrame[1,:] ) + np.array( [ i, j, res.x[0] ] ) ),
                    ( np.matmul( rotationMatrix( [ res.x[1], res.x[2], res.x[3] ], i = 1 ), contactsRobotFrame[2,:] ) + np.array( [ i, j, res.x[0] ] ) ),
                    ( np.matmul( rotationMatrix( [ res.x[1], res.x[2], res.x[3] ], i = 1 ), contactsRobotFrame[3,:] ) + np.array( [ i, j, res.x[0] ] ) ),
                    ( np.matmul( rotationMatrix( [ res.x[1], res.x[2], res.x[3] ], i = 1 ), contactsRobotFrame[4,:] ) + np.array( [ i, j, res.x[0] ] ) ) ]
            
            count = 0
            
            angle =  math.acos( v3[2]/ np.linalg.norm(v3) )

            if( abs( np.pi/2 - angle ) > grade ):
                costMap[i, j] = 1000
                print("ZERO")
            else:
                costMap[i, j] = K * math.acos( v3[2] / np.linalg.norm(v3) )

    cmap = cm.get_cmap('jet')
    plt.contour( X, Y, phi, [0], linewidths=(3), colors='black')
    plt.contourf( X, Y, costMap, 100, cmap = cmap)
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
    plt.gca().xaxis.set_major_locator(plt.NullLocator())
    plt.gca().yaxis.set_major_locator(plt.NullLocator())
    plt.colorbar()
    plt.show()

    t = skfmm.travel_time( phi, costMap )

    cmap = cm.get_cmap('jet')
    plt.contour( X, Y, phi, [0], linewidths=(3), colors='black')
    plt.contourf( X, Y, t, 100, cmap = cmap)
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
    plt.gca().xaxis.set_major_locator(plt.NullLocator())
    plt.gca().yaxis.set_major_locator(plt.NullLocator())
    plt.colorbar()
    plt.show()

    np.save( "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/scripts/timeMatrix.npy", t )
    np.save( "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/scripts/costMatrix.npy", costMap )

    #FFM Path Planning

    t = np.load( "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/scripts/timeMatrix.npy" )
    costMap = np.load( "/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/scripts/costMatrix.npy" )

    nbNodes = 100

    sobel_x = np.array( [ [-1, 0, 1], [-2, 0, 2], [-1, 0, 1] ] )
    sobel_y = np.array( [ [1, 2, 1], [0, 0, 0], [-1, -2, -1] ] )

    gradX =  signal.convolve2d( t, sobel_x, boundary = 'symm', mode = 'same' )
    gradY =  -signal.convolve2d( t, sobel_y, boundary = 'symm', mode = 'same' )

    cmap = cm.get_cmap('jet')
    plt.contour(X, Y, phi, [0], linewidths = (0.5), colors = 'black')
    #plt.contourf(X, Y, t, 100, cmap = cmap)
    plt.contourf(X, Y, gradX, 100, cmap = cmap)
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
    plt.gca().xaxis.set_major_locator( plt.NullLocator() )
    plt.gca().yaxis.set_major_locator( plt.NullLocator() )
    plt.colorbar()
    plt.show()

    cmap = cm.get_cmap('jet')
    plt.contour(X, Y, phi, [0], linewidths = (0.5), colors = 'black')
    #plt.contourf(X, Y, t, 100, cmap = cmap)
    plt.contourf(X, Y, gradY, 100, cmap = cmap)
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
    plt.gca().xaxis.set_major_locator( plt.NullLocator() )
    plt.gca().yaxis.set_major_locator( plt.NullLocator() )
    plt.colorbar()
    plt.show()

    currentPoint = [12, 11, 0]

    Coord_X = int( round( ( 129.0 / 25 ) * currentPoint[0] + 64.5 ) )
    Coord_Y = int( round( ( 129.0 / 25 ) * currentPoint[1] + 64.5 ) )

    print("X [px]", Coord_X)
    print("Y [px]", Coord_Y)

    print( "Travel Time[s]", t[128][0] )
    print( t )

    travelTime = t[Coord_Y][Coord_X]

    plt.ion()

    cmap = cm.get_cmap('jet')
    plt.contour( X, Y, phi, [0], linewidths=(3), colors='black')
    plt.contourf( X, Y, t, 100, cmap = cmap)
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
    plt.gca().xaxis.set_major_locator(plt.NullLocator())
    plt.gca().yaxis.set_major_locator(plt.NullLocator())
    plt.colorbar()

    plt.plot( Coord_X, Coord_Y, 'wx' )
    plt.draw()

    plt.pause(0.1)

    sys.stdin.read(1)

    i = 0

    while( travelTime > 0.5 and i < 100 ):

        grad_x = gradX[Coord_Y][Coord_X]
        grad_y = gradY[Coord_Y][Coord_X]

        print("Grad_X", grad_x)
        print("Grad_Y", grad_y)

        mod = math.sqrt( grad_x**2 + grad_y**2 )
        alpha = vectorOrientation( [ grad_x, grad_y ] )
        print("Heading [degrees]", alpha * 180/math.pi)

        currentPoint[0] = currentPoint[0] + 2 * math.cos(alpha)
        currentPoint[1] = currentPoint[1] + 2 * math.sin(alpha)
        currentPoint[2] = alpha

        print( "X [m]", currentPoint[0] )
        print( "Y [m]", currentPoint[1] )
        print( "Heading [radians]", currentPoint[2] )
        print("\n\n")

        Coord_X = int( round( ( 129.0 / 25 ) * currentPoint[0] + 64.5 ) )
        Coord_Y = int( round( ( 129.0 / 25 ) * currentPoint[1] + 64.5 ) )

        plt.plot( Coord_X, Coord_Y, 'o' )
        plt.draw()

        print("X [px]", Coord_X)
        print("Y [px]", Coord_Y)

        print( "Travel Time[s]", t[Coord_Y][Coord_X] )

        travelTime = t[Coord_Y][Coord_X]

        i += 1

        plt.pause(0.1)

        sys.stdin.read(1)