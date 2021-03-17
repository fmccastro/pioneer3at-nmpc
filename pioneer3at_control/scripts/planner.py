#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import skfmm
import math
import sys

import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt

from scipy import signal
from klampt.model import trajectory

def convolutionCorrection( array1, array2, indexes):

    toConvolute = np.array( array1 )
    kernel = np.array( array2 )
    index = indexes

    height = toConvolute.shape[0]
    width = toConvolute.shape[1]

    aux = True
    
    for row, col in enumerate(indexes):
        
        subMatrix = np.zeros( ( 3, 3 ) )
        value = 0

        #######################################################################

        if( 0 <= col[0] + 1 <= height - 1 and 0 <= col[1] - 1 <= width - 1 ):
            if( np.isnan( toConvolute[ col[0] + 1, col[1] - 1 ] ) == False ):
                subMatrix[0, 0] = toConvolute[ col[0] + 1, col[1] - 1 ]

        if( 0 <= col[0] + 1 <= height - 1 and 0 <= col[1] <= width - 1 ):
            if( np.isnan( toConvolute[ col[0] + 1, col[1] ] ) == False ):
                subMatrix[0, 1] = toConvolute[ col[0] + 1, col[1] ]
        
        if( 0 <= col[0] + 1 <= height - 1 and 0 <= col[1] + 1 <= width - 1 ):
            if( np.isnan( toConvolute[ col[0] + 1, col[1] + 1 ] ) == False ):
                subMatrix[0, 2] = toConvolute[ col[0] + 1, col[1] + 1 ]

        #######################################################################

        if( 0 <= col[0] <= height - 1 and 0 <= col[1] - 1 <= width - 1 ):
            if( np.isnan( toConvolute[ col[0], col[1] - 1 ] ) == False ):
                subMatrix[1, 0] = toConvolute[ col[0], col[1] - 1 ]

        if( 0 <= col[0] <= height - 1 and 0 <= col[1] <= width - 1 ):
            if( np.isnan( toConvolute[ col[0], col[1] ] ) == False ):
                subMatrix[1, 1] = toConvolute[ col[0], col[1] ]
        
        if( 0 <= col[0] <= height - 1 and 0 <= col[1] + 1 <= width - 1 ):
            if( np.isnan( toConvolute[ col[0], col[1] + 1 ] ) == False ):
                subMatrix[1, 2] = toConvolute[ col[0], col[1] + 1 ]

        #######################################################################

        if( 0 <= col[0] - 1 <= height - 1 and 0 <= col[1] - 1 <= width - 1 ):
            if( np.isnan( toConvolute[ col[0] - 1, col[1] - 1 ] ) == False ):
                subMatrix[2, 0] = toConvolute[ col[0] - 1, col[1] - 1 ]
        
        if( 0 <= col[0] - 1 <= height - 1 and 0 <= col[1] <= width - 1 ):
            if( np.isnan( toConvolute[ col[0] - 1, col[1] ] ) == False ):
                subMatrix[2, 1] = toConvolute[ col[0] - 1, col[1] ]
        
        if( 0 <= col[0] - 1 <= height - 1 and 0 <= col[1] + 1 <= width - 1 ):
            if( np.isnan( toConvolute[ col[0] - 1, col[1] + 1 ] ) == False ):
                subMatrix[2, 2] = toConvolute[ col[0] - 1, col[1] + 1 ]

        #######################################################################

        print( col )
        print( subMatrix )

        subMatrix = symmetricMatrix(subMatrix)

        print(subMatrix)

        multEleByEle = np.multiply( -kernel, subMatrix )
        value = multEleByEle.sum()
    
        if(aux):
            toReturn = np.array( [ [ col[0], col[1], value ] ] )
            aux = False
        else:
            toReturn = np.concatenate( ( toReturn, np.array( [ [ col[0], col[1], value ] ] ) ) )

    return toReturn

def symmetricMatrix( matrix ):

    newMatrix = np.array( matrix )

    if( newMatrix[0, 0] == 0 ):

        if( newMatrix[1, 0] == 0 and newMatrix[0, 1] == 0 ):
            newMatrix[0, 0] = newMatrix[1, 1]
            newMatrix[0, 1] = newMatrix[1, 1]
            newMatrix[1, 0] = newMatrix[1, 1]

        elif( newMatrix[1, 0] == 0 ):
            newMatrix[0, 0] = newMatrix[0, 1]
            newMatrix[1, 0] = newMatrix[1, 1]

        elif( newMatrix[0, 1] == 0 ):
            newMatrix[0, 0] = newMatrix[1, 0]
            newMatrix[0, 1] = newMatrix[1, 1]
        
        else:
            newMatrix[0, 0] = newMatrix[1, 1]
    
    if( newMatrix[0, 2] == 0 ):

        if( newMatrix[0, 1] == 0 and newMatrix[1, 2] == 0 ):
            newMatrix[0, 2] = newMatrix[1, 1]
            newMatrix[0, 1] = newMatrix[1, 1]
            newMatrix[1, 2] = newMatrix[1, 1]
        
        elif( newMatrix[0, 1] == 0 ):
            newMatrix[0, 2] = newMatrix[1, 2]
            newMatrix[0, 1] = newMatrix[1, 1]
        
        elif( newMatrix[1, 2] == 0 ):
            newMatrix[0, 2] = newMatrix[0, 1]
            newMatrix[1, 2] = newMatrix[1, 1]
        
        else:
            newMatrix[0, 2] = newMatrix[1, 1]
    
    if( newMatrix[2, 2] == 0 ):

        if( newMatrix[1, 2] == 0 and newMatrix[2, 1] == 0 ):
            newMatrix[2, 2] = newMatrix[1, 1]
            newMatrix[1, 2] = newMatrix[1, 1]
            newMatrix[2, 1] = newMatrix[1, 1]
        
        elif( newMatrix[1, 2] == 0 ):
            newMatrix[2, 2] = newMatrix[2, 1]
            newMatrix[1, 2] = newMatrix[1, 1]
        
        elif( newMatrix[2, 1] == 0 ):
            newMatrix[2, 2] = newMatrix[1, 2]
            newMatrix[2, 1] = newMatrix[1, 1]
        else:
            newMatrix[2, 2] = newMatrix[1, 1]
    
    if( newMatrix[2, 0] == 0 ):

        if( newMatrix[1, 0] == 0 and newMatrix[2, 1] == 0 ):
            newMatrix[2, 0] = newMatrix[1, 1]
            newMatrix[1, 0] = newMatrix[1, 1]
            newMatrix[2, 1] = newMatrix[1, 1]
        
        elif( newMatrix[1, 0] == 0 ):
            newMatrix[2, 0] = newMatrix[2, 1]
            newMatrix[1, 0] = newMatrix[1, 1]

        elif( newMatrix[2, 1] == 0 ):
            newMatrix[2, 0] = newMatrix[1, 0]
            newMatrix[2, 1] = newMatrix[1, 1]
        
        else:
            newMatrix[2, 0] = newMatrix[1, 1]
        
    if( newMatrix[0, 1] == 0 and newMatrix[0, 0] != 0 and newMatrix[0, 2] != 0 ):
        newMatrix[0, 1] = newMatrix[1, 1]
    
    if( newMatrix[1, 0] == 0 and newMatrix[0, 0] != 0 and newMatrix[2, 0] != 0 ):
        newMatrix[1, 0] = newMatrix[1, 1]
    
    if( newMatrix[2, 1] == 0 and newMatrix[2, 0] != 0 and newMatrix[2, 2] != 0 ):
        newMatrix[2, 1] = newMatrix[1, 1]
    
    if( newMatrix[1, 2] == 0 and newMatrix[0, 2] != 0 and newMatrix[2, 2] != 0 ):
        newMatrix[1, 2] = newMatrix[1, 1]

    return newMatrix

def vectorOrientation( array ):

    angle = 0

    angle = math.acos( np.dot( array, [ 1, 0 ] ) / np.linalg.norm( array )  )

    if( array[1] < 0 ):
        angle = -angle
    
    return angle

X, Y = np.meshgrid( np.linspace(-10, 10, 501), np.linspace(-10, 10, 501) )

phi = (X + 9)**2 + (Y - 6)**2 - 0.01

mask1 = np.logical_and( np.logical_and( Y >= -4, Y <= 10 ), np.logical_and( X >= 1, X <= 2 ) )
speed = 0.7 * np.ones_like(X)

mask2 = np.logical_and( np.logical_and( Y <= 4 , Y >= -10), np.logical_and( X >= 5, X <= 6 ) )
speed = 0.7 * np.ones_like(X)

mask = np.logical_or(mask1, mask2)

phi = np.ma.MaskedArray(phi, mask)
speed = np.ma.MaskedArray(speed, mask)

#mask = ( (X - 7)**2 + (Y - 7)**2 < 0.5 )
#phi = np.ma.MaskedArray(phi, mask)

t = skfmm.travel_time(phi, speed, dx = 20.0/501)
d, f_ext = skfmm.extension_velocities(phi, speed, dx = 20.0/501)

sobel_x = np.array( [ [-1, 0, 1], [-2, 0, 2], [-1, 0, 1] ] )
sobel_y = np.array( [ [1, 2, 1], [0, 0, 0], [-1, -2, -1] ] )

t[mask] = np.nan

gradX =  signal.convolve2d( t, sobel_x, boundary='symm', mode='same' )
gradY =  -signal.convolve2d( t, sobel_y, boundary='symm', mode='same' )

## Correction

gradX = np.ma.MaskedArray(gradX, mask)
gradY = np.ma.MaskedArray(gradY, mask)

gradX = np.array( gradX )
gradY = np.array( gradY )

gradX[mask] = np.inf
gradY[mask] = np.inf

indexNaN_gradX = np.argwhere( np.isnan( gradX ) )
indexNaN_gradY = np.argwhere( np.isnan( gradY ) )

corrections_X = convolutionCorrection( t, sobel_x, indexNaN_gradX )
corrections_Y = convolutionCorrection( t, sobel_y, indexNaN_gradY )

#print( corrections_X )
#print( corrections_Y )

print( t[0, 0] )
print( t[0, 1] )
print( t[1, 0] )
print( t[1, 1] )
#print( t[500, 0] )
#print(t)

print("\n\n")

print( gradX[0, 0] )
print( gradX[500, 0] )
print( gradY[0, 0] )
print( gradY[500, 0] )

for row_X, row_Y in zip( corrections_X, corrections_Y ):

    gradX[ int ( row_X[0] ), int( row_X[1] ) ] = row_X[2]
    gradY[ int( row_Y[0] ), int( row_Y[1] ) ] = row_Y[2]
###############################################################################

gradX[mask] = np.inf
gradY[mask] = np.inf

plt.figure(1)
plt.scatter( indexNaN_gradX[:, 1], indexNaN_gradX[:, 0], marker = 'o' )
plt.gca().set_aspect(1)

plt.figure(2)
cmap = cm.get_cmap('jet')
plt.contour(X, Y, phi, [0], linewidths = (0.5), colors = 'black')
plt.contour(X, Y, phi.mask, [0], linewidths = (0.5), colors = 'red')
plt.contourf(X, Y, t, 100, cmap = cmap)
plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
plt.gca().xaxis.set_major_locator( plt.NullLocator() )
plt.gca().yaxis.set_major_locator( plt.NullLocator() )
plt.colorbar()

plt.figure(3)
cmap = cm.get_cmap('jet')
#plt.contour(X, Y, phi, [0], linewidths = (0.5), colors = 'black')
plt.contour(X, Y, phi.mask, [0], linewidths = (0.5), colors = 'red')
#plt.contourf(X, Y, t, 100, cmap = cmap)
plt.contourf(X, Y, gradX, 100, cmap = cmap)
plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
plt.gca().xaxis.set_major_locator( plt.NullLocator() )
plt.gca().yaxis.set_major_locator( plt.NullLocator() )
plt.colorbar()

plt.figure(4)
cmap = cm.get_cmap('jet')
#plt.contour(X, Y, phi, [0], linewidths = (0.5), colors = 'black')
plt.contour(X, Y, phi.mask, [0], linewidths = (0.5), colors = 'red')
#plt.contourf(X, Y, t, 100, cmap = cmap)
plt.contourf(X, Y, gradY, 100, cmap = cmap)
plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
plt.gca().xaxis.set_major_locator( plt.NullLocator() )
plt.gca().yaxis.set_major_locator( plt.NullLocator() )
plt.colorbar()

plt.show()

sys.stdin.read(1)

x = 1
y = -4
a = 25 * x + 250
b = 25 * y + 250

print(t.shape)
print( t[ int( round( b ) ) ][ int( round( a ) ) ] )
print( a )
print( b )
print(t)
print( gradY[ int( round( b ) ) ][ int( round( a ) ) ] )

print("\n\n")

sys.stdin.read(1)

currentPoint = [0, 0, 0]

path = []
time = []

path.append( list(currentPoint) )

Coord_X = int( round( 25 * currentPoint[0] + 250 ) )
Coord_Y = int( round( 25 * currentPoint[1] + 250 ) )

travelTime = t[Coord_Y][Coord_X]

time.append( travelTime )

while( travelTime > 0.5 ):

    grad_x = gradX[Coord_Y][Coord_X]
    grad_y = gradY[Coord_Y][Coord_X]

    print(grad_x)
    print(grad_y)

    mod = math.sqrt( grad_x**2 + grad_y**2 )
    alpha = vectorOrientation( [ grad_x, grad_y ] )
    print(alpha * 180/math.pi)

    currentPoint[0] = currentPoint[0] + mod * math.cos(alpha)
    currentPoint[1] = currentPoint[1] + mod * math.sin(alpha)
    currentPoint[2] = alpha

    print( currentPoint[0] )
    print( currentPoint[1] )
    print( currentPoint[2] )
    print("\n\n")

    path.append( list(currentPoint) )

    Coord_X = int( round( 25 * currentPoint[0] + 250 ) )
    Coord_Y = int( round( 25 * currentPoint[1] + 250 ) )

    print(Coord_X)
    print(Coord_Y)

    travelTime = t[Coord_Y][Coord_X]

    time.append( travelTime )

pathToPlot = np.array(path)

cmap = cm.get_cmap('jet')
plt.contour(X, Y, phi,[0], linewidths=(3), colors = 'black')
plt.contour(X, Y, phi.mask, [0], linewidths=(0.5), colors = 'red')
plt.contourf(X, Y, t, 100, cmap = cmap)
plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
plt.gca().xaxis.set_major_locator( plt.NullLocator() )
plt.gca().yaxis.set_major_locator( plt.NullLocator() )
plt.plot( pathToPlot[ : , 0], pathToPlot[ : , 1], color = 'black', linestyle = 'solid' )
plt.show()

sys.stdin.read(1)

correctedArray = time - time[-1]

time = list( correctedArray[::-1] )

traj = trajectory.Trajectory(times = time, milestones = path)

endTime = traj.endTime()

parameterizedTraj = []

i = 0

while(i <= endTime):
    
    parameterizedTraj.append( list( traj.eval(i) ) )

    i += 0.1

i = 0

derivatives_vx = []
derivatives_wz = []

while(i <= endTime):
    
    dt = traj.deriv(i)
    derivatives_vx.append( math.sqrt( dt[0]**2 + dt[1]**2 ) )
    derivatives_wz.append( dt[2] )

    i += 0.1

abc = np.array(parameterizedTraj)

plt.figure(1)
plt.title("Theta")
plt.plot( range(len(path)), pathToPlot[:, 2] * 180 / math.pi )

plt.figure(2)
plt.title("Angular Velocity")
plt.plot( range(len(derivatives_wz)), derivatives_wz )

plt.figure(3)
plt.title("Theta")
plt.plot( range(len(parameterizedTraj)), abc[ : , 2] * 180 / math.pi )

plt.figure(4)
plt.title("Linear Velocity")
plt.plot( range(len(derivatives_vx)), derivatives_vx )

plt.show()