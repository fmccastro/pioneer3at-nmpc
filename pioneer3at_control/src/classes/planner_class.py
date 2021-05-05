#!/usr/bin/python3.8
from __future__ import absolute_import
from __future__ import division

import math, os, sys, matplotlib, skfmm

import numpy as np
import cv2 as cv
import matplotlib.cm as cm
import matplotlib.pyplot as plt

from klampt.model import trajectory
from scipy import interpolate

os.chdir("/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_gazebo/models")

class Planner:
    def __init__( self, Path, NbStates, NbControls, Image, costMap, heightPx, paramVel, paramVelFM, length, option, samplingTime, goal ):
    
        """
            option: 0 -> Path
                    1 -> Trajectory
                    2 -> Fast Marching points
        """

        self.path = Path

        self.Nx = NbStates
        self.Nu = NbControls

        self.img = cv.flip( Image, 0 ) * heightPx
        self.X, self.Y = np.meshgrid( np.linspace( -length/2, length/2, Image.shape[0] ), np.linspace( -length/2, length/2, Image.shape[1] ) )
        
        self.heightProportion = heightPx
        self.parameterVel = paramVel
        self.parameterVel_FM = paramVelFM
        self.option = option
        self.squareDim = length
        self.Ts = samplingTime
        self.goal = goal

        if( self.option == 2 ):

            self.img_costMap = cv.flip( costMap, 0 ) * heightPx
            self.X_costMap, self.Y_costMap = np.meshgrid( np.linspace( -length/2, length/2, costMap.shape[0] ), np.linspace( -length/2, length/2, costMap.shape[1] ) )

            self.phi = ( self.X_costMap - self.goal[0] )**2 + ( self.Y_costMap - self.goal[1] )**2 - 0.01
            self.speed = self.img_costMap
            self.t = skfmm.travel_time( self.phi, self.speed, dx = self.squareDim/self.img_costMap.shape[0] )

    def _fastMarching( self ):

        grad_x = -cv.Sobel( self.t, cv.CV_64F, 1, 0, ksize = 5 )
        grad_y = -cv.Sobel( self.t, cv.CV_64F, 0, 1, ksize = 5 )

        grad_x = np.ma.masked_where( grad_x == 0.0, grad_x )
        grad_y = np.ma.masked_where( grad_y == 0.0, grad_y )

        x = np.arange( -self.squareDim/2, self.squareDim/2, self.squareDim/self.img_costMap.shape[0] )
        y = np.arange( -self.squareDim/2, self.squareDim/2, self.squareDim/self.img_costMap.shape[0] )

        f_x = interpolate.interp2d( x, y, grad_x, kind = 'cubic' )
        f_y = interpolate.interp2d( x, y, grad_y, kind = 'cubic' )

        return f_x, f_y
    
    def _computeFM_Path( self, gradX, gradY, x_0, y_0, nbIntervals ):

        ref = []

        p_x = x_0
        p_y = y_0

        idx = 0

        while( idx < nbIntervals ):

            mod = math.sqrt( math.pow( gradX( p_x, p_y ), 2 ) + math.pow( gradY( p_x, p_y ), 2 ) )
            alpha = math.atan2( gradY( p_x, p_y ), gradX( p_x, p_y ) )

            p_x = p_x + self.parameterVel_FM * math.cos( alpha )
            p_y = p_y + self.parameterVel_FM * math.sin( alpha )
        
            ref += [ [ p_x, p_y, alpha ] ]

            idx += 1
        
        ref = np.array( ref )

        return ref

    def _showMap_FM( self, gradX, gradY, x_0, y_0 ):

        posX = []
        posY = []
        angles = []

        p_x = x_0
        p_y = y_0

        posX += [p_x]
        posY += [p_y]

        while( math.sqrt( ( p_x - self.goal[0] )**2 + ( p_y - self.goal[1] )**2 ) > 0.25 ):

            mod = math.sqrt( math.pow( gradX( p_x, p_y ), 2 ) + math.pow( gradY( p_x, p_y ), 2 ) )
            alpha = math.atan2( gradY( p_x, p_y ), gradX( p_x, p_y ) )
            angles += [alpha]

            p_x = p_x + self.parameterVel_FM * math.cos( alpha )
            p_y = p_y + self.parameterVel_FM * math.sin( alpha )

            posX += [p_x]
            posY += [p_y]
        
        cmap = cm.get_cmap( 'jet' )
        plt.title('Travel time from the boundary')
        plt.contour( self.X_costMap, self.Y_costMap, self.phi,[0], linewidths = (3), colors = 'white' )
        plt.gca().patch.set_color('.0')
        plt.contourf( self.X_costMap, self.Y_costMap, self.t, 100, cmap = cmap )

        for i in range( len(posX) - 1 ):

            plt.arrow( posX[ i ], posY[ i ], math.cos( angles[ i ] ) / 100, math.sin( angles[ i ] ) / 100, head_width = 0.01, head_length = 0.01, fc = 'k', ec = 'k' )

        plt.plot( posX, posY, color = 'black' )
        plt.gca().xaxis.set_major_locator( plt.NullLocator() )
        plt.gca().yaxis.set_major_locator( plt.NullLocator() )
        plt.colorbar()
        plt.show()
    
    def _pathOrientation( self, points ):

        directions = np.array( [ points[ 1, : ] - points[ 0, : ] ] )

        for i in range( 2, points.shape[0] ):
        
            directions = np.append( directions, np.array( [ points[ i, : ] - points[ i - 1, : ] ] ), axis = 0 )

        headings = np.append( np.array( [ [ 0 ] ] ), [ np.arctan2( directions[:, 1], directions[:, 0] ) ] )

        headings = headings.reshape( directions.shape[0] + 1, 1 )

        return headings

    def _pathSlicing( self ):

        self.path = np.array( self.path ) * ( self.squareDim / self.img.shape[0] ) - self.squareDim/2

        self.path = self.path.tolist()

        if( self.option == 1 ):

            traj = trajectory.HermiteTrajectory( milestones = self.path )

            traj_timed = trajectory.path_to_trajectory( traj, velocities = 'constant', timing = 'sqrt-L2', speed = 0.5 )

            endTime = traj_timed.endTime()

            t = 0

            path = []

            while( t <= endTime ):

                points = traj_timed.eval( t )

                path.append( list( points[ :2 ] ) )

                t += self.Ts
            
            pathPoints = np.array( path )

            pathPoints = np.hstack( ( pathPoints, self._pathOrientation( pathPoints ) ) )

            pathPoints = pathPoints.tolist()

            traj = trajectory.HermiteTrajectory( milestones = pathPoints )

            traj_timed = trajectory.path_to_trajectory( traj, velocities = 'constant', timing = 'sqrt-L2', speed = 1.0, dt = 0.001 )

            pathPoints = np.array( pathPoints )

            return pathPoints, traj_timed

        elif( self.option == 0 ):

            path = np.array( self.path )
            
            x = path[ :, 0 ]
            y = path[ :, 1 ]

            tck, u = interpolate.splprep( [ x, y ], s = 0 )

            unew = np.arange( 0, 1.0, 0.000001 )
            out = interpolate.splev( unew, tck )

            index = 0
            distance = 0

            while( index < out[0].shape[0] - 1 ):
                distance += math.sqrt( math.pow( out[ 0 ][ index + 1 ] - out[ 0 ][ index ], 2 ) + math.pow( out[ 1 ][ index + 1 ] - out[ 1 ][ index ], 2 ) )
                index += 1
            
            intervals = distance / ( self.parameterVel )
            
            unew = np.arange( 0, 1.0, 1.0 / intervals )
            out = interpolate.splev( unew, tck )

            i = 0

            path = []

            while( i < out[0].shape[0] ):

                path.append( [ out[0][i], out[1][i] ] )

                i += 1
            
            pathPoints = np.array( path )

            return pathPoints
    
    def _distanceBtwPoints( self, point1, point2 ):

        """

            #   Inputs:
                    point1: receives the robot's pose
                    point2: it receives a waypoint from the path
                    poseType: it sets point1 message type

            #   Output:
                    Distance between two points

        """

        return math.sqrt( ( point1.x - point2[0] )**2 + ( point1.y - point2[1] )**2 )
    
    def _lookAheadPoint( self, path, lastLookAheadPoint, lastFracIndex, pose, radius, maxLoopCycles ):

        index = lastLookAheadPoint
        count = 0

        while( index + 1 < path.shape[ 0 ] and count < maxLoopCycles ):
            
            d = np.array( [ path[ index + 1, 0 ], path[ index + 1, 1 ] ] ) - np.array( [ path[ index, 0 ], path[ index, 1 ] ] )
            f = np.array( [ path[ index, 0 ], path[ index, 1 ] ] ) - np.array( [ pose.x, pose.y ] )

            a = np.dot( d, d )
            b = 2 * np.dot( f, d )
            c = np.dot( f, f ) - math.pow( radius, 2 )

            discriminant = math.pow( b, 2 ) - 4 * a * c

            if( discriminant < 0 ):
                #   No intersection
                pass
            
            else:
                discriminant = math.sqrt( discriminant )

                t1 = ( -b - discriminant ) / ( 2 * a )
                t2 = ( -b + discriminant ) / ( 2 * a )

                if( t1 >= 0 and t1 <= 1 ):
                    return index + 1, index + t1
                
                if( t2 >= 0 and t2 <= 1 ):
                    return index + 1, index + t2
  
            index += 1
            count += 1

        return lastLookAheadPoint, lastFracIndex
        
    def _showPath( self, points ):

        checkpoints = np.array( self.path )

        plt.figure()

        cmap = cm.get_cmap( 'jet' )
        plt.plot( points[:, 0], points[:, 1], color = "white" )
        plt.plot( points[:, 0], points[:, 1], 'rx', markersize = 4 )
        plt.plot( checkpoints[:, 0], checkpoints[:, 1], 'ro' )
        plt.contourf( self.X, self.Y, self.img, 100, cmap = cmap )
        plt.subplots_adjust( top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0 )
        plt.gca().xaxis.set_major_locator( plt.NullLocator() )
        plt.gca().yaxis.set_major_locator( plt.NullLocator() )
        plt.colorbar( )
        plt.show()
        plt.close()

        plt.figure()

        cmap = cm.get_cmap( 'jet' )
        plt.plot( points[:, 0], points[:, 1], color = "white" )

        for i in range( points.shape[0] ):
            plt.arrow( points[i, 0], points[i, 1], math.cos( points[i, 2] ), math.sin( points[i, 2] ),
                        head_width = 0.05, head_length = 0.1, fc = 'k', ec = 'k' )

        plt.contourf( self.X, self.Y, self.img, 100, cmap = cmap )
        plt.subplots_adjust( top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0 )
        plt.gca().xaxis.set_major_locator( plt.NullLocator() )
        plt.gca().yaxis.set_major_locator( plt.NullLocator() )
        plt.show()
        plt.close()

        plt.figure()

        plt.plot( points[:, 2] )
        plt.show()