#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division
from cv2 import data

import gpflow, sys, math, pprint, time, pickle

import matplotlib.pyplot as plt

import numpy as np
from numpy.core.shape_base import vstack
from numpy.lib.function_base import disp
import tensorflow as tf

from multiprocessing import Queue
from dataclasses import dataclass, field
from typing import List
from gpflow.utilities import print_summary
from casadi import*

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

"""
    Local Gaussian Process (LGP)
"""

class LGP:
    def __init__( self, common ):

        self.pointsCollected = 0

        #   For function calls
        self.funCommon = common

        self._N = common.N
        self._dt = common.Ts

        self.linVel_max = common.U_ub[0]

        self.nbStates = common.NbStates
        self.nbControls = common.NbControls

        self.inputDim = common.nbInputs
        self.outputDim = common.nbOutputs

        self.inputFlag = True
        self.outFlag = True

        self._rawInputData = None
        self._rawOutputData = None

        self.maxNbDataPoints = common.maxDataPts

        weights = []
        self.kernel = []
        self.gpModel = []

        for _ in range( self.inputDim ):
            weights.append( 1.0 )

        self.nbBins = 0

        self.bin = {}
        self.predVector = {}

        self.angleDiscretization = common.angleDiscretization
        self.velDiscretization = common.velDiscretization

        self.cRoll = common.c_roll
        self.cPitch = common.c_pitch
        self.cVel = common.c_vel

        self.cutVar = common.cutVar

        """
            outputID:   
                        0   ->   x   
                        1   ->   y
                        2   ->   theta
        """

        for i in range( self.outputDim ):
            self.predVector[ str(i) ] = None
            #self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) + gpflow.kernels.White() ]
            self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) ]

        for j in range( common.angleDiscretization + 1 ):
            for l in range( -common.angleDiscretization, common.angleDiscretization + 1 ):
                for k in range( -common.velDiscretization, common.velDiscretization + 1 ):
                    self.bin[ str(j) + "_" + str(l) + "_" + str(k) ] = None

    def _getOrientation( self, pose ):

        return np.array( [ [ pose.roll, pose.pitch ] ] )
    
    def _getVelocity( self, currentPose, previousPose, timeDifference ):

        return np.array( [ [ math.sqrt( math.pow( currentPose.x - previousPose.x, 2 ) + math.pow( currentPose.y - previousPose.y, 2 ) ) / timeDifference,\
                             self.funCommon._shortestAngle( previousPose.yaw, currentPose.yaw ) / timeDifference ] ] )

    def _getControls( self, actuation ):

        return np.array( [ [ actuation.linear.x, actuation.angular.z ] ] )
    
    def _updateRawInData( self, input ):

        if( self.inputFlag ):
            self._rawInputData = input
            self.inputFlag = False

        else:
            self._rawInputData = np.vstack( ( self._rawInputData, input ) )

    def _updateRawOutData( self, output ):

        if( self.outFlag ):
            self._rawOutputData = output
            self.outFlag = False
        
        else:
            self._rawOutputData = np.vstack( ( self._rawOutputData, output ) )
    
    def _trainingData( self ):

        return self._rawInputData, self._rawOutputData
    
    def _saveTrainingData( self, rawInput, rawOutput ):

        self._rawInputData = rawInput
        self._rawOutputData = rawOutput

    def _partitioning( self, input, output ):

        """
            inputs:
                    input: 1 * self.inputDimension (numpy)
                    output: 1 * self.outputDimension (numpy)
        """

        roll = abs( input[0, 0] ) * ( self.angleDiscretization / math.pi )
        pitch = input[0, 1] * ( self.angleDiscretization / math.pi )
        vel = input[0, 4] * ( self.velDiscretization / self.linVel_max ) 

        dataInput = input[0, 2:].reshape(1, -1)
        index = str( int( round(roll) ) ) + "_" + str( int( round(pitch) ) ) + "_" + str( int( round(vel) ) )

        if( self.bin[ index ] == None ):
            self.bin[ index ] = { 'center': dataInput, 'inputs': dataInput, 'outputs': output }
            self.nbBins += 1

        else:
            self.bin[ index ][ 'inputs' ] = np.vstack( ( self.bin[ index ][ 'inputs' ], dataInput ) )
            self.bin[ index ][ 'outputs' ] = np.vstack( ( self.bin[ index ][ 'outputs'], output ) )
            self.bin[ index ][ 'center' ] = np.mean( self.bin[ index ][ 'inputs' ], axis = 0 ).reshape(1, -1)

            if( self.bin[ index ][ 'inputs' ].shape[0] > self.maxNbDataPoints ):

                self.bin[ index ][ 'inputs' ] = np.delete( self.bin[ index ][ 'inputs' ], 0, 0 )
                self.bin[ index ][ 'outputs' ] = np.delete( self.bin[ index ][ 'outputs' ], 0, 0 )

                self.bin[ index ][ 'center' ] = np.mean( self.bin[ index ][ 'inputs' ], axis = 0 ).reshape( 1, self.inputDim )

        self.pointsCollected += 1
    
    def _buildInitialLocalModels( self ): # OK

        """
            Description:    Access each row from both matrices at the same time, and input them in the 
                            self._partitioning function. Each row is reshaped to 1 * N (numpy).
        """

        index = 0

        for x, y in zip( self._rawInputData, self._rawOutputData ):
            self._partitioning( x.reshape(1, -1), y.reshape(1, -1) )
            print("Index: ", index)

            index += 1

        self._printBins()
        self._printInfo()

    def _forwardSubs( self, A, B ): # OK

        """
            inputs:
                    A -> N*N lower triangular matrix (numpy)
                    B -> N*1 array (numpy)
            
            output:
                    X -> solution of AX = B by forward substitution, X is a N*1 array (numpy)
        """

        X = []

        for i in range( A.shape[0] ):
            X += [ B[i, 0] ]

            for j in range( i ):
                X[i] = X[i] - A[i, j] * X[j]
        
            X[i] = X[i] / A[i, i]

        X = np.array( X ).reshape( len(X), 1 )

        return X
    
    def _backSubs( self, A, B ): # OK

        """
            inputs:
                    A -> N*N upper triangular matrix (numpy)
                    B -> N*1 array (numpy)
            
            output:
                    X -> solution of AX = B by back substitution, X is a N*1 array (numpy)
        """

        X = np.ones( ( A.shape[0], 1 ) )

        for i in reversed( range( A.shape[0] ) ):
            X[i, 0] = B[ i, 0 ]

            for j in range( i + 1, A.shape[1] ):
                if( j >= A.shape[1] ):
                    break

                X[ i, 0 ] = X[ i, 0 ] - A[ i, j ] * X[ j, 0 ]
            
            X[ i, 0 ] = X[ i, 0 ] / A[ i, i ]

        return X
    
    def _updateCholesky( self, prevCholesky, variance, k_new, l ): # OK

        """
            inputs:
                    prevCholesky (previous Cholesky) -> N*N matrix (numpy)
                    variance -> kernel variance scalar
                    k_new ( k( x, x ) ) -> variance scalar
                    l -> N*1 matrix (numpy)
            
            output:
                    L_new (updated Cholesky matrix) -> ( N + 1 )*( N + 1 ) matrix (numpy)
        """

        #print("Variance: ", variance )
        #print( k_new + variance - math.pow( np.linalg.norm( l ), 2 ) )
        L_star = math.sqrt( k_new + variance - math.pow( np.linalg.norm( l ), 2 ) )
        L_new = np.block( [ [ prevCholesky, np.zeros( ( prevCholesky.shape[0], 1 ) ) ], [ np.transpose( l ), L_star ] ] )

        return L_new
    
    def _predictionVector( self, cholesky, output ): # OK

        """
            inputs:
                    cholesky: N*N matrix (numpy)
                    output: N*1 matrix (numpy)

            output:
                    X: N*1 matrix (numpy)

                    cholesky * X = output <=> X = cholesky.T * output
        """

        Y = self._forwardSubs( cholesky, output )
        X = self._backSubs( np.transpose( cholesky ), Y )

        return X

    def _assembleLocalModels( self, minPhi, maxPhi, minTheta, maxTheta, index, test = None ):
        
        """
            Make prediction for a given "state" with query point "test"
        """

        if( index == 0 ):
            velIndex = 0

        elif( index > 0 ):
            velIndex = test[0, 4] * ( self.velDiscretization / self.linVel_max )
            velIndex = int( round( velIndex ) )

        minVel = velIndex - self.cVel
        maxVel = velIndex + self.cVel

        if( minVel <= -self.velDiscretization ):
            minVel = -self.velDiscretization
        
        if( maxVel >= self.velDiscretization ):
            maxVel = self.velDiscretization

        ###
        funOutput = []

        for k in range( self.outputDim ):
            index = 0
            state_str = str(k)

            for i in range( minPhi, maxPhi + 1 ):
                for j in range( minTheta, maxTheta + 1 ):
                    for l in range( minVel, maxVel + 1 ):

                        if( self.bin[str(i) + "_" + str(j) + "_" + str(l)] != None ):
                            dataInput = self.bin[str(i) + "_" + str(j) + "_" + str(l)]['inputs']
                            dataOutput = self.bin[str(i) + "_" + str(j) + "_" + str(l)]['outputs']
                            
                            for p in range( dataInput.shape[0] ):
                                input  = dataInput[p, :].reshape(1, -1)
                                output = dataOutput[p, :].reshape(1, -1)

                                if( index == 0 ):
                                    _outputs = output
                                
                                elif( index > 0 ):
                                    _outputs = np.vstack( ( _outputs, output ) )

                                if( self.predVector[state_str] == None ):
                                    k_new_scl = self.kernel[k]( input, input ).numpy()

                                    #gram = np.array( k_new_scl ) + np.array( [ [ self.kernel[j].kernels[1].variance.numpy() ] ] )
                                    gram = k_new_scl + np.array( [ [ self.gpModel[k].likelihood.variance.numpy() ] ] )

                                    alpha = np.linalg.inv( gram ) @ output[ 0, k ].reshape( 1, 1 )

                                    ch = np.sqrt( gram )

                                    self.predVector[state_str] = { 'predictionVector': alpha, 'cholesky': ch }
                                
                                else:
                                    K_new = self.kernel[k]( input, _inputs ).numpy()
                                    
                                    k_new_scl = self.kernel[k]( input, input ).numpy()[0, 0]

                                    #kernel_var = self.kernel[j].kernels[1].variance.numpy()
                                    model_var = self.gpModel[k].likelihood.variance.numpy()

                                    cholesky = self.predVector[state_str][ 'cholesky' ]

                                    l = self._forwardSubs( cholesky, K_new )

                                    self.predVector[state_str][ 'cholesky' ] = self._updateCholesky( cholesky, model_var, k_new_scl, l )

                                    ch = self.predVector[state_str][ 'cholesky' ]

                                    self.predVector[state_str][ 'predictionVector' ] = self._predictionVector( ch, _outputs[:, k] )
                                
                                if( index == 0 ):
                                    _inputs = input
                                
                                elif( index > 0 ):
                                    _inputs = np.vstack( ( _inputs, input ) )
                                
                                index += 1
            
            funOutput += [ [ self.predVector[state_str][ 'predictionVector' ], _inputs ] ]
        
        return funOutput
    
    def _fullPrediction( self, pose, iteration, predInputs = None ):

        indexRoll = abs( pose.roll ) * ( self.angleDiscretization / math.pi )
        indexPitch = pose.pitch * ( self.angleDiscretization / math.pi )

        indexRoll = int( round( indexRoll ) )
        indexPitch = int( round( indexPitch ) )

        minRoll = indexRoll - self.cRoll
        maxRoll = indexRoll + self.cRoll

        if( minRoll <= 0 ):
            minRoll = 0
        
        if( maxRoll >= self.angleDiscretization ):
            maxRoll = self.angleDiscretization

        minPitch = indexPitch - self.cPitch
        maxPitch = indexPitch + self.cPitch

        if( minPitch <= -self.angleDiscretization ):
            minPitch = -self.angleDiscretization
        
        if( maxPitch >= self.angleDiscretization ):
            maxPitch = self.angleDiscretization

        pv = []

        if( iteration == 0 ):
                pv = self._assembleLocalModels( minRoll, maxRoll, minPitch, maxPitch, iteration )
        
        elif( iteration > 0 ):
                pv = self._assembleLocalModels( minRoll, maxRoll, minPitch, maxPitch, iteration, test = predInputs )
        
        for j in range( predInputs.shape[0] ):

            for i in range( self.outputDim ):
                k = self.kernel[i]( predInputs[j, :].reshape(1, -1), pv[i, 1] ).numpy()

                if( i == 0 ):
                    output = k @ pv[i, 0]
                
                elif( i > 0 ):
                    output = np.hstack( ( k @ pv[i, 0], output ) )

            if( j == 0 ):
                fullOutput = output
    
            elif( j > 0 ):
                fullOutput = np.vstack( ( fullOutput, output ) )

        return fullOutput

    def _getPredictionInputs( self, iniPose, seqPose, seqControl, gpModel ):

        """
            Obtain prediction inputs for the selected prediction model, with respect to the solution from the previous NMPC optimization
        """
        
        #   { u_k, u_{k-1}, v_{k-1} }
        if( gpModel == 1 ):

            index = 0

            for i, j in zip( range( 0, len( seqControl ), self.nbControls ), range( 0, len( seqPose ), self.nbStates ) ):

                if( i == 0 and j == 0 ):
                    
                    vel_x = self.funCommon._distanceBtwPoints( [ seqPose[j], seqPose[j + 1] ], iniPose, 3 ) / self._dt
                    w_z = self.funCommon._shortestAngle( iniPose.yaw, seqPose[j + 2] ) / self._dt

                    inputs = np.array( [ [ seqControl[ i + self.nbControls ], seqControl[ i + self.nbControls + 1 ], seqControl[ i ], seqControl[ i + 1 ], vel_x, w_z ] ] )

                elif( i > 0 and j > 0 and i < len(seqControl) - self.nbControls and j < len(seqPose) - self.nbStates ):
                    
                    vel_x = self.funCommon._distanceBtwPoints( [ seqPose[j], seqPose[j + 1] ], [ seqPose[j - self.nbStates ], seqPose[j - self.nbStates  + 1] ], 2 ) / self._dt
                    w_z = self.funCommon._shortestAngle( seqPose[j - self.nbStates + 2], seqPose[ j + 2 ] ) / self._dt

                    inputs = np.vstack( ( inputs, np.array( [ [ seqControl[ i + self.nbControls ],\
                                                                seqControl[ i + self.nbControls + 1 ],\
                                                                seqControl[ i ], seqControl[ i + 1 ], vel_x, w_z ] ] ) ) )
                    
                elif( i == len(seqControl) - self.nbControls and j == len(seqPose) - self.nbStates ):
                    
                    vel_x = self.funCommon._distanceBtwPoints( [ seqPose[j], seqPose[j + 1] ],\
                                                               [ seqPose[j - self.nbStates],\
                                                                 seqPose[j - self.nbStates  + 1] ], 2 ) / self._dt
                    w_z = self.funCommon._shortestAngle( seqPose[j - self.nbStates + 2], seqPose[ j + 2 ] ) / self._dt

                    inputs = np.vstack( ( inputs, np.array( [ [ seqControl[ i ], seqControl[ i + 1 ], seqControl[ i ], seqControl[ i + 1 ], vel_x, w_z ] ] ) ) )

                index += 1

        return inputs

    def _rankOneUpdate( self, L, x ):

        """
            inputs:
                    L -> cholesky sub-matrix with dimension N*N
                    x -> array with dimension N

            outputs:
                    L_upd -> updated cholesky matrix with dimension N*N
        """

        N = x.shape[0]

        for i in range( N ):

            r = math.sqrt( math.pow( L[ i, i ], 2 ) + math.pow( x[ i ], 2 ) )
            c = r / L[ i, i ]
            s = x[ i ] / L[ i, i ]
            L[ i, i ] = r

            L[ i, i + 1:N ] = ( L[ i, i + 1:N ] + s * x[ i + 1:N ] ) / c
            x[ i + 1:N] = c * x[ i + 1:N ] - s * L[ i, i + 1:N ]

        return L
    
    def _pointsCollected( self ):
        return self.pointsCollected 
    
    def _printBins( self ):

        print("\n")
        print("|*****************************************************************************|")

        for key, value in self.bin.items():
            if( self.bin[ key ] != None ):
                print( "| Index: ", key )
                print( "| Center: ", self.bin[ key ][ 'center' ] )
                print( "| Inputs: ", self.bin[ key ][ 'inputs' ] )
                print( "| Outputs: ", self.bin[ key ][ 'outputs' ] )
                print("\n")
    
        print("|*****************************************************************************|")
        print("\n")
    
    def _printPredData( self ):

        print("\n")
        print("|*****************************************************************************|")

        for i in range( self.outputDim ):
            state_str = str(i)
            print( "| State: ", i )
            print( "| Prediction Vector: ", self.predVector[ state_str ][ 'predictionVector' ] )
            print( "| Cholesky: ", self.predVector[ state_str ][ 'cholesky' ] )
            print("\n")
            
        print("|*****************************************************************************|")
        print("\n")
    
    def _printInfo( self ):

        print("\n")
        print("|*****************************************************************************|")

        print("| Number of bins: ", self.nbBins )
        print("| Number of points collected: ", self.pointsCollected )

        print("|*****************************************************************************|")
        print("\n")

    def _loadModel( self ):

        """
            Return local models and number of local models for each state
        """

        return self.bin
    
    def _loadParameters( self ):

        """
            Load parameters of each model
        """

        parameters = []

        for index in range( self.outputDim ):
            parameters += [ [ self.gpModel[index].kernel.variance.numpy(),\
                              self.gpModel[index].kernel.lengthscales.numpy(),\
                              self.gpModel[index].likelihood.variance.numpy()\
                                                                                ] ]

        return parameters
    
    def _testKernel( self ):

        a = np.random.rand(1, 6)
        b = np.random.rand(1, 6)

        for j in range( self.outputDim ):

            print( self.funCommon._kernel( self.gpModel[j].kernel.lengthscales.numpy(),\
                                           self.gpModel[j].kernel.variance.numpy(), a, b ) )
            print( self.kernel[j]( a, b ).numpy() ) 
    
    def _loadPickle( self, kernelFilename, localModelsFilename, opt ):

        if( opt == 0 ):
            pickleKernel = open( kernelFilename, "rb" )
            outputKernel = pickle.load( pickleKernel )

            index = 0

            for element in outputKernel:
                self.kernel[index].variance.assign( element[0] )
                self.kernel[index].lengthscales.assign( element[1] )

                if( element[2] == 1e-6 ):
                    element[2] += 1e-20

                self.gpModel += [ gpflow.models.GPR( data = ( self._rawInputData, self._rawOutputData[ :, index ].reshape( -1, 1 ) ),\
                                                                    kernel = self.kernel[index], mean_function = None, noise_variance = element[2] ) ]

                #print_summary( self.kernel[index] )
                #print_summary( self.gpModel[index] )

                index += 1
        
        elif( opt == 1 ):
            pickleLGP = open( localModelsFilename, "rb" )

            outputLGP = pickle.load( pickleLGP )
            self.bin = outputLGP