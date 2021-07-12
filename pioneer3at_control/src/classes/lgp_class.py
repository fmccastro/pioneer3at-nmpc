#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

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

@dataclass
class localModel:

    """
        outputID:   
                    0   ->   x   
                    1   ->   y
                    2   ->   theta
    """

    __slots__ = [ 'index', 'outputID', 'center', 'inputs', 'outputs', 'predictionVector', 'cholesky' ]

    index: int
    outputID: int
    center: np.ndarray
    inputs: np.ndarray
    outputs: np.ndarray
    predictionVector: np.ndarray
    cholesky: np.ndarray

@dataclass
class LGP_Record:

    localModelsIndex: List[ localModel ] = field( default_factory = list )

    def __repr__( self ):
        localModelsIndex = ', '.join(f'{c!s}' for c in self.localModelsIndex)
        return f'{self.__class__.__name__}({localModelsIndex})'

class LGP:
    def __init__( self, common ):

        self.pointsCollected = 0

        #   For function calls
        self.funCommon = common

        self._N = common.N
        self._dt = common.Ts

        self.__shortestAngle = common._shortestAngle

        self.nbStates = common.NbStates
        self.nbControls = common.NbControls

        self.inputDim = common.nbInputs
        self.outputDim = common.nbOutputs

        self.inputFlag = True
        self.outFlag = True

        self._rawInputData = None
        self._rawOutputData = None

        self.threshold = common.limitValue
        self.maxNbDataPoints = common.maxDataPts
        self.maxNbLocalModels = common.maxLocalModels

        weights = []
        self.kernel = []
        self.gpModel = []
        self.localModelsNb = []
        self.localGPModels = []

        for _ in range( self.inputDim ):
            weights.append( 1.0 )

        for _ in range( self.outputDim ):
            self.localModelsNb += [ 0 ]
            self.localGPModels += [ LGP_Record() ]
            #self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) + gpflow.kernels.White() ]
            self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) ]

    def _getOrientation( self, pose ):

        return np.array( [ [ pose.roll, pose.pitch ] ] )
    
    def _getVelocity( self, currentPose, previousPose, timeDifference ):

        return np.array( [ [ math.sqrt( math.pow( currentPose.x - previousPose.x, 2 ) + math.pow( currentPose.y - previousPose.y, 2 ) )\
                                                                / timeDifference, ( currentPose.yaw - previousPose.yaw ) / timeDifference ] ] )

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
    
    def _saveTrainingData( self, filenameInput, filenameOutput ):

        rawInput = np.load( filenameInput )
        rawOutput = np.load( filenameOutput )

        self._rawInputData = rawInput
        self._rawOutputData = rawOutput
    
    def _loopLocalModels( self, input, state ): # OK

        """
            inputs:
                    input:  1*N array (numpy)
                    index: scalar selecting the state
            
            output:
                    wk: list with the kernel variance of k( diff, diff ) for each local model
        """

        wk = []

        for j in range( self.localModelsNb[state] ):

            wk += [ self.funCommon._kernel( self.gpModel[state].kernel.lengthscales.numpy(),\
                                            self.gpModel[state].likelihood.variance.numpy(), input, self.localGPModels[state].localModelsIndex[j].center ) ]

        return wk

    def _partitioning( self, input, output ):

        """
            inputs:
                    input: 1 * self.inputDimension (numpy)
                    output: 1 * self.outputDimension (numpy)
        """

        for j in range( self.outputDim ):

            if( self.localModelsNb[j] == 0 ):

                k_new_scl = self.funCommon._kernel( self.gpModel[j].kernel.lengthscales.numpy(),\
                                                    self.gpModel[j].likelihood.variance.numpy(), input, input )

                #gram = np.array( k_new_scl ) + np.array( [ [ self.kernel[j].kernels[1].variance.numpy() ] ] )
                gram = np.array( [ [ k_new_scl ] ] ) + np.array( [ [ self.gpModel[j].likelihood.variance.numpy() ] ] )

                alpha = np.linalg.inv( gram ) @ output[ 0, j ].reshape( 1, 1 )

                ch = np.sqrt( gram )

                newlocalGP = localModel( self.localModelsNb[j], j, input, input, output[ 0, j ].reshape( 1, 1 ), alpha, ch )
                self.localGPModels[j].localModelsIndex += [ newlocalGP ]
                self.localModelsNb[j] += 1

            else:

                distances = self._loopLocalModels( input, j )

                nearestLM = max( distances )

                maxIndex = distances.index( nearestLM )

                if( nearestLM > self.threshold[j] ):

                    _inputs = self.localGPModels[j].localModelsIndex[maxIndex].inputs

                    for i in range( _inputs.shape[0] ):

                        if( i == 0 ):
                            K_new = np.array( [ [ self.funCommon._kernel( self.gpModel[j].kernel.lengthscales.numpy(),\
                                                                          self.gpModel[j].likelihood.variance.numpy(),\
                                                                                input, _inputs[i, :].reshape(1, -1) ) ] ] )

                        elif( i > 0 ):
                            K_new = np.vstack( ( K_new, self.funCommon._kernel( self.gpModel[j].kernel.lengthscales.numpy(),\
                                                                                              self.gpModel[j].likelihood.variance.numpy(),\
                                                                                                    input, _inputs[i, :].reshape(1, -1) ) ) )
                    
                    self.localGPModels[j].localModelsIndex[maxIndex].inputs = np.vstack( ( self.localGPModels[j].localModelsIndex[maxIndex].inputs, input ) )

                    self.localGPModels[j].localModelsIndex[maxIndex].outputs = np.vstack( ( self.localGPModels[j].localModelsIndex[maxIndex].outputs,\
                                                                                                                            output[ 0, j ].reshape(1, 1) ) )

                    self.localGPModels[j].localModelsIndex[maxIndex].center = np.mean( self.localGPModels[j].localModelsIndex[maxIndex].inputs,\
                                                                                                                axis = 0 ).reshape( 1, self.inputDim )

                    k_new_scl = self.funCommon._kernel( self.gpModel[j].kernel.lengthscales.numpy(),\
                                                        self.gpModel[j].likelihood.variance.numpy(), input, input )

                    #kernel_var = self.kernel[j].kernels[1].variance.numpy()
                    model_var = self.gpModel[j].likelihood.variance.numpy()

                    cholesky = self.localGPModels[j].localModelsIndex[maxIndex].cholesky

                    l = self._forwardSubs( cholesky, K_new )

                    self.localGPModels[j].localModelsIndex[maxIndex].cholesky = self._updateCholesky( cholesky, model_var, k_new_scl, l )

                    if( self.localGPModels[j].localModelsIndex[maxIndex].inputs.shape[0] > self.maxNbDataPoints ):

                        self.localGPModels[j].localModelsIndex[maxIndex].inputs = np.delete( self.localGPModels[j].localModelsIndex[maxIndex].inputs, 0, 0 )
                        self.localGPModels[j].localModelsIndex[maxIndex].outputs = np.delete( self.localGPModels[j].localModelsIndex[maxIndex].outputs, 0, 0 )

                        self.localGPModels[j].localModelsIndex[maxIndex].center = np.mean( self.localGPModels[j].localModelsIndex[maxIndex].inputs,\
                                                                                                                axis = 0 ).reshape( 1, self.inputDim )
                        
                        ch = self.localGPModels[j].localModelsIndex[maxIndex].cholesky
                        
                        self.localGPModels[j].localModelsIndex[maxIndex].cholesky = ( self._rankOneUpdate( ch[ 1:, 1: ].T, ch[ 1:, 0 ] ) ).T

                    ch = self.localGPModels[j].localModelsIndex[maxIndex].cholesky

                    newOutput = self.localGPModels[j].localModelsIndex[maxIndex].outputs

                    self.localGPModels[j].localModelsIndex[maxIndex].predictionVector = self._predictionVector( ch, newOutput )

                else:

                    k_new_scl = self.funCommon._kernel( self.gpModel[j].kernel.lengthscales.numpy(),\
                                                        self.gpModel[j].likelihood.variance.numpy(), input, input )

                    #gram = np.array( k_new_scl ) + np.array( [ [ self.kernel[j].kernels[1].variance.numpy() ] ] )
                    gram = np.array( [ [ k_new_scl ] ] ) + np.array( [ [ self.gpModel[j].likelihood.variance.numpy() ] ] )

                    alpha = np.linalg.inv( gram ) @ output[ 0, j ].reshape(1, 1)

                    ch = np.sqrt( gram )

                    newlocalGP = localModel( self.localModelsNb[j], j, input, input, output[0, j].reshape(1, 1), alpha, ch )
                    self.localGPModels[j].localModelsIndex += [ newlocalGP ]
                    self.localModelsNb[j] += 1

        self.pointsCollected += 1
    
    def _buildInitialLocalModels( self ): # OK

        """
            Description:    Access each row from both matrices at the same time, and input them in the 
                            self._partitioning function. Each row is reshaped to 1 * N (numpy).
        """

        index = 0

        for x, y in zip( self._rawInputData, self._rawOutputData ):
            self._partitioning( x.reshape(1, x.shape[0] ), y.reshape(1, y.shape[0] ) )

            print("Index: ", index)

            index += 1

        self._printLocalModel( 0 )
        self._printLocalModel( 1 )
        self._printLocalModel( 2 )

        self._printSummary()

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

    def _nearestModels( self, state, predInput ):
        
        """
            Find nearest models for query input

            Inputs:
                    state -> output state to look for nearest models
                    predInput -> query data point

            Output:
                    result -> ordered list with distance from the local model center to the query by ascending order                        
        """
        
        predInput = np.array( predInput ).reshape(1, -1)

        for i in range( self.localModelsNb[state] ):

            if( i == 0 ):
                full = np.array( [ [ self.funCommon._kernel( self.gpModel[state].kernel.lengthscales.numpy(),\
                                                             self.gpModel[state].likelihood.variance.numpy(),\
                                                                predInput, self.localGPModels[state].localModelsIndex[i].center ), i ] ] )

            elif( i == 1 ):
                full = np.vstack( ( full, np.array( [ [ self.funCommon._kernel( self.gpModel[state].kernel.lengthscales.numpy(),\
                                                                                self.gpModel[state].likelihood.variance.numpy(),\
                                                                                    predInput, self.localGPModels[state].localModelsIndex[i].center ), i ] ] ) ) )

        #   Higher values mean a smaller distance from the prediction input to the respective local model center
        full = full[ ( -full[ :, 0 ] ).argsort() ]

        full = full[ :self.maxNbLocalModels, : ]

        return full

    def _prediction( self, state, test ):

        """
            Make prediction for a given "state" with query point "test"
        """

        models = self._nearestModels( state, test )

        for i in range( models.shape[0] ):
            
            _inputs = self.localGPModels[state].localModelsIndex[ models[i, 1] ].inputs

            for j in range( _inputs.shape[0] ):

                if( j == 0 ):
                    k = np.array( self.funCommon._kernel( self.gpModel[state].kernel.lengthscales.numpy(),\
                                                          self.gpModel[state].likelihood.variance.numpy(),\
                                                                test, _inputs[j, :].reshape(1, -1) ) ).reshape(1, -1)

                elif( j > 0 ):
                    k = np.hstack( ( k, np.array( self.funCommon._kernel( self.gpModel[state].kernel.lengthscales.numpy(),\
                                                                          self.gpModel[state].likelihood.variance.numpy(),\
                                                                                test, _inputs[j, :].reshape(1, -1) ) ).reshape(1, -1) ) )

            if( i == 0 ):
                y = k @ self.localGPModels[state].localModelsIndex[ models[i, 1] ].predictionVector.reshape(-1, 1)
                w = models[i, 0]
            
            elif( i > 0 ):
                y = np.vstack( ( y, k @ self.localGPModels[state].localModelsIndex[ models[i, 1] ].predictionVector.reshape(-1, 1) ) )
                w = np.hstack( ( w, models[i, 0] ) )
        
        output = np.dot( w, y ) / np.sum(w)

        return output
    
    def _fullPrediction( self, predInputs ):

        for j in range( predInputs.shape[0] ):

            for i in range( self.outputDim ):

                if( i == 0 ):
                    output = np.array( [ [ self._prediction( i, predInputs[j, :].reshape(1, -1) ) ] ] )
                
                elif( i > 0 ):
                    output = np.hstack( ( output, self._prediction( i, predInputs[j, :].reshape(1, -1) ) ) )

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
                    
                    vel_x = self.funCommon._distanceBtwPoints( [ seqPose[j], seqPose[j + 1] ],\
                                [ seqPose[j - self.nbStates ], seqPose[j - self.nbStates  + 1] ], 2 ) / self._dt
                    w_z = self.funCommon._shortestAngle( seqPose[j - self.nbStates + 2], seqPose[ j + 2 ] ) / self._dt

                    inputs = np.vstack( ( inputs,\
                                np.array( [ [ seqControl[ i + self.nbControls ], seqControl[ i + self.nbControls + 1 ], seqControl[ i ], seqControl[ i + 1 ], vel_x, w_z ] ] ) ) )
                    
                elif( i == len(seqControl) - self.nbControls and j == len(seqPose) - self.nbStates ):
                    
                    vel_x = self.funCommon._distanceBtwPoints( [ seqPose[j], seqPose[j + 1] ],\
                                    [ seqPose[j - self.nbStates ], seqPose[j - self.nbStates  + 1] ], 2 ) / self._dt
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
    
    def _printLocalModel( self, state ):

        print("\n")
        print("|*****************************************************************************|")

        for index in range( self.localModelsNb[state] ):

            print( "| Index: ", self.localGPModels[state].localModelsIndex[index].index )
            print( "| OutputID: ", self.localGPModels[state].localModelsIndex[index].outputID )
            print( "| Center: ", self.localGPModels[state].localModelsIndex[index].center )
            print( "| Inputs: ", self.localGPModels[state].localModelsIndex[index].inputs )
            print( "| Outputs: ", self.localGPModels[state].localModelsIndex[index].outputs )
            print( "| Prediction Vector: ", self.localGPModels[state].localModelsIndex[index].predictionVector )
            print( "| Cholesky: ", self.localGPModels[state].localModelsIndex[index].cholesky )
            print("\n")
    
        print("|*****************************************************************************|")
        print("\n")
    
    def _printSummary( self ):

        print("\n")
        print("|*****************************************************************************|")

        for index in range( self.outputDim ):
            print("| Local Model " + str(index) + ": " + str( self.localModelsNb[index] ) )
        
        print("|*****************************************************************************|")
        print("\n")

    def _loadModel( self ):

        """
            Return local models and number of local models for each state
        """

        return self.localModelsNb, self.localGPModels
    
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
    
    def _loadPickle( self, filename ):

        pickle_in = open( filename, "rb" )

        output1 = pickle.load( pickle_in )

        index = 0

        for element in output1:
            self.kernel[index].variance.assign( element[0] )
            self.kernel[index].lengthscales.assign( element[1] )

            if( element[2] == 1e-6 ):
                element[2] += 1e-20

            self.gpModel += [ gpflow.models.GPR( data = ( self._rawInputData, self._rawOutputData[ :, index ].reshape( -1, 1 ) ),\
                                                                kernel = self.kernel[index], mean_function = None, noise_variance = element[2] ) ]

            #print_summary( self.kernel[index] )
            print_summary( self.gpModel[index] )

            index += 1