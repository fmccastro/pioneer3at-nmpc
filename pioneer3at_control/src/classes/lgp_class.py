#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import gpflow, sys, math, pprint, time

import matplotlib.pyplot as plt

import numpy as np
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

    localModelsIndex: List[localModel] = field( default_factory = list )

    def __repr__( self ):
        localModelsIndex = ', '.join(f'{c!s}' for c in self.localModelsIndex)
        return f'{self.__class__.__name__}({localModelsIndex})'

class LGP:
    def __init__( self, inputDimension, outputDimension, limitValue, maxNbDataPts ):

        self.pointsCollected = 0

        self.inputDim = inputDimension
        self.outputDim = outputDimension

        self.inputFlag = True
        self.outFlag = True

        self.rawInputData = None
        self.rawOutputData = None

        self.threshold = limitValue

        self.maxNbDataPoints = maxNbDataPts

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

        return np.array( [ [ math.sqrt( math.pow( currentPose.x - previousPose.x, 2 ) + math.pow( currentPose.y - previousPose.y, 2 ) ) / timeDifference, ( currentPose.yaw - previousPose.yaw ) / timeDifference ] ] )

    def _getControls( self, actuation ):

        return np.array( [ [ actuation.linear.x, actuation.angular.z ] ] )
    
    def _updateRawInData( self, input ):

        if( self.inputFlag ):
            self.rawInputData = input
            self.inputFlag = False

        else:
            self.rawInputData = np.vstack( ( self.rawInputData, input ) )

    def _updateRawOutData( self, output ):

        if( self.outFlag ):
            self.rawOutputData = output
            self.outFlag = False
        
        else:
            self.rawOutputData = np.vstack( ( self.rawOutputData, output ) )
    
    def _trainingData( self ):

        return self.rawInputData, self.rawOutputData
    
    def _saveTrainingData( self, rawInput, rawOutput ):

        self._rawInputData = rawInput
        self._rawOutputData = rawOutput
    
    def _loopLocalModels( self, input, index ): # OK

        """
            inputs:
                    input:  1*N array (numpy)
                    index: scalar selecting the state
            
            output:
                    wk: list with the kernel variance of k( diff, diff ) for each local model
        """

        wk = []

        for j in range( self.localModelsNb[index] ):

            wk += [ self.kernel[index]( input, self.localGPModels[index].localModelsIndex[j].center ).numpy()[0, 0] ]

        return wk

    def _partitioning( self, input, output ):

        """
            inputs:
                    input: 1 * self.inputDimension (numpy)
                    output: 1 * self.outputDimension (numpy)
        """

        for j in range( self.outputDim ):

            if( self.localModelsNb[j] == 0 ):

                k_new_scl = self.kernel[j]( input, input ).numpy()

                #gram = np.array( k_new_scl ) + np.array( [ [ self.kernel[j].kernels[1].variance.numpy() ] ] )
                gram = np.array( k_new_scl ) + np.array( [ [ self.gpModel[j].likelihood.variance.numpy() ] ] )

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

                    K_new = np.transpose( np.array( self.kernel[j]( input, self.localGPModels[j].localModelsIndex[maxIndex].inputs ).numpy() ) )

                    self.localGPModels[j].localModelsIndex[maxIndex].inputs = np.vstack( ( self.localGPModels[j].localModelsIndex[maxIndex].inputs, input ) )

                    self.localGPModels[j].localModelsIndex[maxIndex].outputs = np.vstack( ( self.localGPModels[j].localModelsIndex[maxIndex].outputs,\
                                                                                                                            output[ 0, j ].reshape(1, 1) ) )

                    self.localGPModels[j].localModelsIndex[maxIndex].center = np.mean( self.localGPModels[j].localModelsIndex[maxIndex].inputs,\
                                                                                                                axis = 0 ).reshape( 1, self.inputDim )

                    k_new_scl = self.kernel[j]( input, input ).numpy()

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

                    k_new_scl = self.kernel[j]( input, input ).numpy()

                    #gram = np.array( k_new_scl ) + np.array( [ [ self.kernel[j].kernels[1].variance.numpy() ] ] )
                    gram = np.array( k_new_scl ) + np.array( [ [ self.gpModel[j].likelihood.variance.numpy() ] ] )

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

        for x, y in zip( self._rawInputData, self._rawOutputData ):
            
            print(x, y)

            self._partitioning( x.reshape(1, x.shape[0] ), y.reshape(1, y.shape[0] ) )

        self._printLocalModel( 0 )
        self._printLocalModel( 1 )
        self._printLocalModel( 2 )

    def _hyperParametersOpt( self ): # OK

        opt = gpflow.optimizers.Scipy()

        ###

        self.gpModel += [ gpflow.models.GPR( data = ( self._rawInputData, self._rawOutputData[ :, 0 ].reshape( -1, 1 ) ), kernel = self.kernel[0], mean_function = None ) ]

        opt.minimize( self.gpModel[0].training_loss, self.gpModel[0].trainable_variables, method = "BFGS", options = dict( maxiter = 100 ) )

        print_summary( self.gpModel[0] )

        time.sleep(2)
        
        ###

        self.gpModel += [ gpflow.models.GPR( data = ( self._rawInputData, self._rawOutputData[ :, 1 ].reshape( -1, 1 ) ), kernel = self.kernel[1], mean_function = None ) ]

        opt.minimize( self.gpModel[1].training_loss, self.gpModel[1].trainable_variables, method = "BFGS", options = dict( maxiter = 100 ) )

        time.sleep(2)

        print_summary( self.gpModel[1] )

        ###

        self.gpModel += [ gpflow.models.GPR( data = ( self._rawInputData, self._rawOutputData[ :, 2 ].reshape( -1, 1 ) ), kernel = self.kernel[2], mean_function = None ) ]

        opt.minimize( self.gpModel[2].training_loss, self.gpModel[2].trainable_variables, method = "BFGS", options = dict( maxiter = 100 ) )
        print_summary( self.gpModel[2] )

        time.sleep(2)

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

                X[ i ] = X[ i ] - A[ i, j ] * X[ j ]
        
            X[ i ] = X[ i ] / A[ i, i ]

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

                    cholesky * X = output <=> X = cholesky⁻¹ * output
        """

        Y = self._forwardSubs( cholesky, output )

        X = self._backSubs( np.transpose( cholesky ), Y )

        return X
    
    ###   Squared - exponential kernel
    def _kernelSE( self, state, xp, xq ):

        res = []

        w = self.kernel[state].kernels[0].lengthscales.numpy()

        W = MX.sym( 'W', len(w), len(w) ) * 0

        for i in range( len(w) ):
            W[i, i] = math.pow( w[i], -2 )

        for row in xq:

            row = row.reshape( 1, self.inputDim )

            res += [ self.kernel[state].kernels[0].variance.numpy() * exp( -0.5 * mtimes( mtimes( xp - row, W ), transpose( xp - row ) ) ) ]
        
        res = transpose( vertcat(*res) )

        return res
    
    def _prediction( self, state, test ):

        wk = []
        yk = []

        for i in range( self.localModelsNb[state] ):

            dist = self._kernelSE( state, test, self.localGPModels[state].localModelsIndex[i].center )

            wk += [ dist ]
            yk += [ mtimes( self._kernelSE( state, test, self.localGPModels[state].localModelsIndex[i].inputs ), self.localGPModels[state].localModelsIndex[i].predictionVector ) ]
    
        wk = vertcat( *wk )
        yk = vertcat( *yk )

        print( "(Wk shape, Yk shape): ", wk.shape, yk.shape )

        sum_wk = 0

        for i in range( wk.shape[0] ):
            sum_wk = sum_wk + wk[i, 0]

        y = dot( wk, yk ) / sum_wk
        
        return y
    
    def _rankOneUpdate(self, L, x ):

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