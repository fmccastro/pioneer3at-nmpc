#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import gpflow, sys, math, pprint, time

import numpy as np
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
    def __init__( self, inputDim, outputDim, limitValue, predLimitValue, nbTrainPts, maxNbDataPts ):

        self.X_trainingPts = None
        self.Y_trainingPts = None

        self.trainingPointsNb = nbTrainPts
        self.trainingPtsCollected = 0

        self.inputDimension = inputDim
        self.outputDimension = outputDim

        self.threshold = limitValue
        self.predThreshold = predLimitValue

        self.maxNbDataPoints = maxNbDataPts

        weights = []
        self.kernel = []
        self.localModelsNb = []
        self.localGPModels = []

        for i in range( self.inputDimension ):
            weights.append( 1.0 )

        for i in range( self.outputDimension ):
            self.localModelsNb += [ 0 ]
            self.localGPModels += [ LGP_Record() ]
            self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) + gpflow.kernels.White() ]
    
    def _trainingPoints( self, input, output, option = 0 ): # OK

        if( option == 0 ):

            self.X_trainingPts = np.array( input )
            self.Y_trainingPts = np.array( output )

        else:

            self.X_trainingPts = np.vstack( ( self.X_trainingPts, input  ) )
            self.Y_trainingPts = np.vstack( ( self.Y_trainingPts, output ) )

        self.trainingPtsCollected += 1

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

        for j in range( self.outputDimension ):

            if( self.localModelsNb[j] == 0 ):

                k_new_scl = self.kernel[j]( input, input ).numpy()

                gram = np.array( k_new_scl ) + np.array( [ [ self.kernel[j].kernels[1].variance.numpy() ] ] )

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
                                                                                                                axis = 0 ).reshape( 1, self.inputDimension )

                    k_new_scl = self.kernel[j]( input, input ).numpy()

                    kernel_var = self.kernel[j].kernels[1].variance.numpy()

                    cholesky = self.localGPModels[j].localModelsIndex[maxIndex].cholesky

                    l = self._forwardSubs( cholesky, K_new )

                    self.localGPModels[j].localModelsIndex[maxIndex].cholesky = self._updateCholesky( cholesky, kernel_var, k_new_scl, l )

                    if( self.localGPModels[j].localModelsIndex[maxIndex].inputs.shape[0] > self.maxNbDataPoints ):

                        self.localGPModels[j].localModelsIndex[maxIndex].inputs = np.delete( self.localGPModels[j].localModelsIndex[maxIndex].inputs, 0, 0 )
                        self.localGPModels[j].localModelsIndex[maxIndex].outputs = np.delete( self.localGPModels[j].localModelsIndex[maxIndex].outputs, 0, 0 )

                        self.localGPModels[j].localModelsIndex[maxIndex].center = np.mean( self.localGPModels[j].localModelsIndex[maxIndex].inputs,\
                                                                                                                axis = 0 ).reshape( 1, self.inputDimension )
                        
                        ch = self.localGPModels[j].localModelsIndex[maxIndex].cholesky
                        
                        self.localGPModels[j].localModelsIndex[maxIndex].cholesky = ( self._rankOneUpdate( ch[ 1:, 1: ].T, ch[ 1:, 0 ] ) ).T

                    ch = self.localGPModels[j].localModelsIndex[maxIndex].cholesky

                    newOutput = self.localGPModels[j].localModelsIndex[maxIndex].outputs

                    self.localGPModels[j].localModelsIndex[maxIndex].predictionVector = self._predictionVector( ch, newOutput )

                else:

                    k_new_scl = self.kernel[j]( input, input ).numpy()

                    gram = np.array( k_new_scl ) + np.array( [ [ self.kernel[j].kernels[1].variance.numpy() ] ] )

                    alpha = np.linalg.inv( gram ) @ output[ 0, j ].reshape(1, 1)

                    ch = np.sqrt( gram )

                    newlocalGP = localModel( self.localModelsNb[j], j, input, input, output[0, j].reshape(1, 1), alpha, ch )
                    self.localGPModels[j].localModelsIndex += [ newlocalGP ]
                    self.localModelsNb[j] += 1

        self.trainingPtsCollected += 1

    def _buildInitialLocalModels( self ): # OK

        """
            Description:    Access each row from both matrices at the same time, and input them in the 
                            self._partitioning function. Each row is reshaped to 1 * N (numpy).
        """

        for x, y in zip( self.X_trainingPts, self.Y_trainingPts ):

            self._partitioning( x.reshape(1, x.shape[0] ), y.reshape(1, y.shape[0] ) )

        self._printLocalModel( 0 )
        self._printLocalModel( 1 )
        self._printLocalModel( 2 )

    def _collectTrainPts( self, input, output, queue, flag ): # OK
    
        """
            inputs:
                    input: ( 1 * self._inputDimension ) array (numpy)
                    output: ( 1 * self._outputDimension ) array (numpy)

            output:
                    bool: True or False
        """

        if( self.trainingPtsCollected == 0 ):
            self._trainingPoints( input, output )

            if( flag ):
                queue.put( list( self.localModelsNb ) )
                queue.put( list( self.localGPModels ) )

            if( self.trainingPtsCollected < self.trainingPointsNb ):
                return False
            
            elif( self.trainingPtsCollected == self.trainingPointsNb ):
                return True
        
        elif( self.trainingPtsCollected < self.trainingPointsNb ):
            self._trainingPoints( input, output, option = 1 )

            if( flag ):
                queue.put( list( self.localModelsNb ) )
                queue.put( list( self.localGPModels ) )

            if( self.trainingPtsCollected < self.trainingPointsNb ):
                return False
            
            elif( self.trainingPtsCollected == self.trainingPointsNb ):
                return True

        elif( self.trainingPtsCollected >= self.trainingPointsNb ):

            self._partitioning( input, output )
            
            if( flag ):
                queue.put( list( self.localModelsNb ) )
                queue.put( list( self.localGPModels ) )

            return False

    def _hyperParametersOpt( self ): # OK

        opt = gpflow.optimizers.Scipy()
        m = []

        for j in range( self.outputDimension ):

            m += [ gpflow.models.GPR( data = ( self.X_trainingPts, self.Y_trainingPts[ :, j ].reshape( -1, 1 ) ), kernel = self.kernel[j], mean_function = None ) ]

            opt.minimize( m[j].training_loss, m[j].trainable_variables, options = dict( maxiter = 100 ) )

            time.sleep(2)
        
            print_summary( self.kernel[j] )

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

            row = row.reshape( 1, self.inputDimension )

            res += [ self.kernel[state].kernels[0].variance.numpy() * exp( -0.5 * mtimes( mtimes( xp - row, W ), transpose( xp - row ) ) ) ]
        
        res = transpose( vertcat(*res) )

        return res
    
    """def _closestModels( self, state, test, max ):

        wk = []
        yk = []

        distMatrix = []

        flag = True
        
        for i in range( self.localModelsNb[state] ):

            if( flag ):

                distMatrix += [ self._kernelSE( state, test, self.localGPModels[state].localModelsIndex[i].center ),\
                            mtimes( self._kernelSE( state, test, self.localGPModels[state].localModelsIndex[i].inputs ), self.localGPModels[state].localModelsIndex[i].predictionVector ) ]

                distMatrix = np.array( distMatrix )

                flag = False
            
            else:

                dist = self._kernelSE( state, test, self.localGPModels[state].localModelsIndex[i].center )

                for idx in range( distMatrix.shape[0] ):
                
                    distMatrix = if_else( dist > distMatrix[ idx, 0 ],\
                        np.vstack( ( np.array( [ [ dist, mtimes( self._kernelSE( state, test, self.localGPModels[state].localModelsIndex[i].inputs ),\
                                                                    self.localGPModels[state].localModelsIndex[i].predictionVector ) ] ] ), distMatrix ) ),\
                                                                        )

        dist = np.array( dist )

        dist = dist[ dist[ :, 0 ].argsort() ]"""

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

    def _trainingPtsCollected( self ):
        return self.trainingPtsCollected 
    
    def _updateLGP( self, input1, input2 ):
        self.localModelsNb = input1
        self.localGPModels = input2

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