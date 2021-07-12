#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import gpflow, math, time, pickle

import numpy as np

from gpflow.utilities import print_summary
from casadi import*

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

"""
    Sparse Online Gaussian Process (SOGP)
"""

class SOGP:
    def __init__( self, common ):

        self.__nbStates = common.NbStates
        self.__nbControls = common.NbControls

        self.__inputDim = common.nbInputs
        self.__outputDim = common.nbOutputs

        self.__alpha = [ 0 ] * self.__outputDim
        self.__omega = [ 0 ] * self.__outputDim
        self.__R = [ 0 ] * self.__outputDim

        self.__v = common.v
        self.__R_max = common.R_max
        self.__maxSize = common.maxSize

        self.__matrixInverse = [0] * self.__outputDim

        self.__inputDataSet = [0] * self.__outputDim
        self.__outputDataSet = [0] * self.__outputDim

        self.__rawInputData = None
        self.__rawOutputData = None

        self.__pointsCollected = 0

        weights = []
        self.kernel = []
        self.gpModel = []

        for _ in range( self.__inputDim ):
            weights.append( 1.0 )

        weights = [0.2, 0.5, 0.9 -0.2, 0.1]

        for _ in range( self.__outputDim ):
            #self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) + gpflow.kernels.White() ]
            self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) ]
    
    def _saveTrainingData( self, rawInput, rawOutput ):
        self.__rawInputData = rawInput
        self.__rawOutputData = rawOutput

    def _hyperParametersOpt( self ): # OK

        opt = gpflow.optimizers.Scipy()

        ###

        self.gpModel += [ gpflow.models.GPR( data = ( self.__rawInputData, self.__rawOutputData[ :, 0 ].reshape( -1, 1 ) ), kernel = self.kernel[0], mean_function = None ) ]

        opt.minimize( self.gpModel[0].training_loss, self.gpModel[0].trainable_variables, tol = 1e-6/100 )

        print_summary( self.gpModel[0] )

        time.sleep(2)
        
        ###

        self.gpModel += [ gpflow.models.GPR( data = ( self.__rawInputData, self.__rawOutputData[ :, 1 ].reshape( -1, 1 ) ), kernel = self.kernel[1], mean_function = None ) ]

        opt.minimize( self.gpModel[1].training_loss, self.gpModel[1].trainable_variables, tol = 1e-6/100 )

        time.sleep(2)

        print_summary( self.gpModel[1] )

        ###

        self.gpModel += [ gpflow.models.GPR( data = ( self.__rawInputData, self.__rawOutputData[ :, 2 ].reshape( -1, 1 ) ), kernel = self.kernel[2], mean_function = None ) ]

        opt.minimize( self.gpModel[2].training_loss, self.gpModel[2].trainable_variables, tol = 1e-6/100 )

        time.sleep(2)
        
        print_summary( self.gpModel[2] )

    def _nbPointsCollected( self ):
        return self.__pointsCollected
    
    def _initSet( self, input, output ):

        input = input.reshape(1, -1)
        output = output.reshape(1, -1)

        for i in range( self.__outputDim ):
            self.__inputDataSet[i] = input
            self.__outputDataSet[i] = output

            self.__matrixInverse[i] = np.linalg.inv( self.funCommon._kernelSE( i, input, input ) + self.gpModel[i].likelihood.variance.numpy() )

            self.__alpha[i] = self.__matrixInverse[i] @ np.array( [ [ output[0, i] ] ] )
            self.__omega[i] = -self.__matrixInverse[i]
            self.__R[i] = 1
            
            self.__pointsCollected += 1

    def _initSOGP( self ):

        for i in range( self.__rawInputData.shape[0] ):

            print(i)

            a = time.time()

            if ( i == 0 ):
                self._initSet( self.__rawInputData[ i, : ], self.__rawOutputData[ i, : ] )
            
            else:
                self._learning( self.__rawInputData[ i, : ], self.__rawOutputData[ i, : ] )
    
            print( time.time() - a )

        res = [ self.__inputDataSet, self.__outputDataSet, self.__matrixInverse, self.__alpha, self.__omega, self.__R ]

        return res

    def _learning( self, input, output ):

        prevAlpha = [0] * self.__outputDim
        prevOmega = [0] * self.__outputDim

        input = input.reshape(1, -1)
        output = output.reshape(1, -1)

        for i in range( self.__outputDim ):

            crossVariance = self._kernelSE( i, input, self.__inputDataSet[i] )
            variance = self._kernelSE( i, input, input )

            average = crossVariance @ self.__alpha[i]
            covariance = variance + crossVariance @ self.__omega[i] @ crossVariance.T

            alpha = np.linalg.inv( covariance + self.gpModel[i].likelihood.variance.numpy() ) @ ( output[0, i] - average )
            omega = -np.linalg.inv( covariance + self.gpModel[i].likelihood.variance.numpy() )

            beta = self.__matrixInverse[i] @ crossVariance.T
            novelty = variance - crossVariance @ beta

            if( novelty < self.__v[i] ):
                s = self.__omega[i] @ crossVariance.T + beta

                self.__alpha[i] = self.__alpha[i] + s @ alpha
                self.__omega[i] = self.__omega[i] + s @ omega @ s.T
            
            else:
                e = self.__omega[i] @ crossVariance.T

                prevAlpha[i] = self.__alpha[i]
                prevOmega[i] = self.__omega[i]

                self.__alpha[i] = np.vstack( ( self.__alpha[i], 0 ) ) + np.vstack( ( e, 1 ) ) @ alpha
                self.__omega[i] = np.block( [ [ self.__omega[i], np.zeros( ( self.__omega[i].shape[0], 1 ) ) ], [ np.zeros( ( 1, self.__omega[i].shape[0] ) ), 0 ] ] ) +\
                                    np.vstack( ( e, 1 ) ) @ omega @ np.vstack( ( e, 1 ) ).T 

                self.__matrixInverse[i] = self._matrixInverse( self.__matrixInverse[i], self.__inputDataSet[i], input, i )

                self.__inputDataSet[i] = np.vstack( ( self.__inputDataSet[i], input ) )
                self.__outputDataSet[i] = np.vstack( ( self.__outputDataSet[i], output ) )

                self.__R[i] += 1
            
            if( self.__R[i] > self.__R_max[i] ):
                
                error = []
                for j in range( self.__inputDataSet[i].shape[0] ):
                    error += [ [ abs( self.__alpha[i][j, 0] ) / self.__matrixInverse[i][j, j], j ] ]
                
                error = np.array( error )

                error = error[ ( error[ :, 0 ] ).argsort() ]

                index = int( error[0, 1] )

                removedInput = self.__inputDataSet[i][ index, : ].reshape(1, -1)

                self.__inputDataSet[i] = np.delete( self.__inputDataSet[i], index, 0 )
                self.__outputDataSet[i] = np.delete( self.__outputDataSet[i], index, 0 )

                self.__matrixInverse[i] = self._matrixInverseDown( self.__matrixInverse[i], index, index )

                crossVariance = self._kernelSE( i, self.__inputDataSet[i], removedInput )

                e = prevOmega[i] @ crossVariance

                beta = self.__matrixInverse[i] @ crossVariance

                s = e + beta

                self.__alpha[i] = prevAlpha[i] + s * self.__alpha[i][ index, 0 ]

                omega_int = self.__omega[i][ index, index ]

                self.__omega[i] = prevOmega[i] + ( e * omega_int ) @ e.T + ( e * omega_int ) @ beta.T +\
                                    ( beta * omega_int ) @ e.T + ( beta * omega_int ) @ beta.T

                self.__R[i] -= 1
    
    def _fullPrediction( self, input, inputsDataSet, alpha, omega ):

        input = reshape( input, 1, -1 )

        for i in range( self.__outputDim ):

            dataSet_ = inputsDataSet[ i * self.__maxSize * self.__inputDim : ( i + 1 ) * self.__maxSize * self.__inputDim ]

            _alpha = alpha[ i * self.__maxSize : ( i + 1 ) * self.__maxSize ]

            omega_ = omega[ i * self.__maxSize * self.__maxSize : ( i + 1 ) * self.__maxSize * self.__maxSize ]

            for j in range( self.__maxSize ):
            
                if( j == 0 ):
                    _dataSet = reshape( dataSet_[ j * self.__inputDim : ( j + 1 ) * self.__inputDim ], 1, -1 )
                    _omega = reshape( omega_[ j * self.__maxSize : ( j + 1 ) * self.__maxSize ], 1, -1 )
                
                else:
                    _dataSet = vertcat( _dataSet, reshape( dataSet_[ j * self.__inputDim : ( j + 1 ) * self.__inputDim ], 1, -1 ) )
                    _omega = vertcat( _omega, reshape( omega_[ j * self.__maxSize : ( j + 1 ) * self.__maxSize ], 1, -1 ) )

            crossVariance = self._kernelSE( i, input, _dataSet, is_sym = True )
            
            if( i == 0 ):
                average = crossVariance @ _alpha
                covariance = self._kernelSE( i, input, input, is_sym = True ) + crossVariance @ _omega @ crossVariance.T
            
            else:
                average = vertcat( average, crossVariance @ _alpha )
                covariance = vertcat( covariance, self._kernelSE( i, input, input, is_sym = True ) + crossVariance @ _omega @ crossVariance.T )
        
        return average, covariance
    
    def _parameters( self ):

        input = []
        alpha = []
        omega = []

        for i in range( self.__outputDim ):
            input += self.__inputDataSet[i].ravel().tolist()
            alpha += self.__alpha[i].ravel().tolist()
            omega += self.__omega[i].ravel().tolist()
        
        input += [0] * ( self.__maxSize * self.__inputDim * self.__outputDim - len(input) )
        alpha += [0] * ( self.__maxSize * self.__outputDim - len(alpha) )
        omega += [0] * ( self.__maxSize * self.__maxSize * self.__outputDim - len(omega) )

        return input, alpha, omega

    def _matrixInverseDown( self, matrixInverse, row, column ):

        """
            Compute matrix inverse for a submatrix with size (N-1)*(N-1) after removal of the p_th row and j_th column from matrix K with size N*N, 
            by applying the Sherman-Morrison formula.
        """

        x = np.zeros( ( matrixInverse.shape[0], 1 ) )
        x[ row, 0 ] = 1

        u = matrixInverse[ :, column ].reshape(-1, 1) - x

        x[ row, 0 ] = 0
        x[ column, 0 ] = 1

        v = x

        inv = np.linalg.inv( np.array( [ [ 1 ] ] ) - v.T @ matrixInverse @ u )

        A = matrixInverse + ( matrixInverse @ u @ v.T @ matrixInverse ) * inv[0, 0]

        A = np.delete( A, column, 0 )
        A = np.delete( A, row, 1 )

        return A 
   
    def _matrixInverse( self, prevInverse, prevInputSet, newInput, state ):

        """
            Woodbury matrix inverse

            It creates a matrix inverse for a N*N matrix K, from its submatrix K( :-1, :-1 ) of size (N-1)*(N-1) 
        """

        crossVariance = self._kernelSE( state, prevInputSet, newInput )
        C = np.linalg.inv( self._kernelSE( state, newInput, newInput ) + self.gpModel[state].likelihood.variance.numpy() - crossVariance.T @ prevInverse @ crossVariance )

        newInverse = np.block( [ [ prevInverse + prevInverse @ crossVariance @ C @ crossVariance.T @ prevInverse, -prevInverse @ crossVariance @ C ],\
                                 [ -C @ crossVariance.T @ prevInverse, C ] ] )

        return newInverse
    
    def _loadParameters( self ):

        """
            Load parameters of each model
        """

        parameters = []

        for index in range( self.__outputDim ):
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

            self.gpModel += [ gpflow.models.GPR( data = ( self.__rawInputData, self.__rawOutputData[ :, index ].reshape( -1, 1 ) ),\
                                                                kernel = self.kernel[index], mean_function = None, noise_variance = element[2] ) ]

            #print_summary( self.kernel[index] )
            #print_summary( self.gpModel[index] )

            index += 1
        
        index = 0

        output2 = pickle.load( pickle_in )

        for element in output2:

            if( index == 0 ):
                self.__inputDataSet = element
            
            elif( index == 1 ):
                self.__outputDataSet = element
            
            elif( index == 2 ):
                self.__matrixInverse = element
            
            elif( index == 3 ):
                self.__alpha = element
            
            elif( index == 4 ):
                self.__omega = element
            
            elif( index == 5 ):
                self.__R = element

            index += 1

    def _printSOGP( self ):
        
        for i in range( self.__outputDim ):

            print("|************************************************************")

            print("| Output: ", i )
            print("| Inputs: ", self.__inputDataSet[i] )
            print("| Outputs: ", self.__outputDataSet[i] )

            print("| Number of data points: ", self.__R[i] )
            print("| Alpha: ", self.__alpha[i] )
            print("| Omega: ", self.__omega[i] )

            print("|************************************************************")

            print()
            print()