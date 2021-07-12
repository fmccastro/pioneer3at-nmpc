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
    Hyper-parameters optimization class
"""

class OPT:
    def __init__( self, common ):

        self.__inputDim = common.nbInputs
        self.__outputDim = common.nbOutputs

        self.__rawInputData = None
        self.__rawOutputData = None

        weights = []
        self.kernel = []
        self.gpModel = []

        for _ in range( self.__inputDim ):
            weights.append( 1.0 )
        
        print( weights)
        print( len(weights) )
        
        weights = [0.2, 0.5, 0.9, 0.2, 0.1, 0.35]

        print( weights)
        print( len(weights) )

        for _ in range( self.__outputDim ):
            #self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) + gpflow.kernels.White() ]
            self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) ]
    
    def _saveTrainingData( self, rawInput, rawOutput ):
        self.__rawInputData = rawInput
        self.__rawOutputData = rawOutput

    def _hyperParametersOpt( self ): # OK

        weights = [0.2, 0.5, 0.9, 0.2, 0.1, 0.35]

        for _ in range( self.__outputDim ):
            #self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) + gpflow.kernels.White() ]
            self.kernel += [ gpflow.kernels.SquaredExponential( lengthscales = weights ) ]

        opt = gpflow.optimizers.Scipy()

        for state in range( self.__outputDim ):

            ###

            self.gpModel += [ gpflow.models.GPR( data = ( self.__rawInputData, self.__rawOutputData[ :, state ].reshape( -1, 1 ) ), kernel = self.kernel[state], mean_function = None ) ]

            opt.minimize( self.gpModel[state].training_loss, self.gpModel[state].trainable_variables, tol = 1e-6/100 )

            print_summary( self.gpModel[state] )

            time.sleep(2)
    
    def _loadParameters( self, state ):

        """
            Load parameters of each model
        """

        parameters = [ self.gpModel[state].kernel.variance.numpy(),\
                            self.gpModel[state].kernel.lengthscales.numpy(),\
                            self.gpModel[state].likelihood.variance.numpy() ]

        return parameters
    
    def _valMSE( self, validationInput, validationOutput ):

        mse = [0] * self.__outputDim

        for j in range( validationInput.shape[0] ):

            for state in range(self.__outputDim):
                mean, var = self.gpModel[state].predict_f( validationInput[j, :].reshape(1, -1) )
                mse[state] += math.pow(validationOutput[j, state] - mean.numpy(), 2)
        
        for state in range(self.__outputDim):
            mse[state] = (1 / validationInput.shape[0]) * mse[state]
        
        return mse
    
    def _trainMSE( self, trainingInput, trainingOutput ):

        mse = [0] * self.__outputDim

        for j in range( trainingInput.shape[0] ):

            for state in range(self.__outputDim):
                mean, var = self.gpModel[state].predict_f( trainingInput[j, :].reshape(1, -1) )
                mse[state] += math.pow(trainingOutput[j, state] - mean.numpy(), 2)
        
        for state in range(self.__outputDim):
            mse[state] = (1.0 / trainingInput.shape[0]) * mse[state]
        
        return mse