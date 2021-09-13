#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import pickle, os, sys
import numpy as np
import matplotlib.pyplot as plt 

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from classes import Common, OPT

if __name__ == '__main__':
    
    common = Common()

    opt = OPT( common )

    input = np.load( common.pathInputTrainingData_np )
    output = np.load( common.pathOutputTrainingData_np )

    """for i in range( input.shape[0] ):
        if( np.linalg.norm( input[i, :].reshape(1, -1) - np.array( [0] * input.shape[1] ).reshape(1, -1) ) < 0.1 ):
            input = np.delete( input, i, 0 )
            output = np.delete( output, i, 0 )
            i = i - 2"""

    mse = {}

    div = int( (common.nbTrainingPoints / common.trainingSet) )

    for k in range( common.trainingSet ):
        print("Index: ", k)

        #   { u_k, u_{k-1}, v_{k-1} }
        if( common.gpModel == 1 ):
            valInput = input[ div * k : div * ( k + 1 ), common.cutVar: ]
            valOutput = output[ div * k : div * ( k + 1 ), : ]

            trainInput = np.delete( input, slice( div * k, div * ( k + 1 ) ), 0 )
            trainOutput = np.delete( output, slice( div * k, div * ( k + 1 ) ), 0 )

            trainInput = trainInput[ :, common.cutVar: ]

        opt._saveTrainingData( trainInput, trainOutput )
        opt._hyperParametersOpt()

        valError = opt._valMSE( valInput, valOutput )
        trainError = opt._trainMSE( trainInput, trainOutput )

        for state in range( common.nbOutputs ):

            parametersGP = opt._loadParameters( state )

            if( k == 0 ):
                mse[ str(state) ] = np.array( [ [ valError[state], trainError[state], parametersGP ] ] )

            else:
                mse[ str(state) ] = np.vstack( ( mse[ str(state) ], np.array( [ [ valError[state], trainError[state], parametersGP ] ] ) ) )

    for state in range( common.nbOutputs ):
        mse[ str(state) ] = mse[ str(state) ][ ( mse[ str(state) ][ :, 0 ] ).argsort() ]

    parametersGP = []

    for state in range( common.nbOutputs ):
        parametersGP += [ mse[ str(state) ][0, 2] ]

    os.remove(common.pathKernel)

    f = open(common.pathKernel, "wb")

    pickle.dump( parametersGP, f )

    f.close()