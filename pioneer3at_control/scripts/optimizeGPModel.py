#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import pickle, os
import numpy as np

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from classes import Common, OPT

if __name__ == '__main__':
    
    common = Common()

    opt = OPT( common )

    input = np.load(common.pathInputTrainingData_np)
    output = np.load(common.pathOutputTrainingData_np)

    mse = {}

    div = int( (common.nbTrainingPoints / common.trainingSet) )
    print( div )

    for k in range( common.trainingSet ):
        print(k)
        valInput = input[ div * k : div * ( k + 1 ), : ]
        valOutput = output[ div * k : div * ( k + 1 ), : ]

        trainInput = np.delete( input, slice( div * k, div * ( k + 1 ) ), 0 )
        trainOutput = np.delete( output, slice( div * k, div * ( k + 1 ) ), 0 )

        opt._saveTrainingData( trainInput, trainOutput )
        opt._hyperParametersOpt()

        valError = opt._valMSE( valInput, valOutput )
        trainError = opt._trainMSE( trainInput, trainOutput )

        for state in range( common.nbOutputs ):

            print(state, valError[state], trainError[state] )

            parametersGP = opt._loadParameters( state )

            if( k == 0 ):
                mse[ str(state) ] = np.array( [ [ valError[state], trainError[state], parametersGP ] ] )

            else:
                mse[ str(state) ] = np.vstack( ( mse[ str(state) ], np.array( [ [ valError[state], trainError[state], parametersGP ] ] ) ) )

    for state in range( common.nbOutputs ):
        mse[ str(state) ] = mse[ str(state) ][ ( mse[ str(state) ][ :, 0 ] ).argsort() ]

    print( "1", mse['0'] )
    print( "2", mse['1'] )
    print( "3", mse['2'] )

    parametersGP = []

    for state in range( common.nbOutputs ):
        parametersGP += [ mse[ str(state) ][0, 2] ]

    os.remove(common.pathKernel)

    f = open(common.pathKernel, "wb")

    pickle.dump( parametersGP, f )

    f.close()