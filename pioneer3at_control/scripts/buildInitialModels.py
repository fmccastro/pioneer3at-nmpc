#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import pickle, os
import numpy as np

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from classes import Common, LGP, SOGP

if __name__ == '__main__':
    
    common = Common()

    ###     Local Gaussian Processes (LGP)
    if( common.gpType == 0 ):
        gp = LGP( common )

        gp._saveTrainingData( common.pathInputTrainingData_np, common.pathOutputTrainingData_np )
        gp._loadPickle( common.pathKernel, common.pathLocalModels, 0 )

        print( "[nmpc.py] Start local models building." )
        gp._buildInitialLocalModels()
        print( "[nmpc.py] Initial local models are built." )

        a, b = gp._loadModel()

        os.remove(common.pathLocalModels)

        f = open(common.pathLocalModels, "wb")

        pickle.dump( a, f )
        pickle.dump( b, f )

        f.close()

    ###     Sparse Online Gaussian Processes (SOGP)
    elif( common.gpType == 1 ):
        gp = SOGP( common )