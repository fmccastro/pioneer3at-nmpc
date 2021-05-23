#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import pickle, os
import numpy as np

from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from classes import Common, Model, LGP, localModel, LGP_Record

from gpflow.utilities import print_summary

common = Common()

gp = LGP( inputDimension = common.LGP.nbInputs, outputDimension = common.LGP.nbOutputs, limitValue = common.LGP.limitValue, maxNbDataPts = common.LGP.maxDataPts,\
                        maxNbLocalModels = common.LGP.maxLocalModels )

input = np.load(common.LGP.pathInputTrainingData_np)
output = np.load(common.LGP.pathOutputTrainingData_np)

gp._saveTrainingData( input, output )
gp._hyperParametersOpt()
gp._buildInitialLocalModels()

parametersGP = gp._loadParameters()
localModelsNb, localGPModels = gp._loadModel()

###
os.remove(common.LGP.pathModel)

f = open(common.LGP.pathModel, "wb")

pickle.dump( parametersGP, f )
pickle.dump( localModelsNb, f )
pickle.dump( localGPModels, f )

f.close()