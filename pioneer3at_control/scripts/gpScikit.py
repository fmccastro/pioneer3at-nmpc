#!/usr/bin/env python3
import rospy
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.colors import LogNorm

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel

if __name__ == '__main__':

    inputData = np.load("/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/trainingData/input.npy")
    outputData = np.load("/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/trainingData/output.npy")

    kernel = ConstantKernel() * RBF( length_scale = np.array( [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ] ) ) + WhiteKernel()

    print("A")

    gpr1 = GaussianProcessRegressor( kernel = kernel, n_restarts_optimizer = 0, normalize_y = True ).fit(inputData, outputData[:, 0] )

    print(gpr1.log_marginal_likelihood( theta = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] ) )