#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import gpflow
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf

from gpflow.utilities import print_summary
from gpflow.ci_utils import ci_niter

plt.rcParams["figure.figsize"] = (12, 6)

MAXITER = ci_niter(2000)

def plot_model(m, lower=-8.0, upper=8.0):
    pX = np.linspace(lower, upper, 100)[:, None]
    pY, pYv = m.predict_y(pX)
    if pY.ndim == 3:
        pY = pY[:, 0, :]
    plt.plot(X, Y, "x")
    plt.gca().set_prop_cycle(None)
    plt.plot(pX, pY)
    for i in range(pY.shape[1]):
        top = pY[:, i] + 2.0 * pYv[:, i] ** 0.5
        bot = pY[:, i] - 2.0 * pYv[:, i] ** 0.5
        plt.fill_between(pX[:, 0], top, bot, alpha=0.3)
    plt.xlabel("X")
    plt.ylabel("f")
    plt.title(f"ELBO: {m.elbo(data):.3}")
    plt.plot(Z, Z * 0.0, "o")

if __name__ == '__main__':

    data = np.genfromtxt("gp1.csv", delimiter=",")
    X = data[ :, :5 ].reshape(50, 5)
    Y1 = data[ :, 5 ].reshape(50, 1)
    Y2 = data[ :, 6 ].reshape(50, 1)
    Y3 = data[ :, 7 ].reshape(50, 1)

    kernel = gpflow.kernels.SquaredExponential( lengthscales = [ 1, 1, 1, 1, 1 ] )

    print_summary( kernel )

    m1 = gpflow.models.GPR( data = (X, Y1), kernel = kernel, mean_function = None )
    m2 = gpflow.models.GPR( data = (X, Y2), kernel = kernel, mean_function = None )
    m3 = gpflow.models.GPR( data = (X, Y3), kernel = kernel, mean_function = None )

    print_summary(m1)
    print_summary(m2)
    print_summary(m3)

    opt = gpflow.optimizers.Scipy()

    opt_logs1 = opt.minimize( m1.training_loss, m1.trainable_variables, options = dict( maxiter = 100 ) )
    opt_logs2 = opt.minimize( m2.training_loss, m2.trainable_variables, options = dict( maxiter = 100 ) )
    opt_logs3 = opt.minimize( m3.training_loss, m3.trainable_variables, options = dict( maxiter = 100 ) )
    
    print_summary( m1.kernel )
    print_summary( m2.kernel )
    print_summary( m3.kernel )
    
    ## generate test points for prediction
    xx = np.linspace(-5, 5, 500).reshape(100, 5)  # test points must be of shape (N, D)

    print(xx)

    ## predict mean and variance of latent GP at test points
    mean, var = m1.predict_f(xx)

    print( mean[:, 0] - 1.96 * np.sqrt(var[:, 0]) )
    print( mean[:, 0] + 1.96 * np.sqrt(var[:, 0]) )

    tf.random.set_seed(1)  # for reproducibility
    samples = m1.predict_f_samples(xx, 10)  # shape (10, 100, 1)
    
    plt.figure( figsize = (12, 6) )
    plt.plot( X[:, 4], Y1, "kx", mew = 2 )
    plt.plot( xx[:, 0], mean, "C0", lw = 2 )
    plt.fill_between(
        xx[:, 0],
        mean[:, 0] - 1.96 * np.sqrt(var[:, 0]),
        mean[:, 0] + 1.96 * np.sqrt(var[:, 0]),
        color="C0",
        alpha=0.2,
    )

    plt.plot( xx[:, 0], samples[:, :, 0].numpy().T, "C0", linewidth = 0.5 )
    plt.xlim( -5, 5 )

    plt.show()