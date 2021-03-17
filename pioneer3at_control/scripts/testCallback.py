#!/usr/bin/python3
from sys import path
path.append(r"../src")

import gpflow
import tensorflow.compat.v1 as tf
from casadi import *
from classes import GaussianProcess, GPR, TensorFlowEvaluator

gp = GaussianProcess( kernel = gpflow.kernels.SquaredExponential, xMin = -2, xMax = 4, nbSamples = 100 )

observations = np.array( [ [ -2.003215153478787869e-06, -3.884095967124176791e-08, -2.179177756985985865e-07, 3.036447113050129709e-01,	2.443460952792061214e+00, -5.834045576816559997e-02, -1.454589080009882456e-02, -4.886921905992183679e-01 ],  
                           [ -2.023559936684127258e-06,	-3.889568486623122515e-08, -2.179585817888019618e-07, 2.995377864115198463e-01,	2.443460952792061214e+00, -5.755137636899704257e-02, -1.434915138956683461e-02, -4.886921905992107629e-01 ],
                           [ -2.043904727966339152e-06,	-3.895041004307675360e-08, -2.179993802615667594e-07, -5.730389348989807230e-01, 2.442035772445206199e+00, 1.101053654870729620e-01, 2.743566350792468397e-02, -4.884071545298336869e-01 ], 
                           [ -2.064249526839700977e-06,	-3.900513500641870075e-08, -2.180401727789921149e-07, -6.887379396336021919e-01, 2.443458911687557578e+00, 6.469179847967672370e-02, 3.578601106909032326e-02, -2.356399495664729948e-01 ],
                           [ -6.764016486428868491e-02,	2.792481942144573598e-03, 2.530516147308657371e-01, -5.065212476363187832e-01, 2.350555071422394615e+00, -2.509242670200573233e-02, 1.445516887263871564e-02, -1.743590000623806979e-01 ]                           
                        ] )

gp._buildModel( observations )

X = MX.sym( 'X', 3 )
U = MX.sym( 'U', 2 )

modelX = gp._model( 0 )
modelY = gp._model( 1 )
modelTheta = gp._model( 2 )

with tf.Session() as session:

   fX = GPR( 'fX', modelX, session, opts = { "enable_fd": True } )
   fY = GPR( 'fY', modelY, session, opts = { "enable_fd": True } )
   fTheta = GPR( 'fTheta', modelTheta, session, opts = { "enable_fd": True } )

   session.run( fX._mean )
   session.run( fY._mean )
   session.run( fTheta._mean )

res = f( X, U )

print( res )

print( res[0] )
print( res[1] )
print( res[2] )

a = [2, 3, 1]
b = [1, 1]

res = f( a, b )

print( res )