/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    DifferentialState x;
    DifferentialState y;
    DifferentialState theta;
    Control v_x;
    Control w_z;
    OnlineData x_ref; 
    OnlineData y_ref; 
    OnlineData theta_ref; 
    OnlineData v_x_ref; 
    OnlineData w_z_ref; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "sim_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "sim_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << (x-x_ref);
    acadodata_f1 << (y-y_ref);
    acadodata_f1 << (theta-theta_ref);
    acadodata_f1 << (v_x-v_x_ref);
    acadodata_f1 << (w_z-w_z_ref);
    Function acadodata_f2;
    acadodata_f2 << (x-x_ref);
    acadodata_f2 << (y-y_ref);
    acadodata_f2 << (theta-theta_ref);
    SIMexport ExportModule2( 1, 0.1 );
    ExportModule2.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule2.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule2.set( NUM_INTEGRATOR_STEPS, 5 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    DifferentialEquation acadodata_f4;
    acadodata_f4 << dot(x) == cos(theta)*v_x;
    acadodata_f4 << dot(y) == sin(theta)*v_x;
    acadodata_f4 << dot(theta) == w_z;

    ExportModule2.setModel( acadodata_f4 );

    uint export_flag = 0;
    ExportModule2.setTimingSteps( 0 );
    export_flag = ExportModule2.exportCode( "export_SIM" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

