clc;
clear all;
close all;

%% Declaration of parameters, differential states and controls
acadoSet('results_to_file', false);
EXPORT = 1;

DifferentialState x y theta;
Control v_x w_z;
OnlineData x_ref y_ref theta_ref v_x_ref w_z_ref;

nbStates = length(diffStates);
nbControls = length(controls);

%% ODE
f = dot([x; y; theta]) == [cos(theta) * v_x; sin(theta) * v_x; w_z];

%% NMPC solver
N = 60;
Ts = 0.1;
Ns = 20;
Integrator = num2str('INT_IRK_GL4');
T = N * Ts;

h = [x - x_ref, y - y_ref, theta - theta_ref, v_x - v_x_ref, w_z - w_z_ref];
hN = [x - x_ref, y - y_ref, theta - theta_ref];

W_mat = eye(5, 5);
WN_mat = eye(3, 3);

W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp = acado.OCP(0.0, N * Ts, N);

ocp.minimizeLSQ(W, h);
ocp.minimizeLSQEndTerm(WN, hN);

ocp.subjectTo( -0.7 <= v_x <= 0.7 );
ocp.subjectTo( -140*pi/180 <= w_z <= 140*pi/180 );
ocp.setModel( f );

%% Export NMPC
acadoSet('problemname', 'nmpc');

nmpc = acado.OCPexport( ocp );
nmpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
nmpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
nmpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
nmpc.set( 'INTEGRATOR_TYPE',             Integrator          );
nmpc.set( 'NUM_INTEGRATOR_STEPS',        Ns                  );
nmpc.set( 'QP_SOLVER',                   'QP_QPOASES'        );
nmpc.set( 'HOTSTART_QP',                 'NO'                );
nmpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10	    );

if EXPORT
    nmpc.exportCode('export_NMPC');
    copyfile('/home/fmccastro/Desktop/ACADOtoolkit/external_packages/qpoases', 'export_NMPC/qpoases', 'f');
    
    cd export_NMPC
    make_acado_solver('../acado_NMPCstep');
    cd ..
end

%% Simulation Export
acadoSet('problemname', 'sim');

nbSteps = 5;
sim = acado.SIMexport( Ts );
sim.setModel( f );
sim.set( 'INTEGRATOR_TYPE',     'INT_IRK_GL4');
sim.set( 'NUM_INTEGRATOR_STEPS',    nbSteps);

if EXPORT
    sim.exportCode('export_SIM');
    
    cd export_SIM
    make_acado_integrator('../integrate_nmpc_CodeGeneration');
    cd ..
end

%% Parameters Simulation
%{
timeArray = 0:Ts:T;

ref_x = 0.25;
ref_y = 0.25;

ref = [ ref_x/2, ref_y/2, atan( ref_y / ref_x )/2, 0.7/2, 0.0 ];

%   Define Online Data
input.od = repmat( ref, N + 1, 1);

%   Define trajectories
input.y = repmat(ref, N, 1);
input.yN = [ ref(1) ref(2) ref(3) ];

%   Initialize state and control trajectories
input.x = repmat([0 0 0], N + 1, 1);
input.u = zeros(N, nbControls);

input.W = diag([0.8 0.8 0.8 0.8 0.8]);
input.WN = diag([0.1 0.1 0.1]);

%input.shifting.strategy = 1;

%   Current State values
stateSim = zeros(1, nbStates);

trajectory = [];
commands = [];

disp('--------------------------------------------------');
disp('                 Simulation Loop                  ');
disp('--------------------------------------------------');

for time = 0:Ts:T                              
    disp('|---------------------------------|');
    disp(['| Current time -> ', num2str(time)]);
    input.x0 = stateSim;                                                    %   Current state values
    output = acado_NMPCstep(input);                                         %   Solve OCP
    
    disp(['| Iteration time: ', num2str(output.info.cpuTime)]);
    disp(['| KKT value: ', num2str(output.info.kktValue)]);
    disp(['| Obj value: ', num2str(output.info.objValue)]);
    disp(['| Number of iterations: ', num2str(output.info.nIterations)]);
    disp(['|---------------------------------|' newline newline]);
    
    %   Shift state and control trajectories
    input.x = [output.x(2:end, :); output.x(end, :)];
    input.u = [output.u(2:end, :); output.u(end, :)];
    
    inputSim.x = stateSim.';
    inputSim.u = output.u(1, :).';
    outputSim = integrate_nmpc_CodeGeneration(inputSim);                    %   Simulate System
    stateSim = outputSim.value.';
    
    trajectory = [trajectory; stateSim];
    commands = [commands; output.u(1, :)];
end
%}

%{
index_1 = find(trajectory(:, 1) >= ref(1)*2 - 0.00001);
index_2 = find(trajectory(:, 2) >= ref(2)*2 - 0.00001);
%}

%iter = 1;
toSave = 0;

%{
while true
    if index_1(iter) == index_2(iter)
        timeToGoal = num2str( timeArray( index_1(iter) ) );
        disp( ['Time to reach goal was ', timeToGoal, ' seconds.'] );
        break
    else
        iter = iter + 1;
    end
end
%}

%draw;
