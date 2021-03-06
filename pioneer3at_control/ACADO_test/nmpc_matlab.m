% I can include a mex input scalar, vector or matrix so that I define some
% of the parameteres such as : weight matrices, initial and goal points.

clear;

BEGIN_ACADO;

    acadoSet('problemname', 'nmpcMethods');
    acadoSet('results_to_file', true);
    
    %% Symbolics
                                    
    DifferentialState x;                                                    % X coordinate of the robot
    DifferentialState y;                                                    % Y coordinate of the robot
    DifferentialState theta;                                                % Theta coordinate of the robot 
    
    Control v_x;                                                            % X velocity of the robot control actuation
    Control w_z;                                                            % Z angular velocity of the robot control actuation 
    
    %% Discrete Differential Equation (process a-priori model)
    Ts = 0.1;                                                               % Sampling Time
    N = 30;                                                                 % Horizon length(control intervals) 
    simulationTime = N * 2;                                                 % Simulation Time
    World_to_Robot = [cos(theta), 0; sin(theta), 0; 0, 1];                  % Rotation Matrix
    
    %{
    f = acado.DiscretizedDifferentialEquation( Ts );
    f.add(next(x) == x + Ts * World_to_Robot(1, 1) * v_x);                  % Process a-priori Model
    f.add(next(y) == y + Ts * World_to_Robot(2, 1) * v_x);
    f.add(next(theta) == theta + Ts * World_to_Robot(3, 2) * w_z);
    %}
    
    f = acado.DifferentialEquation();
    f.add(dot(x) == World_to_Robot(1, 1) * v_x);                            % Process a-priori Model
    f.add(dot(y) == World_to_Robot(2, 1) * v_x);
    f.add(dot(theta) == World_to_Robot(3, 2) * w_z);
    
    %% Optimal Control Problem
    h = {x, y};
    
    Q = eye(2);                                                             % states weight matrix
    Q(1, 1) = 0.8;
    Q(2, 2) = 0.8;
    %Q(3, 3) = 0.0;
    
    P = eye(2);
    P(1, 1) = 0.1;
    P(2, 2) = 0.1;
    %P(3, 3) = 0.0;
    
    r = [20 20];
    
    ocp = acado.OCP(0, N*Ts, N);
    ocp.minimizeLSQ( Q, h, r );
    ocp.minimizeLSQEndTerm(P, h, r);
    
    ocp.subjectTo( f );
    
    %{
    ocp.subjectTo( 'AT_START', x == 0.0 );
    ocp.subjectTo( 'AT_START', y == 0.0);
    ocp.subjectTo( 'AT_START', theta == 0.0 );
    ocp.subjectTo( 'AT_START', v_x == 0.0 );
    ocp.subjectTo( 'AT_START', w_z == 0.0 );
    %}
    
    ocp.subjectTo( -0.7 <= v_x <= 0.7 );                                    % path constraint on the robot's velocity
    ocp.subjectTo( -140*pi/180 <= w_z <= 140*pi/180 );                      % path constraint on the robot's angular velocity
    
    %% Setting up the simulated process
    identity = acado.OutputFcn();
    dynamicSystem = acado.DynamicSystem(f, identity);
    process = acado.Process(dynamicSystem, 'INT_RK45');
    
    %% Setup of the algorithm and the tuning options
    algo = acado.RealTimeAlgorithm(ocp, Ts);
    algo.set( 'MAX_NUM_ITERATIONS', 3 );
    algo.set('HESSIAN_APPROXIMATION', 'GAUSS_NEWTON');
    algo.set('KKT_TOLERANCE', 1e-10);
    algo.set('INTEGRATOR_TOLERANCE', 1e-5);
    
    %% Setting up the NMPC controller
    reference = acado.StaticReferenceTrajectory( trajectory_reference() );
    controller = acado.Controller(algo, reference);
    
    %% Simulation Environment
    sim = acado.SimulationEnvironment(0, simulationTime, process, controller);
    % Initialize the states
    initial_states = [0 0 0];
    sim.init( initial_states );
    
END_ACADO;

out = nmpcMethods_RUN();

draw;