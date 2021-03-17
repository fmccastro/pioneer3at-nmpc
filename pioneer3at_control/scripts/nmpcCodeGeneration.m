addpath('/home/fmccastro/Thesis_RoverNavigation/casADi/casadi-linux-matlabR2014b-v3.5.2')
import casadi.*

%%

Ts = 0.1;                               %   Sampling Time
N = 60;                                 %   Control Intervals

T = N * Ts;                             %   Time Horizon

x = MX.sym( 'x' );                      %   States
y = MX.sym( 'y' );
theta = MX.sym( 'theta' );

state = [x; y; theta];

V_x = MX.sym( 'V_x' );                  %   Controls
W_z = MX.sym( 'W_z' );

controls = [ V_x; W_z ];

%%

ODE = [ cos(theta) * V_x; sin(theta) * V_x; W_z ];      %   A-priori model

F = Function( 'F', {state, controls}, {ODE}, {'state', 'controls'}, {'ODE'} );

%%

%   Integrator to discretize system
intg_options = struct;
intg_options.tf = Ts;
intg_options.simplify = true;
intg_options.number_of_finite_elements = 4;

%   DAE problem structure
dae = struct;
dae.x = state;
dae.p = controls;
dae.ode = F(state, controls);

intg = integrator( 'intg', 'rk', dae, intg_options );

res = intg('x0', state, 'p', controls);
state_next = res.xf;

F = Function( 'F', {state, controls}, {state_next}, {'state', 'controls'}, {'state_next'} ); 

%%
sim = F.mapaccum(N);

x0 = [0; 0; 0];

u( 1, 1:N ) = 0.7;
u( 2, 1:N ) = 0;
res = sim( x0, u );

figure
tgrid = linspace( 0, T, N + 1 );
plot(tgrid, full( [ x0 res] ) );
legend( 'x', 'y', 'theta');
xlabel('Time [s]');

%%

U = MX.sym( 'U', 2, N );

X = sim( x0, U );

J = jacobian( X(1, :), U );

size( J );

figure

spy(J);

figure

Jf = Function( 'Jf', {U}, {J} );
full( Jf(0) );
imshow( full( Jf(0) ) );

%%
opti = casadi.Opti();

state = opti.variable( 3, N + 1 );                                          %   Decision variables
controls = opti.variable( 2, N );

initialState = opti.parameter( 3, 1 );                                      %   Parameter (initial state)
state_ref = opti.parameter(3, N );
controls_ref = opti.parameter( 2, N );
Q = opti.parameter(1, 1);
P = opti.parameter(1, 1);

opti.minimize( sumsqr( Q * [ state( : , 2 : N + 1 ) - state_ref( : , 1 : N ); controls - controls_ref ] ) + ...
                    sumsqr( P * [ state( : , N + 1) - state_ref( : , N ) ] ) );

for k = 1:N
    opti.subject_to( state( : , k + 1 ) == F( state( : , k), controls( : , k ) ) ); 
end

opti.subject_to( -0.7 <= controls( 1, :) <= 0.7 );
opti.subject_to( -140*pi/180 <= controls( 2, : ) <= 140*pi/180 );
opti.subject_to( state( : , 1) == initialState );

%   Choose a concrete solver
opti.solver( 'sqpmethod', struct('qpsol', 'qrqp') );

%   Choose a concrete value for p
opti.set_value( initialState, [ 0; 0; 0 ] );
opti.set_value( state_ref, repmat( [ 10.0; 10.0; 0.0 ], 1, N ) );
opti.set_value( controls_ref, repmat( [ 0.7; 0.0 ], 1, N ) );
opti.set_value( Q, 0.8 );
opti.set_value( P, 0.1 );

sol = opti.solve();

%%
figure
hold on

tgrid = linspace(0, T, N + 1);
plot( tgrid, sol.value(state) );
stairs( tgrid, [ sol.value(controls( 1, : )), NaN ], '-.' );
stairs( tgrid, [ sol.value(controls( 2, : )), NaN ], '-.' );
xlabel( 'Time [s]' );
legend( 'x', 'y', 'theta', 'V_x', 'W_z' );

figure
hold on
plot( sol.value( state( 1, : ) ), sol.value( state( 2, : ) ) );

%% 
M = opti.to_function( 'M', { initialState, state_ref, controls_ref, Q, P }, { controls( : , 1 ) },...
                                    { 'new_state', 'x_ref', 'u_ref', 'W', 'WN' }, { 'u_opt' } );
       
% NMPC loop
States_log = [];
Controls_log = [];

x = [0; 0; 0];

for i = 1 : 2 * N
    fprintf('\n\n Iteration : %d \n\n', i);
    u = full( M( x, repmat( [ 10.0; 10.0; 0.0 ], 1, N ), repmat( [ 0.7; 0.0 ], 1, N ), 0.8, 0.1 ) );
    
    States_log( :, i ) = x;
    Controls_log( :, i ) = u;
    
    %   Simulate system
    x = full( F( x, u ) );
end

figure
hold on

tgrid_nmpc = linspace(0, 2 * T, 2 * N + 1 );
plot( tgrid_nmpc, [ x0, States_log ] );
stairs( tgrid_nmpc, [ Controls_log( 1, : ) NaN ], 'LineStyle', '-.', 'Color', 'r' );
stairs( tgrid_nmpc, [ Controls_log( 2, : ) NaN ], 'LineStyle', '--', 'Color', 'k' );
xlabel('Time [s]');
legend( 'x', 'y', 'theta', 'V_x', 'W_z' );

figure
hold on
plot( States_log( 1, : ), States_log( 2, : ) );
xlabel('X[m]');
ylabel('Y[m]');

%%
M.generate( 'codegen_demo', struct( 'mex', true, 'cpp', false, 'with_header', true, 'with_mem', true, 'main', true ) );