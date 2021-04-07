clear all
topics = rosbag('/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/bagfiles/gpOff_terramechanicsOff/pathTracking/truePose/DirectSingleShooting/change_Ts/0_05s/subset.bag');
topicsResample = rosbag('/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_viz/bagfiles/gpOff_terramechanicsOff/pathTracking/truePose/DirectSingleShooting/change_Ts/0_05s/subsetResample.bag');

%%  Raw Data

%   Pose
selPose = select(topics, 'Topic', '/pioneer3at/robotPose');
structPose = readMessages(selPose, 'DataFormat', 'struct');

X = cellfun( @(m) double( m.X ), structPose );
Y = cellfun( @(m) double( m.Y ), structPose );
Z = cellfun( @(m) double( m.Z ), structPose );
Roll = cellfun( @(m) double( m.Roll ), structPose );
Pitch = cellfun( @(m) double( m.Pitch ), structPose );
Yaw = cellfun( @(m) double( m.Yaw ), structPose );

%   Optimization time
selOptTime = select(topics, 'Topic', '/pioneer3at/optTime');
structOptTime = readMessages(selOptTime, 'DataFormat', 'struct');

optTime = cellfun( @(m) double( m.Data ), structOptTime );

%   Cycle time
selCycleTime = select(topics, 'Topic', '/pioneer3at/cycleTime');
structCycleTime = readMessages(selCycleTime, 'DataFormat', 'struct');

cycleTime = cellfun( @(m) double( m.Data ), structCycleTime );

%   Distance
selDistance = select(topics, 'Topic', '/pioneer3at/distance');
structDistance = readMessages(selDistance, 'DataFormat', 'struct');

distance = cellfun( @(m) double( m.Data ), structDistance );

%   Error
selError = select(topics, 'Topic', '/pioneer3at/error');
structError = readMessages(selError, 'DataFormat', 'struct');

errorX = cellfun( @(m) double( m.Data(1) ), structError );
errorY = cellfun( @(m) double( m.Data(2) ), structError );
errorTheta = cellfun( @(m) double( m.Data(3) ), structError );

%   Current step
selStep = select(topics, 'Topic', '/pioneer3at/currentStep');
structStep = readMessages(selStep, 'DataFormat', 'struct');

step = cellfun( @(m) double( m.Data ), structStep );

%   Current reference
selRef = select(topics, 'Topic', '/pioneer3at/currentRef');
structRef = readMessages(selRef, 'DataFormat', 'struct');

refX = cellfun( @(m) double( m.Data(1) ), structRef );
refY = cellfun( @(m) double( m.Data(2) ), structRef );
refTheta = cellfun( @(m) double( m.Data(3) ), structRef );

%   Current solution
selSol = select(topics, 'Topic', '/pioneer3at/currentSol');
structSol = readMessages(selSol, 'DataFormat', 'struct');

solX = cellfun( @(m) double( m.Data(1) ), structSol );
solY = cellfun( @(m) double( m.Data(2) ), structSol );
solTheta = cellfun( @(m) double( m.Data(3) ), structSol );

%   Commands
selCmd = select(topics, 'Topic', '/pioneer3at/cmd_vel');
structCmd = readMessages(selCmd, 'DataFormat', 'struct');

cmd_Vx = cellfun( @(m) double( m.Linear.X ), structCmd );
cmd_Wz = cellfun( @(m) double( m.Angular.Z ), structCmd );

%%  Resample
select = select(topicsResample, 'Topic', '/pioneer3at/resample');
struct = readMessages(select, 'DataFormat', 'struct');

%  Pose
resX = cellfun( @(m) double( m.Pose.X ), struct );
resY = cellfun( @(m) double( m.Pose.Y ), struct );
resZ = cellfun( @(m) double( m.Pose.Z ), struct );
resRoll = cellfun( @(m) double( m.Pose.Roll ), struct );
resPitch = cellfun( @(m) double( m.Pose.Pitch ), struct );
resYaw = cellfun( @(m) double( m.Pose.Yaw ), struct );

%  Step
resStep = cellfun( @(m) double( m.Step ), struct );

%  Solution
resSolX = cellfun( @(m) double( m.Solution(1) ), struct );
resSolY = cellfun( @(m) double( m.Solution(2) ), struct );
resSolTheta = cellfun( @(m) double( m.Solution(3) ), struct );

%  Reference
resRefX = cellfun( @(m) double( m.Reference(1) ), struct );
resRefY = cellfun( @(m) double( m.Reference(2) ), struct );
resRefTheta = cellfun( @(m) double( m.Reference(3) ), struct );

%   Simulation Time
resSimTime =  cellfun( @(m) double( m.Clock_ ), struct );

%  Cycle Time
resCycleTime = cellfun( @(m) double( m.CycleTime ), struct );

%  Optimization Time
resOptTime = cellfun( @(m) double( m.OptTime ), struct );

%  Actuations 
resCom_vx = cellfun( @(m) double( m.Actuation.Linear.X ), struct );
resCom_wz = cellfun( @(m) double( m.Actuation.Angular.Z ), struct );

%  Distance
resDistance = cellfun( @(m) double( m.Distance ), struct );

%  Error
resErrorX = cellfun( @(m) double( m.Error(1) ), struct );
resErrorY = cellfun( @(m) double( m.Error(2) ), struct );
resErrorTheta = cellfun( @(m) double( m.Error(3) ), struct );