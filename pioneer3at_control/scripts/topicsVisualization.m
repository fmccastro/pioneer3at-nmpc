topics = rosbag('/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/subset_10.bag');

%%  Resample

select = select(topics, 'Topic', '/pioneer3at/resample');

struct = readMessages(select, 'DataFormat', 'struct');

%%  Pose

x = cellfun( @(m) double( m.Pose.X ), struct );
y = cellfun( @(m) double( m.Pose.Y ), struct );
z = cellfun( @(m) double( m.Pose.Z ), struct );
roll = cellfun( @(m) double( m.Pose.Roll ), struct );
pitch = cellfun( @(m) double( m.Pose.Pitch ), struct );
yaw = cellfun( @(m) double( m.Pose.Yaw ), struct );

%%  Step

step = cellfun( @(m) double( m.Step ), struct );

%%  Solution

solX = cellfun( @(m) double( m.Solution(1) ), struct );
solY = cellfun( @(m) double( m.Solution(2) ), struct );
solTheta = cellfun( @(m) double( m.Solution(3) ), struct );

%%  Reference

refX = cellfun( @(m) double( m.Reference(1) ), struct );
refY = cellfun( @(m) double( m.Reference(2) ), struct );
refTheta = cellfun( @(m) double( m.Reference(3) ), struct );

%%   Simulation Time

simTime =  cellfun( @(m) double( m.Clock_ ), struct );

%%  Cycle Time

cycleTime = cellfun( @(m) double( m.CycleTime ), struct );

%%  Optimization Time

optTime = cellfun( @(m) double( m.OptTime ), struct );

%%  Actuations 

com_vx = cellfun( @(m) double( m.Actuation.Linear.X ), struct );
com_wz = cellfun( @(m) double( m.Actuation.Angular.Z ), struct );

%%  Distance

distance = cellfun( @(m) double( m.Distance ), struct );

%%  Error

errorX = cellfun( @(m) double( m.Error(1) ), struct );
errorY = cellfun( @(m) double( m.Error(2) ), struct );
errorTheta = cellfun( @(m) double( m.Error(3) ), struct );