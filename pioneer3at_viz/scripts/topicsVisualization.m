topics = rosbag('/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/bagfiles/subset.bag');

%%  Robot Pose
poseSel = select( topics, 'Topic', '/pioneer3at/robotPose' );
poseStruct = readMessages( poseSel, 'DataFormat', 'struct' );

xPoints = cellfun( @(m) double(m.X), poseStruct );
yPoints = cellfun( @(m) double(m.Y), poseStruct );

%%  Simulation Time
simTimeSel = select( topics, 'Topic', '/pioneer3at/clock' );
simTimeStruct = readMessages( simTimeSel, 'DataFormat', 'struct' );

simTimeData = cellfun( @(m) double(m.Data), simTimeStruct );

%%  Optimization Time
optSel = select( topics, 'Topic', '/pioneer3at/optTime' );
optStruct = readMessages( optSel, 'DataFormat', 'struct' );

optiData = cellfun( @(m) double(m.Data), optStruct );

%%  Reference
refSel = select( topics, 'Topic', '/pioneer3at/currentRef' );
refStruct = readMessages( refSel, 'DataFormat', 'struct' );

refX_Data = cellfun( @(m) double(m.Data(1)), refStruct );
refY_Data = cellfun( @(m) double(m.Data(2)), refStruct );

%%  Commands
comSel = select( topics, 'Topic', '/pioneer3at/cmd_vel' );
comStruct = readMessages( comSel, 'DataFormat', 'struct' );

com_VX_Data = cellfun( @(m) double(m.Linear.X), comStruct );
com_WZ_Data = cellfun( @(m) double(m.Angular.Z), comStruct );

%%  Clock Time
clockSel = select( topics, 'Topic', '/clock' );
clockStruct = readMessages( clockSel, 'DataFormat', 'struct' );

clockData = cellfun( @(m) double( m.Clock_.Sec + m.Clock_.Nsec*10^(-9) ), clockStruct );

%%  Error with respect to reference
errorSel = select( topics, 'Topic', '/pioneer3at/error' );
errorStruct = readMessages( errorSel, 'DataFormat', 'struct' );

errorXData = cellfun( @(m) double( m.Data(1) ), errorStruct );
errorYData = cellfun( @(m) double( m.Data(2) ), errorStruct );
errorThetaData = cellfun( @(m) double( m.Data(3) ), errorStruct );

%%  Distance covered by the robot
distanceSel = select( topics, 'Topic', '/pioneer3at/distance' );
distanceStruct = readMessages( distanceSel, 'DataFormat', 'struct' );

distanceData = cellfun( @(m) double( m.Data ), distanceStruct );

%%  Plots
figure
title('Reference');
plot(refX_Data, refY_Data);

figure
title('Pose');
plot(xPoints, yPoints);
