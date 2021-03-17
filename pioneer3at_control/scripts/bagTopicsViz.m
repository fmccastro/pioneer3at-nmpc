%%  Distance
distance = readtable( 'distance.csv' );
tsDistance = timeseries( distance.data, distance.Time );

%%  Error
errorTheta = readtable( 'errorTheta.csv' );
tsErrorTheta = timeseries( errorTheta.data_2, errorTheta.Time );

errorY = readtable( 'errorY.csv' );
tsErrorY = timeseries( errorY.data_1, errorY.Time );

errorX = readtable( 'errorX.csv' );
tsErrorX = timeseries( errorX.data_0, errorX.Time );

%%  Pose
poseX = readtable( 'poseX.csv' );
tsPoseX = timeseries( poseX.x, poseX.Time );

poseY = readtable( 'poseY.csv' );
tsPoseY = timeseries( poseY.y, poseY.Time );


poseZ = readtable( 'poseZ.csv' );
tsPoseZ = timeseries( poseZ.z, poseZ.Time );


poseRoll = readtable( 'poseRoll.csv' );
tsPoseRoll = timeseries( poseRoll.roll, poseRoll.Time );

posePitch = readtable( 'posePitch.csv' );
tsPosePitch = timeseries( posePitch.pitch, posePitch.Time );

poseYaw = readtable( 'poseYaw.csv' );
tsPoseYaw = timeseries( poseYaw.yaw, poseYaw.Time );

%%  Simulation Time

simTime = readtable( 'simClock.csv' );
tsSimTime = timeseries( simTime.data, simTime.Time );

%%  Plots
plot(
