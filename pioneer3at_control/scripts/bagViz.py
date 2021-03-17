import bagpy
import pandas as pd
from bagpy import bagreader

b = bagreader('/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/bagfiles/subset.bag')

msgDistance = b.message_by_topic( topic = '/pioneer3at/distance' )
msgError = b.message_by_topic( topic = '/pioneer3at/error' )
msgPose = b.message_by_topic( topic = '/pioneer3at/robotPose' )
msgSimClock = b.message_by_topic( topic = '/pioneer3at/clock' )

###

distance = pd.read_csv( msgDistance )
distance.to_csv( 'distance.csv' )

###

errorX = pd.read_csv( msgError, usecols = [0, 3] )
errorY = pd.read_csv( msgError, usecols = [0, 4] )
errorTheta = pd.read_csv( msgError, usecols = [0, 5] ) 

errorX.to_csv( 'errorX.csv' )
errorY.to_csv( 'errorY.csv' )
errorTheta.to_csv( 'errorTheta.csv' )

###

poseX = pd.read_csv( msgPose, usecols = [0, 1] )
poseY = pd.read_csv( msgPose, usecols = [0, 2] )
poseZ = pd.read_csv( msgPose, usecols = [0, 3] )
poseRoll = pd.read_csv( msgPose, usecols = [0, 4] )
posePitch = pd.read_csv( msgPose, usecols = [0, 5] )
poseYaw = pd.read_csv( msgPose, usecols = [0, 6] )

poseX.to_csv( 'poseX.csv' )
poseY.to_csv( 'poseY.csv' )
poseZ.to_csv( 'poseZ.csv' )
poseRoll.to_csv( 'poseRoll.csv' )
posePitch.to_csv( 'posePitch.csv' )
poseYaw.to_csv( 'poseYaw.csv' )

###

simClock = pd.read_csv( msgSimClock )
simClock.to_csv( 'simClock.csv' )