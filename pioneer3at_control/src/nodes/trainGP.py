#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import rospy, tf, sys, time

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

from multiprocessing import Process, Queue
from sys import path
path.append(r"/home/fmccastro/Tese_RoverNavigation/ROS_workspaces/wsPy3/src/pioneer3at_control/src")

from std_msgs.msg import Int32, Float64, Float64MultiArray
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from classes import Common

from pioneer3at_control.msg import pose3D
from pioneer3at_control.srv import getPath

if __name__ == 'main':
    common = Common()