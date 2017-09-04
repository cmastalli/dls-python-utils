#!/usr/bin/python
# dwl modules
import dwl
import numpy as np
from dwl_msgs import BagParser as bag_parser
import roslib; roslib.load_manifest('dwl_msgs')

# dls-python-utils modules
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))) # import the classes inside the src folder
from src import array_utils

# matplotlib modules
import matplotlib.patches as mpatches
import matplotlib as mpl
import matplotlib.pyplot as plt



dwl_path = '/home/cmastalli/dls_ws/src/dls/dwl-distro/dwl/'
initial_time = 14.314 + 15
duration = 5 #25.
bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/ral18icra/sloppy_gap15cm/exp3/2017-07-26-20-11-28.bag'
topic = '/hyq/robot_states'

# Creating the hyq class and its floating-base model
relative_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
urdf = relative_path + '/models/hyq.urdf'
yarf = relative_path + '/models/hyq.yarf'
fbs = dwl.FloatingBaseSystem()
fbs.resetFromURDFFile(urdf, yarf)

# Extrating the whole-body trajectory
ws_vec = bag_parser.extractWholeBodyState(bag_file,
                                          topic,
                                          initial_time,
                                          duration)
                                          
time = array_utils.getTimeArray(ws_vec)
joint_vel = array_utils.getJointVelocityArray(ws_vec)
joint_eff = array_utils.getJointEffortArray(ws_vec)

power = 0.
for i in range(len(joint_eff)):
    for j in range(len(joint_eff[i])):
        dt = 0.
        if j > 1:
            t0 = time[j-1][0]
            tf = time[j][0]
            dt = tf - t0
        power += joint_eff[i][j] * joint_vel[i][j] * dt
weight = fbs.getGravityAcceleration() * fbs.getTotalMass()
travel_distance = np.linalg.norm(ws_vec[-1].getBasePosition_W() - ws_vec[0].getBasePosition_W())

print('The mechanical CoT is:', power[0] / (weight * travel_distance))


