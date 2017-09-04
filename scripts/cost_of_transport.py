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

# Getting the time, joint torque and velocity vectors   
time = array_utils.getTimeArray(ws_vec)
joint_vel = array_utils.getJointVelocityArray(ws_vec)
joint_eff = array_utils.getJointEffortArray(ws_vec)

# Computing the total energy
total_energy = 0.
power = [0.] * len(joint_eff[0])
for i in range(len(joint_eff)):
    for j in range(len(joint_eff[i])):
        dt = 0.
        if j > 1:
            t0 = time[j-1][0]
            tf = time[j][0]
            dt = tf - t0
        actual_power = joint_eff[i][j] * joint_vel[i][j]
        power[j] += actual_power
        total_energy += math.fabs(actual_power * dt)
        

# Computing the weight of the robot and the travel distance
weight = fbs.getGravityAcceleration() * fbs.getTotalMass()
travel_distance = np.linalg.norm(ws_vec[-1].getBasePosition_W() - ws_vec[0].getBasePosition_W())

# Computing the CoT
print('The mechanical CoT is:', total_energy / (weight * travel_distance))


# Plotting the power
fig, ax = plt.subplots(nrows=1, sharex=True)
ax.plot(time, power, 'b', linewidth=2.)
#ax.set_title('Robot power', fontsize=18)
ax.set_xlabel(r'time (s)', {'color':'k', 'fontsize':12})
ax.set_ylabel(r'Power (W)', {'color':'k', 'fontsize':12})
ax.grid(True)
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)
plt.show()
