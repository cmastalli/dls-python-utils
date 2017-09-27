#!/usr/bin/python
# dwl modules
import dwl
import numpy as np
from dwl_msgs import BagParser as bag_parser
import roslib; roslib.load_manifest('dwl_msgs')
import math

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
topic = '/hyq/robot_states'
########################### ICRA-15
# gap25cm
#initial_time = 6.9
#duration = 16.5
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/ijrr18/decoupled/gap25cm/2017-09-26-17-31-47.bag'

# stepping stones
#initial_time = 5
#duration = 37
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/ijrr18/decoupled/stepping_stones/2017-09-26-17-25-06.bag'
#
# stairs
initial_time = 27.5
duration = 15.5
bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/ijrr18/decoupled/stairs/2017-09-26-17-23-05.bag'
#
# pallet
initial_time = 5.6
duration = 0
bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/ijrr18/decoupled/pallet/2017-09-26-17-41-25.bag'


########################### ICRA-17
# gap25cm
#initial_time = 39.8
#duration = 13.1
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/icra17/gap25cm/9/2016-10-06-13-04-07_modified.bag'
#
## gap15cm
#initial_time = 43
#duration = 19
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/icra17/gap15cm/2016-09-03-14-37-11.bag'
#
## stepping-stones
#initial_time = 68
#duration = 19
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/icra17/stepping_stones/7/2016-10-17-13-30-12.bag'
#
## attitude modulation
#initial_time = 17
#duration = 8
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/icra17/attitude_modulation/1/2016-08-31-10-23-33.bag'


######################### RAL-18
# Two slopes: walk
#initial_time = 15
#duration = 19
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/ral18icra/two_slopes/2017-08-04-11-38-33.bag'
#
# Two slopes: trot
#initial_time = 14.3
#duration = 7
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/ral18icra/two_slopes_trot/2017-08-04-11-58-10.bag'

# Two slopes: trans
#initial_time = 15.4
#duration = 10
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/ral18icra/two_slopes_trans/exp1/2017-08-04-15-36-06.bag'

#initial_time = 14.314 + 15
#duration = 5 #25.
#bag_file = '/media/cmastalli/Maxtor/Documents/Experiments/ral18icra/sloppy_gap15cm/exp3/2017-07-26-20-11-28.bag'






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
base_vel = array_utils.getBaseVelocityArray(ws_vec)
joint_vel = array_utils.getJointVelocityArray(ws_vec)
joint_eff = array_utils.getJointEffortArray(ws_vec)

# Computing the total energy
total_energy = 0.
average_vel_x = 0.
average_vel_y = 0.
average_vel_z = 0.
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

print len(base_vel[2])
for i in range(len(base_vel[0])):
    average_vel_x += base_vel[dwl.X][i][0]
    average_vel_y += base_vel[dwl.Y][i][0]
    average_vel_z += base_vel[dwl.Z][i][0]

average_vel_x /= len(base_vel[0])
average_vel_y /= len(base_vel[0])
average_vel_z /= len(base_vel[0])
average_vel = math.sqrt(math.pow(average_vel_x,2)+math.pow(average_vel_y,2)+math.pow(average_vel_z,2))


# Computing the weight of the robot and the travel distance
weight = fbs.getGravityAcceleration() * fbs.getTotalMass()
travel_distance = np.linalg.norm(ws_vec[-1].getBasePosition_W() - ws_vec[0].getBasePosition_W())

# Computing the CoT
print('The travel distance is:', travel_distance)
print('The mechanical CoT is:', total_energy / (weight * travel_distance))
print('The average base velocity is:', average_vel)
print('The mechanical CoT / average velocity is:', total_energy / (weight * travel_distance * average_vel))

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
