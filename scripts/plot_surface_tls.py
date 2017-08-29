# -*- coding: utf-8 -*-
"""
Created on Mon Aug 28 12:22:37 2017

@author: user
"""

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random
import dwl
from numpy.linalg import inv

# dls-python-utils modules
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))) # import the classes inside the src folder
from src import hyq, array_utils

# matplotlib modules
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter


def drange(start, stop, step):
     r = start
     while r < stop:
         yield r
         r += step



# Construct an instance of the WholeBody(Kinematics/Dynamics) classes, which wraps the C++ class.
ws = dwl.WholeBodyState()
fbs = dwl.FloatingBaseSystem()
wkin = dwl.WholeBodyKinematics()
wdyn = dwl.WholeBodyDynamics()

# set to True if you want to use constant torque limits (independent from the joint configuration)
# set to False if you want to use the DWS function that computes the max torque limits depending on the joints
use_urdf_tau_lims = False

# Resetting the robot model
relative_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
urdf = relative_path + '/models/hyq.urdf'
yarf = relative_path + '/models/hyq.yarf'
wdyn.modelFromURDFFile(urdf, yarf)
fbs = wdyn.getFloatingBaseSystem()
wkin = wdyn.getWholeBodyKinematics()
robot = hyq.HyQ(urdf, yarf)


# Computing the nominal posture
leg = 'lf_foot'
q_nom = fbs.getDefaultPosture()
c_nom_B = wkin.computePosition(np.zeros(6), q_nom,
                               fbs.getEndEffectorNames(dwl.FOOT),
                               dwl.Linear)
contact_pos_B = c_nom_B

tmp = dwl.FloatingBaseSystem()
lf_nom = np.array([[ 0.37077344], [ 0.32406699], [-0.57750958]])

# Studying the LF foot
#lf_pos_B = [[np] for z in range(10) for j in range(10) for i in range(10)]

res_x = 0.01; res_y = 0.01; res_z = 0.002
#leg_pos_B = dict([])
scaling_factor = 0.05
q_haa = [];  q_hfe = [];  q_kfe = [];
tau_haa_max = []; tau_hfe_max = []; tau_kfe_max = [];
px = []; py = []
lambda_x = []; lambda_y = []; lambda_z = []

joint_lim = fbs.getJointLimits()
q_lbound = np.array([fbs.getLowerLimit(joint_lim['lf_haa_joint']),
                     fbs.getLowerLimit(joint_lim['lf_hfe_joint']),
                     fbs.getLowerLimit(joint_lim['lf_kfe_joint'])])
q_ubound = np.array([fbs.getUpperLimit(joint_lim['lf_haa_joint']),
                     fbs.getUpperLimit(joint_lim['lf_hfe_joint']),
                     fbs.getUpperLimit(joint_lim['lf_kfe_joint'])])
                     
def fun(px, py):
    pz = lf_nom[2]
    lf_pos_B = np.array([[px],[py],[pz]])

    success = False
    q = np.zeros(fbs.getJointDoF())
    contact_pos_B['lf_foot'] = lf_pos_B
    force_1 = np.zeros(3)
    force_2 = np.zeros(3)
    force_3 = np.zeros(3)
    force_4 = np.zeros(3)
    if wkin.computeJointPosition(q, contact_pos_B, q_nom):
        success = True
        q_branch = fbs.getBranchState(q, 'lf_foot')
        
        if use_urdf_tau_lims:
            tau_1 = np.array([[120.0],[150.0],[150.0]]);
                                
            tau_2 = np.array([[120.0],[-150.0],[-150.0]]);
                                
            tau_3 = np.array([[120.0],[150.0],[-150.0]]);

            tau_4 = np.array([[120.0],[-150.0],[150.0]]);
        else:
            tau_1 = np.array([[robot.getTorqueLimit(0, q_branch[0])],
                                [robot.getTorqueLimit(1, q_branch[1])],
                                [robot.getTorqueLimit(2, q_branch[2])]]);
                                
            tau_2 = np.array([[robot.getTorqueLimit(0, q_branch[0])],
                                [-robot.getTorqueLimit(1, q_branch[1])],
                                [-robot.getTorqueLimit(2, q_branch[2])]]);
                                
            tau_3 = np.array([[robot.getTorqueLimit(0, q_branch[0])],
                           [robot.getTorqueLimit(1, q_branch[1])],
                           [-robot.getTorqueLimit(2, q_branch[2])]]);

            tau_4 = np.array([[robot.getTorqueLimit(0, q_branch[0])],
                           [-robot.getTorqueLimit(1, q_branch[1])],
                           [robot.getTorqueLimit(2, q_branch[2])]]);
        
        tau_haa_max.append(tau_1[0])
        tau_hfe_max.append(tau_1[1])
        tau_kfe_max.append(tau_1[2])
#            print(q_branch.transpose())            
        wrench_grav = np.zeros([6])
        tau_grav = np.zeros([12])
        no_grf = { 'lh_foot' : np.array([0., 0., 0., 0., 0., 0.]),
                       'rf_foot' : np.array([0., 0., 0., 0., 0., 0.]),
                       'lh_foot' : np.array([0., 0., 0., 0., 0., 0.]),
                       'rh_foot' : np.array([0., 0., 0., 0., 0., 0.]) };
        wdyn.computeInverseDynamics(wrench_grav, tau_grav,
                                        np.zeros(6), q,
                                        np.zeros(6), np.zeros(12),
                                        np.zeros(6), np.zeros(12),
                                        no_grf);
        lf_tau_grav = fbs.getBranchState(tau_grav, 'lf_foot')

        jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), 6 + fbs.getJointDoF()])
        fixed_jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), fbs.getJointDoF()])
        wkin.computeJacobian(jac,
                                 np.zeros(6), q,
                                 fbs.getEndEffectorNames(dwl.FOOT),
                                 dwl.Linear)

        wkin.getFixedBaseJacobian(fixed_jac, jac)
        lf_jac = fixed_jac[0:3,0:3]
        force_1 = inv(lf_jac.transpose()).dot(tau_1 - 0*lf_tau_grav)
        force_2 = inv(lf_jac.transpose()).dot(tau_2 - 0*lf_tau_grav)
        force_3 = inv(lf_jac.transpose()).dot(tau_3 - 0*lf_tau_grav)
        force_4 = inv(lf_jac.transpose()).dot(tau_4 - 0*lf_tau_grav)

    return success, force_1, force_2, force_3, force_4

res = 0.01

fx_1_list = []
fy_1_list = []
fz_1_list = []
fx_2_list = []
fy_2_list = []
fz_2_list = []
fx_3_list = []
fy_3_list = []
fz_3_list = []
fx_4_list = []
fy_4_list = []
fz_4_list = []
x_list = [] #[]
y_list = []
x = 0.; y = 0.

print tmp
for i in np.arange(-50.0, 50.0, 2.0):
    for j in np.arange(-50.0, 50.0, 1.0):
        x = i*res+lf_nom[0]
        y = j*res+lf_nom[1]
#        print x, y
        success, force_1, force_2, force_3, force_4 = fun(x[0],y[0])
        if success:
            x_list.append(x)
            y_list.append(y)
            fx_1_list.append(force_1[dwl.X])
            fy_1_list.append(force_1[dwl.Y])
            fz_1_list.append(force_1[dwl.Z])
            fx_2_list.append(force_2[dwl.X])
            fy_2_list.append(force_2[dwl.Y])
            fz_2_list.append(force_2[dwl.Z])
            fx_3_list.append(force_3[dwl.X])
            fy_3_list.append(force_3[dwl.Y])
            fz_3_list.append(force_3[dwl.Z])
            fx_4_list.append(force_4[dwl.X])
            fy_4_list.append(force_4[dwl.Y])
            fz_4_list.append(force_4[dwl.Z])

xv = np.array(x_list)

yv = np.array(y_list)
fx_1 = np.array(fx_1_list)
fz_1 = np.array(fz_1_list) 
fx_2 = np.array(fx_2_list)
fz_2 = np.array(fz_2_list) 
fx_3 = np.array(fx_3_list)
fz_3 = np.array(fz_3_list) 
fx_4 = np.array(fx_4_list)
fz_4 = np.array(fz_4_list) 

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xv, yv, fz_1, color='blue')
ax.scatter(xv, yv, fz_2, color='red')
ax.scatter(xv, yv, fz_3, color='yellow')
ax.scatter(xv, yv, fz_4, color='gray')

ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
ax.set_zlabel('fz[N]')

plt.show()