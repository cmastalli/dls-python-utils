# -*- coding: utf-8 -*-
"""
Created on Sat Sep  2 16:23:41 2017

@author: user
"""

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
import matplotlib.tri as mtri

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
#
#''' set to true if you reset want to make a com'''
#use_constant_jac = False

#''' set to True if you want to use constant torque limits (independent from the joint configuration)
# set to False if you want to use the DWL function that computes the max torque limits depending on the joints '''
#use_urdf_tau_lims = False

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
#tau_haa_max = []; tau_hfe_max = []; tau_kfe_max = [];
px = []; py = []
lambda_x = []; lambda_y = []; lambda_z = []

joint_lim = fbs.getJointLimits()
q_lbound = np.array([fbs.getLowerLimit(joint_lim['lf_haa_joint']),
                     fbs.getLowerLimit(joint_lim['lf_hfe_joint']),
                     fbs.getLowerLimit(joint_lim['lf_kfe_joint'])])
q_ubound = np.array([fbs.getUpperLimit(joint_lim['lf_haa_joint']),
                     fbs.getUpperLimit(joint_lim['lf_hfe_joint']),
                     fbs.getUpperLimit(joint_lim['lf_kfe_joint'])])
                     
def fun(px, py, use_constant_jac, use_urdf_tau_lims):
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

            tau_5 = np.array([[-120.0],[150.0],[150.0]]);
                                
            tau_6 = np.array([[-120.0],[-150.0],[-150.0]]);
                                
            tau_7 = np.array([[-120.0],[150.0],[-150.0]]);

            tau_8 = np.array([[-120.0],[-150.0],[150.0]]);
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
            
            tau_5 = np.array([[-robot.getTorqueLimit(0, q_branch[0])],
                                [robot.getTorqueLimit(1, q_branch[1])],
                                [robot.getTorqueLimit(2, q_branch[2])]]);
                                
            tau_6 = np.array([[-robot.getTorqueLimit(0, q_branch[0])],
                                [-robot.getTorqueLimit(1, q_branch[1])],
                                [-robot.getTorqueLimit(2, q_branch[2])]]);
                                
            tau_7 = np.array([[-robot.getTorqueLimit(0, q_branch[0])],
                           [robot.getTorqueLimit(1, q_branch[1])],
                           [-robot.getTorqueLimit(2, q_branch[2])]]);

            tau_8 = np.array([[-robot.getTorqueLimit(0, q_branch[0])],
                           [-robot.getTorqueLimit(1, q_branch[1])],
                           [robot.getTorqueLimit(2, q_branch[2])]]);
        
#        tau_haa_max.append(tau_1[0])
#        tau_hfe_max.append(tau_1[1])
#        tau_kfe_max.append(tau_1[2])
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

        ''' set to true if you want to use a constant jacobian'''
        if use_constant_jac:
#            q_nom = fbs.getDefaultPosture()
            q_defult_lf = fbs.getBranchState(q_nom, 'lf_foot')
            jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), 6 + fbs.getJointDoF()])
            fixed_jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), fbs.getJointDoF()])
            wkin.computeJacobian(jac,
                                 np.zeros(6), q_nom,
                                 fbs.getEndEffectorNames(dwl.FOOT),
                                 dwl.Linear)

            wkin.getFixedBaseJacobian(fixed_jac, jac)
            lf_jac = fixed_jac[0:3,0:3]
        else:
            jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), 6 + fbs.getJointDoF()])
            fixed_jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), fbs.getJointDoF()])
            wkin.computeJacobian(jac,
                                 np.zeros(6), q,
                                 fbs.getEndEffectorNames(dwl.FOOT),
                                 dwl.Linear)

            wkin.getFixedBaseJacobian(fixed_jac, jac)
            lf_jac = fixed_jac[0:3,0:3]

        force_1 = inv(lf_jac.transpose()).dot(tau_1 - lf_tau_grav)
        force_2 = inv(lf_jac.transpose()).dot(tau_2 - lf_tau_grav)
        force_3 = inv(lf_jac.transpose()).dot(tau_3 - lf_tau_grav)
        force_4 = inv(lf_jac.transpose()).dot(tau_4 - lf_tau_grav)
        force_5 = inv(lf_jac.transpose()).dot(tau_1 - lf_tau_grav)
        force_6 = inv(lf_jac.transpose()).dot(tau_2 - lf_tau_grav)
        force_7 = inv(lf_jac.transpose()).dot(tau_3 - lf_tau_grav)
        force_8 = inv(lf_jac.transpose()).dot(tau_4 - lf_tau_grav)
    else:
        force_1 = np.array([np.nan, np.nan, np.nan])
        force_2 = np.array([np.nan, np.nan, np.nan])
        force_3 = np.array([np.nan, np.nan, np.nan])
        force_4 = np.array([np.nan, np.nan, np.nan])
        force_5 = np.array([np.nan, np.nan, np.nan])
        force_6 = np.array([np.nan, np.nan, np.nan])
        force_7 = np.array([np.nan, np.nan, np.nan])
        force_8 = np.array([np.nan, np.nan, np.nan])

    return success, force_1, force_2, force_3, force_4, force_5, force_6, force_7, force_8


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
ub_x_list = []
lb_x_list = []
ub_y_list = []
lb_y_list = []
ub_z_list = []
lb_z_list = []
x_list = [] #[]
y_list = []
x = 0.; y = 0.

print tmp
interval_x = np.arange(-50.0, 50.0, 5.0)
interval_y = np.arange(-50.0, 50.0, 5.0)

ub_z_mat_a = np.zeros((len(interval_x), len(interval_y)))
lb_z_mat_a = np.zeros((len(interval_x), len(interval_y)))
ub_z_mat_b = np.zeros((len(interval_x), len(interval_y)))
lb_z_mat_b = np.zeros((len(interval_x), len(interval_y)))
ub_z_mat_c = np.zeros((len(interval_x), len(interval_y)))
lb_z_mat_c = np.zeros((len(interval_x), len(interval_y)))

ub_x_mat_a = np.zeros((len(interval_x), len(interval_y)))
lb_x_mat_a = np.zeros((len(interval_x), len(interval_y)))
ub_x_mat_b = np.zeros((len(interval_x), len(interval_y)))
lb_x_mat_b = np.zeros((len(interval_x), len(interval_y)))
ub_x_mat_c = np.zeros((len(interval_x), len(interval_y)))
lb_x_mat_c = np.zeros((len(interval_x), len(interval_y)))

ub_y_mat_a = np.zeros((len(interval_x), len(interval_y)))
lb_y_mat_a = np.zeros((len(interval_x), len(interval_y)))
ub_y_mat_b = np.zeros((len(interval_x), len(interval_y)))
lb_y_mat_b = np.zeros((len(interval_x), len(interval_y)))
ub_y_mat_c = np.zeros((len(interval_x), len(interval_y)))
lb_y_mat_c = np.zeros((len(interval_x), len(interval_y)))
x_vec = np.zeros(len(interval_x))
y_vec = np.zeros(len(interval_y))
print len(ub_z_mat_a)

iter_x = 0
iter_y = 0
for i in interval_x:

    for j in interval_y:
        
        x = i*res+lf_nom[0]
        y = j*res+lf_nom[1]
        y_vec[iter_y] = y       

        ''' case A) compute GRFs with const jacobian and constant tau limits from urdf'''
        success, force_1_a, force_2_a, force_3_a, force_4_a, force_5_a, force_6_a, force_7_a, force_8_a = fun(x[0],y[0], True, True)
        
        ''' case B) compute GRFs with const jacobian and real tau limits'''
        success, force_1_b, force_2_b, force_3_b, force_4_b, force_5_b, force_6_b, force_7_b, force_8_b = fun(x[0],y[0], True, False)
        
        ''' case C) compute GRFs with real jacobian and real tau limits'''
        success, force_1_c, force_2_c, force_3_c, force_4_c, force_5_c, force_6_c, force_7_c, force_8_c = fun(x[0],y[0], False, False)
        
        ''' case D) compute GRFs with real jacobian and constant tau limits from urdf'''
        success, force_1, force_2, force_3, force_4, force_5, force_6, force_7, force_8 = fun(x[0],y[0], False, True)
#        if success:
        ''' create the lists needed for scattered plotting '''
#            
        f_x_a = np.array([force_1_a[dwl.X], force_2_a[dwl.X], force_3_a[dwl.X], force_4_a[dwl.X], force_5_a[dwl.X], force_6_a[dwl.X], force_7_a[dwl.X], force_8_a[dwl.X]])
        f_y_a = np.array([force_1_a[dwl.Y], force_2_a[dwl.Y], force_3_a[dwl.Y], force_4_a[dwl.Y], force_5_a[dwl.Y], force_6_a[dwl.Y], force_7_a[dwl.Y], force_8_a[dwl.Y]])
        f_z_a = np.array([force_1_a[dwl.Z], force_2_a[dwl.Z], force_3_a[dwl.Z], force_4_a[dwl.Z], force_5_a[dwl.Z], force_6_a[dwl.Z], force_7_a[dwl.Z], force_8_a[dwl.Z]])
        
        f_x_b = np.array([force_1_b[dwl.X], force_2_b[dwl.X], force_3_b[dwl.X], force_4_b[dwl.X], force_5_b[dwl.X], force_6_b[dwl.X], force_7_b[dwl.X], force_8_b[dwl.X]])
        f_y_b = np.array([force_1_b[dwl.Y], force_2_b[dwl.Y], force_3_b[dwl.Y], force_4_b[dwl.Y], force_5_b[dwl.Y], force_6_b[dwl.Y], force_7_b[dwl.Y], force_8_b[dwl.Y]])
        f_z_b = np.array([force_1_b[dwl.Z], force_2_b[dwl.Z], force_3_b[dwl.Z], force_4_b[dwl.Z], force_5_b[dwl.Z], force_6_b[dwl.Z], force_7_b[dwl.Z], force_8_b[dwl.Z]])

        f_x_c = np.array([force_1_c[dwl.X], force_2_c[dwl.X], force_3_c[dwl.X], force_4_c[dwl.X], force_5_c[dwl.X], force_6_c[dwl.X], force_7_c[dwl.X], force_8_c[dwl.X]])
        f_y_c = np.array([force_1_c[dwl.Y], force_2_c[dwl.Y], force_3_c[dwl.Y], force_4_c[dwl.Y], force_5_c[dwl.Y], force_6_c[dwl.Y], force_7_c[dwl.Y], force_8_c[dwl.Y]])
        f_z_c = np.array([force_1_c[dwl.Z], force_2_c[dwl.Z], force_3_c[dwl.Z], force_4_c[dwl.Z], force_5_c[dwl.Z], force_6_c[dwl.Z], force_7_c[dwl.Z], force_8_c[dwl.Z]])
        
        ''' create the matrices needed for surface plotting '''
        ub_z_mat_a[iter_x][iter_y] = np.amax(f_z_a)
        lb_z_mat_a[iter_x][iter_y] = np.amin(f_z_a)
        ub_z_mat_b[iter_x][iter_y] = np.amax(f_z_b)
        lb_z_mat_b[iter_x][iter_y] = np.amin(f_z_b)
        ub_z_mat_c[iter_x][iter_y] = np.amax(f_z_c)
        lb_z_mat_c[iter_x][iter_y] = np.amin(f_z_c)
        
        ub_x_mat_a[iter_x][iter_y] = np.amax(f_x_a)
        lb_x_mat_a[iter_x][iter_y] = np.amin(f_x_a)
        ub_x_mat_b[iter_x][iter_y] = np.amax(f_x_b)
        lb_x_mat_b[iter_x][iter_y] = np.amin(f_x_b)
        ub_x_mat_c[iter_x][iter_y] = np.amax(f_x_c)
        lb_x_mat_c[iter_x][iter_y] = np.amin(f_x_c)
        
        ub_y_mat_a[iter_x][iter_y] = np.amax(f_y_a)
        lb_y_mat_a[iter_x][iter_y] = np.amin(f_y_a)
        ub_y_mat_b[iter_x][iter_y] = np.amax(f_y_b)
        lb_y_mat_b[iter_x][iter_y] = np.amin(f_y_b)
        ub_y_mat_c[iter_x][iter_y] = np.amax(f_y_c)
        lb_y_mat_c[iter_x][iter_y] = np.amin(f_y_c)        
        
#        print iter_x, iter_y
        iter_y = iter_y+1
  
    x_vec[iter_x] = x
    iter_x = iter_x+1
    iter_y = 0


xv = np.array(x_list)
yv = np.array(y_list)

''' Plot FZ '''
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

''' uncomment here if you want to get a scattered plot of the collected data '''
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
ax.set_zlabel('fz[N]')
ax.grid(True)
X, Y = np.meshgrid(x_vec, y_vec)

# Plot the surface.
''' uncomment here if you want to get a wireframe plot of the collected data '''
surf = ax.plot_surface(Y, X, ub_z_mat_a, rstride=1, cstride=1, color = 'b', alpha = 0.3)
surf = ax.plot_surface(Y, X, lb_z_mat_a, rstride=1, cstride=1, color = 'b', alpha = 0.3)
surf = ax.plot_surface(Y, X, ub_z_mat_b, rstride=1, cstride=1, color = 'r', alpha = 0.3)
surf = ax.plot_surface(Y, X, lb_z_mat_b, rstride=1, cstride=1, color = 'r', alpha = 0.3)
surf = ax.plot_surface(Y, X, ub_z_mat_c, rstride=1, cstride=1, color = 'g', alpha = 0.3)
surf = ax.plot_surface(Y, X, lb_z_mat_c, rstride=1, cstride=1, color = 'g', alpha = 0.3)
''' uncomment here if you want to get a surface plot of the collected data '''
#surf = ax.plot_surface(Y, X, Z, rstride=1, cstride=1)
plt.show()

''' Plot FX '''
fig = plt.figure(2)
ax = fig.add_subplot(111, projection='3d')

''' uncomment here if you want to get a scattered plot of the collected data '''
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
ax.set_zlabel('fz[N]')
ax.grid(True)
X, Y = np.meshgrid(x_vec, y_vec)

# Plot the surface.
''' uncomment here if you want to get a wireframe plot of the collected data '''
surf = ax.plot_surface(Y, X, ub_x_mat_a, rstride=1, cstride=1, color = 'b', alpha = 0.3)
surf = ax.plot_surface(Y, X, lb_x_mat_a, rstride=1, cstride=1, color = 'b', alpha = 0.3)
surf = ax.plot_surface(Y, X, ub_x_mat_b, rstride=1, cstride=1, color = 'r', alpha = 0.3)
surf = ax.plot_surface(Y, X, lb_x_mat_b, rstride=1, cstride=1, color = 'r', alpha = 0.3)
surf = ax.plot_surface(Y, X, ub_x_mat_c, rstride=1, cstride=1, color = 'g', alpha = 0.3)
surf = ax.plot_surface(Y, X, lb_x_mat_c, rstride=1, cstride=1, color = 'g', alpha = 0.3)
''' uncomment here if you want to get a surface plot of the collected data '''
#surf = ax.plot_surface(Y, X, Z, rstride=1, cstride=1)
plt.show()


''' Plot FY '''
fig = plt.figure(3)
ax = fig.add_subplot(111, projection='3d')

''' uncomment here if you want to get a scattered plot of the collected data '''
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
ax.set_zlabel('fz[N]')
ax.grid(True)
X, Y = np.meshgrid(x_vec, y_vec)

# Plot the surface.
''' uncomment here if you want to get a wireframe plot of the collected data '''
surf = ax.plot_surface(Y, X, ub_y_mat_a, rstride=1, cstride=1, color = 'b', alpha = 0.3)
surf = ax.plot_surface(Y, X, lb_y_mat_a, rstride=1, cstride=1, color = 'b', alpha = 0.3)
surf = ax.plot_surface(Y, X, ub_y_mat_b, rstride=1, cstride=1, color = 'r', alpha = 0.3)
surf = ax.plot_surface(Y, X, lb_y_mat_b, rstride=1, cstride=1, color = 'r', alpha = 0.3)
surf = ax.plot_surface(Y, X, ub_y_mat_c, rstride=1, cstride=1, color = 'g', alpha = 0.3)
surf = ax.plot_surface(Y, X, lb_y_mat_c, rstride=1, cstride=1, color = 'g', alpha = 0.3)
''' uncomment here if you want to get a surface plot of the collected data '''
#surf = ax.plot_surface(Y, X, Z, rstride=1, cstride=1)
plt.show()
