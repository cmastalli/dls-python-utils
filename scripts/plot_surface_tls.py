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

# set to True if you want to use constant torque limits (independent from the joint configuration)
# set to False if you want to use the DWS function that computes the max torque limits depending on the joints
use_urdf_tau_lims = True

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
    else:
        force_1 = np.array([np.nan, np.nan, np.nan])
        force_2 = np.array([np.nan, np.nan, np.nan])
        force_3 = np.array([np.nan, np.nan, np.nan])
        force_4 = np.array([np.nan, np.nan, np.nan])

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

ub_z_mat = np.zeros((len(interval_x), len(interval_y)))
lb_z_mat = np.zeros((len(interval_x), len(interval_y)))
x_vec = np.zeros(len(interval_x))
y_vec = np.zeros(len(interval_y))
print len(ub_z_mat)

iter_x = 0
iter_y = 0
for i in interval_x:

    for j in interval_y:
        
        x = i*res+lf_nom[0]
        y = j*res+lf_nom[1]
        y_vec[iter_y] = y       
#        print x, y
        success, force_1, force_2, force_3, force_4 = fun(x[0],y[0])
#        if success:
        ''' create the lists needed for scattered plotting '''
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
            
        f_x = np.array([force_1[dwl.X], force_2[dwl.X], force_3[dwl.X], force_4[dwl.X]])
        f_y = np.array([force_1[dwl.Y], force_2[dwl.Y], force_3[dwl.Y], force_4[dwl.Y]])
        f_z = np.array([force_1[dwl.Z], force_2[dwl.Z], force_3[dwl.Z], force_4[dwl.Z]])

        ub_x_list.append(np.amax(f_x))
        lb_x_list.append(np.amin(f_x))
        ub_y_list.append(np.amax(f_y))
        lb_y_list.append(np.amin(f_y))
        ub_z_list.append(np.amax(f_z))
        lb_z_list.append(np.amin(f_z))
        ''' create the matrices needed for surface plotting '''
        ub_z_mat[iter_x][iter_y] = np.amax(f_z)
        lb_z_mat[iter_x][iter_y] = np.amin(f_z)
        
        print iter_x, iter_y
        iter_y = iter_y+1
  
    x_vec[iter_x] = x
    iter_x = iter_x+1
    iter_y = 0

#        print ub_z_mat
        

xv = np.array(x_list)
yv = np.array(y_list)
print len(xv), len(yv), len(ub_z_mat)
fx_1 = np.array(fx_1_list)
fy_1 = np.array(fy_1_list)
fz_1 = np.array(fz_1_list) 
fx_2 = np.array(fx_2_list)
fy_2 = np.array(fy_2_list)
fz_2 = np.array(fz_2_list) 
fx_3 = np.array(fx_3_list)
fy_3 = np.array(fy_3_list)
fz_3 = np.array(fz_3_list) 
fx_4 = np.array(fx_4_list)
fy_4 = np.array(fy_4_list)
fz_4 = np.array(fz_4_list)
ub_x = np.array(ub_x_list) 
lb_x = np.array(lb_x_list)
ub_y = np.array(ub_y_list)
lb_y = np.array(lb_y_list)
ub_z = np.array(ub_z_list)
lb_z = np.array(lb_z_list)

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

''' uncomment here if you want to get a scattered plot of the collected data '''
#ax.scatter(xv, yv, fx_1, color='blue')
#ax.scatter(xv, yv, fx_2, color='red')
#ax.scatter(xv, yv, fx_3, color='yellow')
#ax.scatter(xv, yv, fx_4, color='gray')
#ax.scatter(xv, yv, ub_z, color='green')
#ax.scatter(xv, yv, lb_z, color='green')
#print len(xv), len(yv)
#ax.set_xlabel('x[m]')
#ax.set_ylabel('y[m]')
#ax.set_zlabel('fz[N]')
#ax.grid(True)
#
#plt.show()

#n_radii = 8
#n_angles = 36
#
## Make radii and angles spaces (radius r=0 omitted to eliminate duplication).
#radii = np.linspace(0.125, 1.0, n_radii)
#angles = np.linspace(0, 2*np.pi, n_angles, endpoint=False)
#
## Repeat all angles for each radius.
#angles = np.repeat(angles[..., np.newaxis], n_radii, axis=1)
#
## Convert polar (radii, angles) coords to cartesian (x, y) coords.
## (0, 0) is manually added at this stage,  so there will be no duplicate
## points in the (x, y) plane.
#x = np.append(0, (radii*np.cos(angles)).flatten())
#y = np.append(0, (radii*np.sin(angles)).flatten())
#
## Compute z to make the pringle surface.
#z = np.sin(-x*y)
##print z
#print len(x), len(y), len(z)
#fig = plt.figure()
#ax = fig.gca(projection='3d')
#ax.plot_trisurf(interval_x, interval_y, z, linewidth=0.2, antialiased=True)
#
#plt.show()

#fig = plt.figure()
#ax = fig.gca(projection='3d')
X, Y = np.meshgrid(x_vec, y_vec)
##R = np.sqrt(X**2 + Y**2)
Z = ub_z_mat

# Plot the surface.
''' uncomment here if you want to get a wireframe plot of the collected data '''
surf = ax.plot_wireframe(Y, X, ub_z_mat, rstride=1, cstride=1)
surf = ax.plot_wireframe(Y, X, lb_z_mat, rstride=1, cstride=1)
''' uncomment here if you want to get a surface plot of the collected data '''
#surf = ax.plot_surface(Y, X, Z, rstride=1, cstride=1)
plt.show()

