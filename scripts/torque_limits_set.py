#!/usr/bin/python
# dwl modules
import dwl
import numpy as np
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



# Resetting the robot model
relative_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
urdf = relative_path + '/models/hyq.urdf'
yarf = relative_path + '/models/hyq.yarf'
wdyn.modelFromURDFFile(urdf, yarf)
fbs = wdyn.getFloatingBaseSystem()
wkin = wdyn.getWholeBodyKinematics()
robot = hyq.HyQ(urdf, yarf)







# Getting the joint limits
#joint_lim = fbs.getJointLimits()
#q_l = np.array([[fbs.getLowerLimit(joint_lim['lf_haa_joint'])],
#                [fbs.getLowerLimit(joint_lim['lf_hfe_joint'])],
#                [fbs.getLowerLimit(joint_lim['lf_kfe_joint'])]])
#q_u = np.array([[fbs.getUpperLimit(joint_lim['lf_haa_joint'])],
#                [fbs.getUpperLimit(joint_lim['lf_hfe_joint'])],
#                [fbs.getUpperLimit(joint_lim['lf_kfe_joint'])]])
#print(q_l)
#q_haa = []; q_hfe = []; q_kfe = []
#tau_haa = []; tau_hfe = []; tau_kfe = []
#lambda_x = []; lambda_y = []; lambda_z = []
#for haa in np.arange(q_l[0],q_u[0], 0.1):
#    for hfe in np.arange(q_l[1],q_u[1], 0.1):
#        for kfe in np.arange(q_l[2],q_u[2], 0.1):
#            q = np.array([haa,hfe,kfe])
##            print(q)
#            tau_max = np.array([[robot.getTorqueLimit(0, haa)],
#                                [robot.getTorqueLimit(1, hfe)],
#                                [robot.getTorqueLimit(2, kfe)]])
#
#            q_haa.append(haa)
#            q_hfe.append(hfe)
#            q_kfe.append(kfe)
#            tau_haa.append(tau_max[0])
#            tau_hfe.append(tau_max[1])
#            tau_kfe.append(tau_max[2])
#
##            print(q)
##            print(tau_max)
#            jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), 6 + fbs.getJointDoF()])
#            fixed_jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), fbs.getJointDoF()])
#            q_ext = np.array([haa,hfe,kfe,haa,hfe,kfe,haa,hfe,kfe,haa,hfe,kfe])
#            leg = "lf_foot"
#            wkin.computeJacobian(jac,
#                                 np.zeros(6), q_ext,
#                                 fbs.getEndEffectorNames(dwl.FOOT),
##                                 leg,
#                                 dwl.Linear)
#
#            wkin.getFixedBaseJacobian(fixed_jac, jac)
#            lf_jac = fixed_jac[0:3,0:3]
#            
#            
#            force = inv(lf_jac.transpose()).dot(tau_max)
#            lambda_x.append(force[0])
#            lambda_y.append(force[1])
#            lambda_z.append(force[2])
#
#
#
#
#fig, ax = plt.subplots(nrows=1, sharex=True)
##ax.plot(q_haa,tau_haa, 'r')
##ax.plot(q_hfe,tau_hfe, 'b')
##ax.plot(q_kfe,tau_kfe, 'g')
#
#ax.plot(q_haa, lambda_x, 'r')
##ax.plot(q_hfe, lambda_y, 'b')
##ax.plot(q_kfe, lambda_z, 'g')






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
print (q_lbound)
for i in range(-60,60,2):
    for j in range(-80,60,2):
        for k in range(0,1,1):
#            print i, j, k
            lf_pos_B = np.array([[i * res_x],
                                 [j * res_y],
                                 [k * res_z]]) + lf_nom
            px.append([lf_pos_B[dwl.X]])
            py.append([lf_pos_B[dwl.Y]])
#            print lf_pos_B
#            print c_nom_B['lf_foot']

            q = np.zeros(fbs.getJointDoF())
            contact_pos_B['lf_foot'] = lf_pos_B
            wkin.computeJointPosition(q, contact_pos_B, q_nom)
            q_branch = fbs.getBranchState(q, 'lf_foot')
            if q_branch[0] < q_lbound[0] and q_branch[0] > q_ubound[0]:
                lambda_x.append(0.0)
                lambda_y.append(0.0)
                lambda_z.append(0.0)
            elif q_branch[1] < q_lbound[1] and q_branch[1] > q_ubound[1]:
                lambda_x.append(0.0)
                lambda_y.append(0.0)
                lambda_z.append(0.0)
            elif q_branch[2] < q_lbound[2] and q_branch[2] > q_ubound[2]:
                lambda_x.append(0.0)
                lambda_y.append(0.0)
                lambda_z.append(0.0)                
            else:
                
                q_haa.append(q_branch[0])
                q_hfe.append(q_branch[1])
                q_kfe.append(q_branch[2])
            
#            print(lf_pos_B.transpose())
            
                tau_max = np.array([[robot.getTorqueLimit(0, q_branch[0])],
                                [robot.getTorqueLimit(1, q_branch[1])],
                                [robot.getTorqueLimit(2, q_branch[2])]]);
                tau_haa_max.append(tau_max[0])
                tau_hfe_max.append(tau_max[1])
                tau_kfe_max.append(tau_max[2])
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
            
            
                force = inv(lf_jac.transpose()).dot(tau_max - lf_tau_grav)
                lambda_x.append(force[dwl.X])
                lambda_y.append(force[dwl.Y])
                lambda_z.append(force[dwl.Z])

#print(q_branch_list[0])


#fig = plt.figure()
#ax = fig.gca(projection='3d')


#print len(py)
#print len(q_haa_ubound)
#px, py = np.meshgrid(px, py)


# Plot the surface.
#surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
#                       linewidth=0, antialiased=False)


# Plot the surface.
#surf = ax.plot_surface(px, py, lambda_z, cmap=cm.coolwarm,
#                       linewidth=0, antialiased=False)

#print px
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.scatter(px, py, lambda_z)#, c=c, marker=m)

ax.set_xlabel('px Label')
ax.set_ylabel('py Label')
ax.set_zlabel('fz Label')

# Add a color bar which maps values to colors.
#fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()


#fig = plt.figure(2)
#ax = fig.add_subplot(111, projection='3d')
#ax.scatter(lambda_x, lambda_y, lambda_z)#, c=c, marker=m)
#
#ax.set_xlabel('fx Label')
#ax.set_ylabel('fy Label')
#ax.set_zlabel('fy Label')

# Add a color bar which maps values to colors.
#fig.colorbar(surf, shrink=0.5, aspect=5)
#
#plt.show()



# Plotting the torques leg_name HFE
#fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, sharex=True, projection='3d')
#ax0.plot(q_haa_lbound, 'r', linewidth=2.)
#ax0.plot(q_haa_ubound, 'r', linewidth=2.)
#ax0.plot(q_haa, 'b', linewidth=2.)
#
#ax1.plot(q_hfe_lbound, 'r', linewidth=2.)
#ax1.plot(q_hfe_ubound, 'r', linewidth=2.)
#ax1.plot(q_hfe, 'b', linewidth=2.)
#
#ax2.plot(q_kfe_lbound, 'r', linewidth=2.)
#ax2.plot(q_kfe_ubound, 'r', linewidth=2.)
#ax2.plot(q_kfe, 'b', linewidth=2.)
#ax.plot(q_branch_list[0], q_lbound_list[0], '#ff8000', linewidth=2., label=r"lims")
#ax.plot(time, joint_ubound[fbs.getJointId(leg_name + '_hfe_joint')], '#ff8000', linewidth=2.)
#ax.plot(time, joint_eff[fbs.getJointId(leg_name + '_hfe_joint')], '#ff0000', linewidth=2., label=r"cmd")
#ax.axhspan(90., 150.,  color='b', alpha=0.15, lw=0)
#ax.axhspan(-90., -150.,  color='b', alpha=0.15, lw=0)
#ax.set_title('Joint torques', fontsize=18)
#ax.set_ylabel(r'KFE (Nm)', {'color':'k', 'fontsize':12})
#ax.grid(True)
#ax.set_ylim([-150, 150])
#ax.spines['right'].set_visible(False)
#ax.spines['top'].set_visible(False)
#plt.show()