#!/usr/bin/python
import dwl
import numpy as np
import math


class HyQ():
    def __init__(self, urdf, yarf):
        self.a1 = 0.3219
        self.b1 = 0.045
        self.a2 = 0.3218
        self.b2 = 0.045
        self.eps11 = 0.1089
        self.eps21 = 0.1403
        self.eps22 = 0.1047
        self.fbs = dwl.FloatingBaseSystem()
        self.fbs.resetFromURDFFile(urdf, yarf)
        self.LF_HAA = self.fbs.getJointId('lf_haa_joint')
        self.LF_HFE = self.fbs.getJointId('lf_hfe_joint')
        self.LF_KFE = self.fbs.getJointId('lf_kfe_joint')
        self.LH_HAA = self.fbs.getJointId('lh_haa_joint')
        self.LH_HFE = self.fbs.getJointId('lh_hfe_joint')
        self.LH_KFE = self.fbs.getJointId('lh_kfe_joint')
        self.RF_HAA = self.fbs.getJointId('rf_haa_joint')
        self.RF_HFE = self.fbs.getJointId('rf_hfe_joint')
        self.RF_KFE = self.fbs.getJointId('rf_kfe_joint')
        self.RH_HAA = self.fbs.getJointId('rh_haa_joint')
        self.RH_HFE = self.fbs.getJointId('rh_hfe_joint')
        self.RH_KFE = self.fbs.getJointId('rh_kfe_joint')
        
    def length_sign_flip(self, joint_id):
        if joint_id == self.LF_HFE or joint_id == self.RF_HFE or joint_id == self.LF_KFE or joint_id == self.RF_KFE:
            return -1
        else:
            return 1

    def getCylinderLength(self, joint_id, theta):
        carg = 0.; c = 0.
        if joint_id == self.LF_HAA or joint_id == self.LH_HAA or joint_id == self.RF_HAA or joint_id == self.RH_HAA:
            return 0
        
        if joint_id == self.LF_KFE or joint_id == self.LH_KFE or joint_id == self.RF_KFE or joint_id == self.RH_KFE:
            carg = math.pi - self.length_sign_flip(joint_id) * theta - (self.eps21 + self.eps22)
            c = math.sqrt(self.a2 * self.a2 + self.b2 * self.b2 - 2 * self.a2 * self.b2 * math.cos(carg))
        else:
            carg = math.pi / 2.0 + self.length_sign_flip(joint_id) * theta + self.eps11
            c = math.sqrt(self.a1 * self.a1 + self.b1 * self.b1 - 2 * self.a1 * self.b1 * math.cos(carg))

        return (c - 0.280);

    def getCylinderLever(self, joint_id, theta):
        if joint_id == self.LF_HAA or joint_id == self.LH_HAA or joint_id == self.RF_HAA or joint_id == self.RH_HAA:
            return 0.

        c = self.getCylinderLength(joint_id, theta) + 0.280;

        lever = 0.
        if joint_id == self.LF_KFE or joint_id == self.LH_KFE or joint_id == self.RF_KFE or joint_id == self.RH_KFE:
            den = self.a2 * self.a2 + c * c - self.b2 * self.b2
            num = (2 * self.a2 * c)
            sinarg = math.acos(den / num)
            lever = self.a2 * math.sin(sinarg)
        else:
            den = self.a1 * self.a1 + c * c - self.b1 * self.b1
            num = 2 * self.a1 * c
            sinarg = math.acos(den / num)
            lever = self.a1 * math.sin(sinarg)
        
        return lever;
    
    def getTorqueLimits(self, q):
        max_actuator_effort = np.array([120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0])
        torque_lim = np.zeros(12)
        for i in range(12):
            if self.getCylinderLever(i, q[i]) == 0:
                torque_lim[i] = max_actuator_effort[i]
            else:
                torque_lim[i] = max_actuator_effort[i] * self.getCylinderLever(i, q[i])
        
        return torque_lim

    def getTorqueLimit(self, joint_id, theta):
        max_actuator_effort = np.array([120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0])
        torque_lim = 0.
        if self.getCylinderLever(joint_id, theta) == 0:
            torque_lim = max_actuator_effort[joint_id]
        else:
            torque_lim = max_actuator_effort[joint_id] * self.getCylinderLever(joint_id, theta)
        
        return torque_lim

