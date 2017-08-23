#!/usr/bin/python
import dwl
import numpy as np



def getTimeArray(ws_vec):
    return [[k.getTime()] for k in ws_vec]


def getBaseRPYArray(ws_vec):
    return [[k.getBaseRPY_W()[i] for k in ws_vec] for i in range(3)]


def getBasePositionArray(ws_vec):
    return [[k.getBasePosition_W()[i] for k in ws_vec] for i in range(3)]


def getBaseAngularVelocityArray(ws_vec):
    return [[k.getBaseAngularVelocity_W()[i] for k in ws_vec] for i in range(3)]


def getBaseVelocityArray(ws_vec):
    return [[k.getBaseVelocity_W()[i] for k in ws_vec] for i in range(3)]


def getBaseAngularAccelerationArray(ws_vec):
    return [[k.getBaseAngularAcceleration_W()[i] for k in ws_vec] for i in range(3)]


def getBaseAccelerationArray(ws_vec):
    return [[k.getBaseAcceleration_W()[i] for k in ws_vec] for i in range(3)]


def getJointPositionArray(ws_vec):
    num_joints = ws_vec[0].getJointDoF()
    return [[k.getJointPosition()[i] for k in ws_vec] for i in range(num_joints)]


def getJointVelocityArray(ws_vec):
    num_joints = ws_vec[0].getJointDoF()
    return [[k.getJointVelocity()[i] for k in ws_vec] for i in range(num_joints)]


def getJointAccelerationArray(ws_vec):
    num_joints = ws_vec[0].getJointDoF()
    return [[k.getJointAcceleration()[i] for k in ws_vec] for i in range(num_joints)]


def getJointEffortArray(ws_vec):
    num_joints = ws_vec[0].getJointDoF()
    return [[k.getJointEffort()[i] for k in ws_vec] for i in range(num_joints)]


def getContactPositionArray(ws_vec):
    contact_names = [c for c in ws_vec[0].getContactPosition_B()]
    contact_pos = dict([])
    for c in contact_names:
        contact_pos[c] = [[k.getContactPosition_B(c)[i] for k in ws_vec] for i in range(3)]
    return contact_pos


def getContactVelocityArray(ws_vec):
    contact_names = [c for c in ws_vec[0].getContactPosition_B()]
    contact_vel = dict([])
    for c in contact_names:
        contact_vel[c] = [[k.getContactVelocity_B(c)[i] for k in ws_vec] for i in range(3)]
    return contact_vel


def getContactAccelerationArray(ws_vec):
    contact_names = [c for c in ws_vec[0].getContactPosition_B()]
    contact_acc = dict([])
    for c in contact_names:
        contact_acc[c] = [[k.getContactAcceleration_B(c)[i] for k in ws_vec] for i in range(3)]
    return contact_acc


def getContactPositionArray(ws_vec):
    contact_names = [c for c in ws_vec[0].getContactPosition_B()]
    contact_wrc = dict([])
    for c in contact_names:
        contact_wrc[c] = [[k.getContactWrench_B(c)[i] for k in ws_vec] for i in range(6)]
    return contact_wrc
