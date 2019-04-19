#!/usr/bin/env python

"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

"""
import numpy as np
import math
""" import matplotlib.pyplot as plt """

k = 0.1  # look forward gain
Lfc = 0.1  # look-ahead distance
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s]
L = 0.2032  # [m] wheel base of vehicle

max_angle=60 # degrees


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    
    y_change = state.v * math.sin(state.yaw) * dt
    state.y = state.y + y_change

    new_yaw = state.yaw + state.v / L * math.tan(delta) * dt

    if(new_yaw>360):
        new_yaw-=360
    elif(new_yaw<0):
        new_yaw+=360

    state.yaw = new_yaw
    state.v = state.v + a * dt

    yaw_change = state.v / L * math.tan(delta) * dt

    return state, yaw_change


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy, pind)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    """ alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0) """

    #return delta, ind
    return ind


def calc_target_index(state, cx, cy, pind):
    #print("CX: " + str(cx))
    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    remaining=len(dx)-pind
    search_area=int(pind+math.floor(remaining/2))

    print(str(search_area))
    #print("REMAINING: " + str(remaining))

    if(search_area>10):
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx[:search_area], dy[:search_area])]
    else:
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    #print(str(d))
    ind = d.index(min(d))
    #print("Before Look Forward: " + str(ind))
    L = 0.0

    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    #print("X: " + str(state.x) + ",          Y: " + str(state.y))
    print("Index: " + str(ind))

    #print("After Look Ahead: " +str(ind))
    return ind
