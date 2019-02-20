import numpy as np
import matplotlib.pyplot as plt
import utils.pure_pursuit as tracker

target_speed = 60
show_animation=True


def drive_path(sp, x, y, yaw, v):
    #Take in a spline input
    s = np.arange(0, sp.s[-1], 1)

    #Creates x, y, yaw, and curvature arrays from spline
    cx, cy, cyaw, ck = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        cx.append(ix)
        cy.append(iy)
        cyaw.append(sp.calc_yaw(i_s))
        ck.append(sp.calc_curvature(i_s))

    #Determines initial state of tracker
    state = tracker.State(x, y, yaw, v)

    lastIndex = len(cx) - 1
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    target_ind = tracker.calc_target_index(state, cx, cy)

    while lastIndex > target_ind:
        #Determines new speed of the simulation based on previous speed and target speed
        ai = tracker.PIDControl(target_speed, state.v)

        #Delta and target index
        di, target_ind = tracker.pure_pursuit_control(state, cx, cy, target_ind)

        delta_to_wheels(di)
        state = tracker.update(state, ai, di)

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.xlim(-172, 172)
            plt.ylim(-172, 172)
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    fx = cx[lastIndex]
    fy = cy[lastIndex]
    fyaw = state.yaw
    fv = state.v

    return fx, fy, fyaw, fv
#Determine the next step to follow that path

#Send signal to the wheels to follow the next step

#Check Conditions

#Loop





def delta_to_wheels(di):

    print(str(di))