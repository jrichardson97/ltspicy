import utils.cubic_spline_planner as planner
import utils.pure_pursuit as tracker
import numpy as np
import matplotlib.pyplot as plt

def path_route(cx,cy,x,y,yaw,v):
    k = 0.1  # look forward gain
    Lfc = 1.0  # look-ahead distance
    Kp = 1.0  # speed propotional gain
    dt = 0.1  # [s]
    L = 2.9  # [m] wheel base of vehicle


    show_animation = True

    #cx = [150, 42, 19.7990, 0, -19.7990, -28, -19.7990, 0, 19.7990]
    #cy = [0, 0, 19.7990, 28, 19.7990, 0, -19.7990, -28, -19.7990]
    ds = 1  # [m] distance of each intepolated points

    sp = planner.Spline2D(cx, cy)
    s = np.arange(0, sp.s[-1], ds)

    cx, cy, cyaw, ck = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        cx.append(ix)
        cy.append(iy)
        cyaw.append(sp.calc_yaw(i_s))
        ck.append(sp.calc_curvature(i_s))

    #Notes on Spline output
    #cx and cy are the various x,y postions along the spline, to be sent to the tracker calculations
    #cyaw is the angle theta in radians from the positive x axis
    #ck is the curvature of the path

    target_speed = 60  # [cm/s]

    # initial state
    state = tracker.State(x, y, yaw, v)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = tracker.calc_target_index(state, cx, cy)

    while lastIndex > target_ind:
        ai = tracker.PIDControl(target_speed, state.v)
        di, target_ind = tracker.pure_pursuit_control(state, cx, cy, target_ind)
        state = tracker.update(state, ai, di)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.xlim(-172, 172)
            plt.ylim(-172,172)
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    fx=cx[lastIndex]
    fy=cy[lastIndex]
    fyaw=state.yaw
    fv=state.v

    return fx,fy,fyaw,fv
