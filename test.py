import utils.nav as nav
import time
import utils.generate_path as gen
import utils.drive_path as drive

PATHING_CIRCLE = 1
PATHING_HOME = 2
TURNAROUND = 3
HOME = 4
state = HOME

# -------------------Circle Data---------------------
currentCircle = 0
circleRadii = [28, 42, 56, 70, 84]
circleXY = [19.7990, 29.6985, 39.5980, 49.4975, 59.3970]
circleStartAngle = 0.7853981634
circleEndAngle = 5.497787144

# ------------------Key Positions in [x,y,theta]------------
homePos = [150, 0, 0]
targetPos = [19.7990, 19.7990, circleStartAngle]

start_time=time.time()
state=HOME
x=150
y=0
yaw=180
v=0

""" while(time.time()-start_time < 30):
    spline,state,currentCircle= gen.generate_path(state, x,y,currentCircle)

    drive.drive_path(spline,x,y,yaw,v) """


##########################################################################################################
#This section of code tests the nav.py function pathRoute
cx = [150, 42, 19.7990, 0, -19.7990, -28, -19.7990, 0, 19.7990]
cy = [0, 0, 19.7990, 28, 19.7990, 0, -19.7990, -28, -19.7990]



while(time.time()-start_time<30):
    x,y,yaw,v = nav.path_route(cx, cy, x, y, yaw, v)
    currentCircle+=1
    i=currentCircle

    if(currentCircle<=4):
        cx=[x,circleRadii[i],circleXY[i],0,-circleXY[i],-circleRadii[i],-circleXY[i],0,circleXY[i]]
        cy = [y, 0, circleXY[i], circleRadii[i], circleXY[i],0, -circleXY[i], -circleRadii[i], -circleXY[i]]
    else:
        cx = [x, circleRadii[0], circleXY[0], 0, -circleXY[0], -circleRadii[0], -circleXY[0], 0, circleXY[0]]
        cy = [y, 0, circleXY[0], circleRadii[0], circleXY[0], 0, -circleXY[0], -circleRadii[0], -circleXY[0]]