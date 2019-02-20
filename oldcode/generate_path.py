import utils.cubic_spline_planner as spline

circleRadii = [28, 42, 56, 70, 84, 98]
circleXY = [19.7990, 29.6985, 39.5980, 49.4975, 59.3970]


PATHING_CIRCLE = 1
PATHING_HOME = 2
TURNAROUND = 3
HOME = 4


def generate_path(state,x,y,c):
    
    #If at home, path back into circle
    if(state==HOME):
        state=PATHING_CIRCLE

        cx=[x,circleRadii[c+1],circleXY[c],0             ,-circleXY[c],-circleRadii[c],-circleXY[c],0              ,circleXY[c]]
        cy=[y,0               ,circleXY[c],circleRadii[c],circleXY[c] ,0              ,-circleXY[c],-circleRadii[c],-circleXY[c]]
    
    #If pathing circle, either return home or continue to next circle depending on the check states
    elif(state==PATHING_CIRCLE):
        #Check Bay Area and Timer State Here
        bayState = False
        timerState = False

        #If bay area is full or time is out
        if(bayState or timerState):
            state=PATHING_HOME

            cx[x,circleRadii[c+1],150]
            cy[y,0,0]

        #Continue to next circle
        else:
            c+=1

            cx = [x, circleXY[c], 0, -circleXY[c], -circleRadii[c], -circleXY[c], 0, circleXY[c]]
            cy = [y, circleXY[c], circleRadii[c], circleXY[c], 0, -circleXY[c], -circleRadii[c], -circleXY[c]]

    #If returned from circle, move to turnaround state
    elif(state==PATHING_HOME):
        state=TURNAROUND

        #Code here will back the robot straight out, do a 180 degree turn, and set HOME state

        

    spline_path = spline.Spline2D(cx, cy)

    return spline_path,state,c
