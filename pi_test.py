import time
import utils.robot as robot

#Circle States
PATHING_CIRCLE = 1
PATHING_HOME = 2
TURNAROUND = 3
HOME = 4

show_animation = True

# -------------------Circle Data---------------------
circleRadii = [28, 42, 56, 70, 84, 98]
circleXY = [19.7990, 29.6985, 39.5980, 49.4975, 59.3970]
circleStartAngle = 0.7853981634
circleEndAngle = 5.497787144

# ------------------Key Positions in [x,y,theta]------------
homePos = [150, 0, 0]
targetPos = [19.7990, 19.7990, circleStartAngle]

####################################
####################################
#Error handling for are we off track
#Each step check if we are near the point we are supposed to be at
#If yes, continue
#If no, save the current index of spline, recreate the spline, start over main loop
#####################################
##################################

#Main Loop
#init()

bot = robot.Robot()

while(1):
     #Plan path
     bot.generate_path()
     bot.track_path()

     if(bot.end_time == True):
          break

print("Time ran out")