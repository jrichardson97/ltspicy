import random, math

speedDesired=60
wheelRadius=20.32
axelLength=20.32


def driveCircle(radius):
    angularVel=speedDesired/radius

    rightVel=angularVel*(radius+axelLength/2)
    leftVel=angularVel*(radius-axelLength/2)

    return leftVel, rightVel

def rotate(dir, angle):
    #Clockwise
    if (dir==0):
        return
    #Counter clockwise
    elif(dir==1):
        return

# Request Position from Localization Brain
def getPosition():
    position = [random.randint(-172,172),random.randint(-172,-172),random.random(0,6.28318529)]
    print("Current Position:" + str(position).strip('[]'))
    return position

    #Check provided coordinates against current postion
def checkCoord(p2):
    p1=getPosition()
    distance=math.sqrt(pow((p1[0]-p2[0]),2)+pow((p1[1]-p2[1]),2))
    if distance<1:
        return True
    else:
        return False