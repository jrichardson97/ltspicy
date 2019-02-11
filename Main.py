import utils.mainLib as nav
import utils.cubic_spline_planner as spline
import utils.generate_path as generate_path
import time, array, math, cmath
# import RPi.GPIO



# -------------------Function Definitions--------------------------------------------------
# Initialization
def init():
    print("Initializing")

    # Don't Use GPIO.BOARD Pins 3,5,8,10,19,21,23,24,26,33

    # --------------Motor Setup---------------------
    # Use GPIO Numbers
    # GPIO.setwarnings(False)
    # GPIO.setmode(GPIO.BOARD)

    # Motor 1
    # GPIO.setup(13, GPIO.OUT)
    # GPIO.setup(15, GPIO.OUT)
    # p1=GPIO.PWM(15,100)

    # Motor 2
    # GPIO.setup(16, GPIO.OUT)
    # GPIO.setup(18, GPIO.OUT)
    # p2 = GPIO.PWM(18,100)

    # Start both motors with 0% DC
    # p1.start(0)
    # p2.start(0)

    # -------------Beam Break Sensor--------------
    # GPIO.setup(11,GPIO.IN)
    # GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP) # if pull up resistor is needed
    # if GPIO.input(11)!=True:
        # Beam Break Error Message here

# Determines how to get to spcified coordinates and calls Drive function accordingly
def path(coordinates = []):
    print ("Path Updated. Navigating to " + str(coordinates[0])+ ", " + str(coordinates[1]))

    if(currentState==PATHING_AROUND_CIRCLE):
        speeds = nav.driveCircle(circleRadii[currentCircle])
        drive(speeds[0],speeds[1])
        return
    else:
        #Determine wheel speed alternative 
        return

# Send Drive Signal to Wheels
def drive(lSpeed, rSpeed):
    print ("Driving at a speed of L:" + str(lSpeed) +" & R:"+ str(rSpeed))

    # p1.ChangeDutyCycle(lSpeed)
    # p2.ChangeDutyCycle(rSpeed)

    checkConditions()
    return

#Checks the time, objects collected, and target point reached conditions
def checkConditions():
    global startTime
    global beamState
    global collectedObjects

    # If less than 20 seconds remain
    if (time.time() - startTime >= 160):
        print("Time's almost up. Returning Home")
        currentState=PATHING_HOME_TIME
        returnHome()
    else:
        # If Beam Break State change
        if beamState==False: # && GPIO.input(11)==True:
            beamState = True
            collectedObjects += 1
        elif beamState==True: #&& GPIO.input(11)==False:
            beamState = False

        if (collectedObjects>=2):
            currentState=PATHING_HOME_BAY
            returnHome()
            collectedObjects = 0
        elif (nav.checkCoord(targetPos)):
            navigate()

#Determine target position
def navigate():
    global currentCircle

    #If current state is pathing to a circle, switch to pathing around circle
    if (currentState==PATHING_TO_CIRCLE):
        print("Pathing around circle Now")
        currentState=PATHING_AROUND_CIRCLE
        targetPos=[circleXY[currentCircle],-circleXY[currentCircle],circleEndAngle]
        path(targetPos)
    
    #If current state is pathing around circle, increment current circle and path to next circle
    elif(currentState==PATHING_AROUND_CIRCLE):
        print("Pathing to next circle from previous circle now")
        currentState=PATHING_TO_CIRCLE
        currentCircle+=1
        targetPos=[circleXY[currentCircle],circleXY[currentCircle],circleEndAngle]
        path(targetPos)

    #If current state was resetting after dropping off, path to circle
    elif(currentState==RESETTING):
        print("Pathing to circle from home")
        currentState=RESETTING
        targetPos=targetPos=[circleXY[currentCircle],circleXY[currentCircle],circleEndAngle]
        path(targetPos)
    
    else:
        #Something fucked up
        return

def returnHome():
    targetPos=homePos

    while not(nav.checkCoord(homePos)):
        path(homePos)

    if (currentState==PATHING_HOME_BAY):
        dropOffReset()
    elif(currentState==PATHING_HOME_TIME):
        raiseFlag()

def raiseFlag():
    return

def dropOffReset():
    return

def main(): 
    # ------------------Conditional Variables-----------------
    startTime = time.time()
    beamState = False
    collectedObjects = 0

    # -------------------State Variables-----------------
    PATHING_CIRCLE = 1
    PATHING_HOME = 2
    TURNAROUND = 3
    HOME = 4
    state = PATHING_CIRCLE

    # -------------------Circle Data---------------------
    currentCircle = 0
    circleRadii = [28, 42, 56, 70, 84]
    circleXY = [19.7990,29.6985,39.5980,49.4975,59.3970]
    circleStartAngle = 0.7853981634
    circleEndAngle = 5.497787144

    # ------------------Key Positions in [x,y,theta]------------
    homePos=[150,0,0]
    targetPos=[19.7990,19.7990,circleStartAngle]
    # Robot Initialization
    init()


    # Continuous Loop
    while(1):
        # Plan Pathing
        path = generate_path.generate_path(state)
        # Path Tracking
        while(1):
            #Drive/Track

            #check Conditionals
                #If conditional met, break out

                #Out of time

                #Bay area full

                #Target Position Met
                break;

    # Generate initial path
    current_path = [];
    path_index = 0;
    while(1):
        # if time is met or if bay area full
            # state = PATHING_HOME
        break;
        


       





if __name__ == "__main__":
    main()