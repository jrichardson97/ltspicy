import random
import time
# import RPi.GPIO

# -------------------Global Variables-----------------
startTime = time.time()
beamState = False
collectedObjects = 0

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


# Request Position from Localization Brain
def getPosition():
    position = [random.randint(0,192),random.randint(0,192)]
    print("Current Position:" + str(position).strip('[]'))
    return position


# Update Current Path
def path(coordinates = []):
    print ("Path Updated. Navigating to " + str(coordinates[0])+ ", " + str(coordinates[1]))
    drive(1,1)
    return


# Send Drive Signal to Wheels
def drive(lSpeed, rSpeed):
    print ("Driving at a speed of L:" + str(lSpeed) +" & R:"+ str(rSpeed))

    # p1.ChangeDutyCycle(lSpeed)
    # p2.ChangeDutyCycle(rSpeed)
    return

def checkConditions():
    global startTime
    global beamState
    global collectedObjects

    # If less than 20 seconds remain
    if time.time() - startTime >= 10:
        print("Time's almost up. Returning Home")
        returnHome(time)
        return

    # If Beam Break State change
    # if beamState==False && GPIO.input(11)==True:
        # beamState = True
        # collectedObjects += 1
    # elif beamState==True && GPIO.input(11)==False:
        # beamState = False

    # If collectedObjects>=2:
        # returnHome(maxObjects)
        # collectedObjects = 0

    return


#############################################
######Look more into state variables#########
#############################################
def returnHome(state):
    global collectedObjects

    while True:
        currentPosition = getPosition()
        if currentPosition != [0,0]:
            path([0,0])
        else:
            print("Returned Home")
            break

    if state==0:
        raiseFlag()
        # Exit Code
    elif state==1:
        collectedObjects=0

    return

def raiseFlag():
    return


# -----------------Main Code-------------------------------------------------------------------
init()

# Running Loop
while True:
    currentPos = getPosition()

    path(currentPos)

    checkConditions()

    time.sleep(1)

