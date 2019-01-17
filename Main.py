import random
import time
# import RPi.GPIO

# -------------------Function Definitions--------------------------------------------------

# Initialization
def init():
    print("Initializing")
    # Use GPIO Numbers
    # GPIO.setwarnings(False)
    # GPIO.setmode(GPIO.BCM)

    # Motor 1
    # GPIO.setup(13, GPIO.OUT)
    # GPIO.setup(6, GPIO.OUT)
    # p1=GPIO.PWM(6,100)

    # Motor 2
    # GPIO.setup(26, GPIO.OUT)
    # GPIO.setup(19, GPIO.OUT)
    # p2 = GPIO.PWM(19,100)

    # Beam Break Sensor
    # GPIO.setup(27,GPIO.IN)


# Request Position from Localization Brain
def requestPos():
    position = [random.randint(0,192),random.randint(0,192)]
    return position


# Update Current Path
def updatePath(coordinates = []):
    print ("Path Updated. Navigating to " + str(coordinates[0])+ ", " + str(coordinates[1]))
    drive(1,1)
    return


# Send Drive Signal to Wheels
def drive(lSpeed, rSpeed):
    print ("Driving at a speed of L:" + str(lSpeed) +" & R:"+ str(rSpeed))

    # p1.ChangeDutyCycle(lSpeed)
    # p2.ChangeDutyCycle(rSpeed)
    return

def checkBeam():
    return

def returnHome():
    while True:
        currentPosition = requestPos()
        if currentPosition != [0,0]:
            updatePath([0,0])
        else:
            print("Returned Home")
            return


# -----------------Main Code-------------------------------------------------------------------
startTime = time.time()
init()
collectedObjects = 0

# Start both motors with 0% DC
# p1.start(0)
# p2.start(0)


# Running Loop
while True:
    # If less than 20 seconds remain
    if time.time()-startTime>=10:
        print("Time's almost up. Returning Home")
        returnHome()
        break

    currentPos = requestPos()
    print("Current Position:" +str(currentPos).strip('[]'))
    updatePath(currentPos)

    # if !GPIO.input(27):
        # collectedObjects+=1

    print(str(collectedObjects))
    if collectedObjects >= 2:
        returnHome()
        collectedObjects = 0

    collectedObjects += 1
    time.sleep(1)

