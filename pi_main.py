import RPi.GPIO as GPIO
import time
import utils.generate_path as planner

def init():
    print("Initializing")

    # Don't Use GPIO.BOARD Pins 3,5,8,10,19,21,23,24,26,33

    # --------------Motor Setup---------------------
    # Use GPIO Numbers
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # Motor 1
    GPIO.setup(13, GPIO.OUT)
    GPIO.setup(15, GPIO.OUT)
    p1=GPIO.PWM(15,100)

    # Motor 2
    GPIO.setup(16, GPIO.OUT)
    GPIO.setup(18, GPIO.OUT)
    p2 = GPIO.PWM(18,100)

    # Start both motors with 0% DC
    p1.start(0)
    p2.start(0)

    # -------------Beam Break Sensor--------------
    GPIO.setup(11,GPIO.IN)
    #GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP) # if pull up resistor is needed
    if GPIO.input(11)!=True:
        #Beam Break Error Message here
        print("Couldn't find Beam Break")

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

        # Robot Initialization
        init()

        # Continuous Loop
        while(1):
                # Plan Pathing
                path = planner.generate_path(state)
                # Path Tracking
                while(1):
                        

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
