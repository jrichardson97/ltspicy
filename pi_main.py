import RPi.GPIO as GPIO


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
    GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP) # if pull up resistor is needed
    if GPIO.input(11)!=True:
        #Beam Break Error Message here
        print("Couldn't find Beam Break")