import cubic_spline_planner as spline
import pure_pursuit as tracker
import RPi.GPIO as io
import numpy as np
import math 
import time

# -------------------Circle Data---------------------
circleRadii = [28, 42, 56, 70, 84, 98]
circleXY = [19.7990, 29.6985, 39.5980, 49.4975, 59.3970]


#Pathing States
PATHING_CIRCLE = 1
PATHING_HOME = 2
TURNAROUND = 3
HOME = 4

class Robot():
    def __init__(self):
        print("Initializaing Robot")

        # Position Variables
        self.x = 150
        self.y = 0
        self.yaw = 180

        # Driving Variables
        self.target_power = 0.3
        self.target_velocity=60
        self.v = 0
        self.max_turning_angle = 70  # Going straight, >0=CCW, <0=CW
        self.min_turn_radius = 0.25  # [m]
        self.wheel_base = 20.32

        # Navigation Variables
        self.state = HOME
        self.circle_completion_state=False
        self.current_circle = 0
        self.spline = spline.Spline2D([0, 1], [0, 1])

        #Condition Variables
        self.object_count = 0
        self.beam_state=True
        self.end_time = False
        self.off_track = False
        self.start_time=time.time()

        #Sets the io scheme to be based on board pin numbers
        #Use io.BCM for GPIO based classifications
        io.setmode(io.BOARD)

        # Motor initialization----------------------------------------------------------
        self.PWM_MAX = 100
        io.setwarnings(False)

        self.leftMotor_DIR_pin = 15
        io.setup(self.leftMotor_DIR_pin, io.OUT)

        self.rightMotor_DIR_pin = 16
        io.setup(self.rightMotor_DIR_pin, io.OUT)

        io.output(self.leftMotor_DIR_pin, False)

        io.output(self.rightMotor_DIR_pin, False)

        self.leftMotor_PWM_pin = 11
        self.rightMotor_PWM_pin = 12

        io.setup(self.leftMotor_PWM_pin, io.OUT)
        io.setup(self.rightMotor_PWM_pin, io.OUT)

        # MAX Frequency 20 Hz
        self.leftMotorPWM = io.PWM(self.leftMotor_PWM_pin, 10000)
        self.rightMotorPWM = io.PWM(self.rightMotor_PWM_pin, 10000)

        self.leftMotorPWM.start(0)
        self.leftMotorPWM.ChangeDutyCycle(0)

        self.rightMotorPWM.start(0)
        self.rightMotorPWM.ChangeDutyCycle(0)

        self.leftMotorPower = 0
        self.rightMotorPower = 0

        # Beam Break Setup----------------------------------------------------------------
        io.setmode(io.BOARD)
        io.setup(40, io.IN)

    def getMotorPowers(self):
        return (self.leftMotorPower, self.rightMotorPower)

    def setMotorLeft(self, power):

        # SetMotorLeft(power)

        # Sets the drive level for the left motor, from +1 (max) to -1 (min).

        # This is a short explanation for a better understanding:
        # SetMotorLeft(0)     -> left motor is stopped
        # SetMotorLeft(0.75)  -> left motor moving forward at 75% power
        # SetMotorLeft(-0.5)  -> left motor moving reverse at 50% power
        # SetMotorLeft(1)     -> left motor moving forward at 100% power

        if power < 0:
            # Reverse mode for the left motor
            io.output(self.leftMotor_DIR_pin, False)
            pwm = -int(self.PWM_MAX * power)
            if pwm > self.PWM_MAX:
                pwm = self.PWM_MAX
        elif power > 0:
            # Forward mode for the left motor
            io.output(self.leftMotor_DIR_pin, True)
            pwm = int(self.PWM_MAX * power)
            if pwm > self.PWM_MAX:
                pwm = self.PWM_MAX
        else:
            # Stopp mode for the left motor
            io.output(self.leftMotor_DIR_pin, False)
            pwm = 0
        #	print "SetMotorLeft", pwm
        self.leftMotorPower = pwm
        self.leftMotorPWM.ChangeDutyCycle(pwm)

    def setMotorRight(self, power):

        # SetMotorRight(power)

        # Sets the drive level for the right motor, from +1 (max) to -1 (min).

        # This is a short explanation for a better understanding:
        # SetMotorRight(0)     -> right motor is stopped
        # SetMotorRight(0.75)  -> right motor moving forward at 75% power
        # SetMotorRight(-0.5)  -> right motor moving reverse at 50% power
        # SetMotorRight(1)     -> right motor moving forward at 100% power

        if power < 0:
            # Reverse mode for the right motor
            io.output(self.rightMotor_DIR_pin, True)
            pwm = -int(self.PWM_MAX * power)
            if pwm > self.PWM_MAX:
                pwm = self.PWM_MAX
        elif power > 0:
            # Forward mode for the right motor
            io.output(self.rightMotor_DIR_pin, False)
            pwm = int(self.PWM_MAX * power)
            if pwm > self.PWM_MAX:
                pwm = self.PWM_MAX
        else:
            # Stopp mode for the right motor
            io.output(self.rightMotor_DIR_pin, False)
            pwm = 0
        #	print "SetMotorRight", pwm
        self.rightMotorPower = pwm
        self.rightMotorPWM.ChangeDutyCycle(pwm)

    def exit(self):
        # Program will clean up all GPIO settings and terminates
        io.output(self.leftMotor_DIR_pin, False)
        io.output(self.rightMotor_DIR_pin, False)
        io.cleanup()

    def generate_path(self):
        x = self.x
        y = self.y
        c = self.current_circle
        #print("Current Circle: "+ str(c))
        state = self.state

        #If returned from circle, move to turnaround state
        if(state == PATHING_HOME):
            print("Arrived home, turning arround")
            self.state = TURNAROUND
            self.object_count=0
            return
        elif(state==TURNAROUND):
            print("Turned around, currently at home preparing to path back out")
            self.x = 150
            self.y = 0
            self.yaw = 180
            self.state=HOME
            return
        #Else state is either HOME or PATHING_CIRCLE
        else:
            #If at home, path back into circle
            if(state == HOME):
                if(self.end_time == True):
                    return

                if (c >= 5):
                    c = 4

                print("Pathing out of home to Circle: " + str(self.current_circle))
                self.state = PATHING_CIRCLE

                cx = [x, circleRadii[c+1], circleXY[c], 0, -circleXY[c], -
                      circleRadii[c], -circleXY[c], 0, circleXY[c]]
                cy = [y, 0, circleXY[c], circleRadii[c], circleXY[c],
                      0, -circleXY[c], -circleRadii[c], -circleXY[c]]

            #If pathing circle, either return home or continue to next circle depending on the check states
            elif(state == PATHING_CIRCLE):
                #If bay area is full or time is out
                if(self.object_count >= 3 or self.end_time == True):
                    print("Bay area full or time ran out, returning home")
                    self.state = PATHING_HOME

                    if (c>=5):
                        c=4
                        
                    cx=[x, circleRadii[c+1], 150]
                    cy=[y, 0, 0]

                    self.current_circle += 1

                #Continue to next circle
                else:
                    target_circle=c+1
                    if(target_circle <= 4 and not self.circle_completion_state):
                        print("Pathing from Circle: " + str(self.current_circle) +" around Circle: " + str(target_circle))
                        cx = [x, circleRadii[c], circleXY[c], 0, -circleXY[c], -
                              circleRadii[c], -circleXY[c], 0, circleXY[c]]
                        cy = [y, 0, circleXY[c], circleRadii[c], circleXY[c],
                              0, -circleXY[c], -circleRadii[c], -circleXY[c]]

                        self.current_circle += 1
                    else:
                        if (not self.circle_completion_state):
                            print("All circles pathed, now looping around Circle: 0")
                        self.circle_completion_state=True
                        self.current_circle=0

                        cx = [x, circleRadii[0], circleXY[0], 0, -circleXY[0], -
                              circleRadii[0], -circleXY[0], 0, circleXY[0]]
                        cy = [y, 0, circleXY[0], circleRadii[0], circleXY[0],
                              0, -circleXY[0], -circleRadii[0], -circleXY[0]]
            
            self.spline = spline.Spline2D(cx, cy)

    def track_path(self):
        x = self.x
        y = self.y
        yaw = self.yaw
        v = self.v
        sp = self.spline

        #s = a series of steps of 1 from 0 to the end of the spline
        #Each step is 1 cm
        s = np.arange(0, sp.s[-1], 1)

        #Creates x, y, yaw, and curvature arrays from spline
        cx, cy, cyaw, ck = [], [], [], []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            cx.append(ix)
            cy.append(iy)
            cyaw.append(sp.calc_yaw(i_s))
            ck.append(sp.calc_curvature(i_s))

        #Determines initial state of tracker
        state = tracker.State(x, y, yaw, v)

        #Finds end of the spline
        lastIndex = len(cx) - 1

        #Calculates the first target index of the spline
        target_ind = tracker.calc_target_index(state, cx, cy)

        while lastIndex > target_ind:

            #self.target_velocity=self.determine_velocity()

            #Determines new speed of the simulation based on previous speed and target speed
            ai = tracker.PIDControl(self.target_velocity, state.v)

            #Delta and target index
            di, target_ind = tracker.pure_pursuit_control(
                state, cx, cy, target_ind)

            state, yaw_change = tracker.update(state, ai, di)

            self.drive_path(yaw_change)

            #Will eventually be change bc these will be determined by actual position insted of simulated ones
            self.x = cx[lastIndex]
            self.y = cy[lastIndex]
            self.yaw = state.yaw
            self.v = state.v

            if(not self.object_count>=3):
                self.check_beam()

            if(time.time()-self.start_time>60):
                self.end_time=True

    def drive_path(self, r):
        d=r*180/math.pi

        pf_shift = d*0.0111111111111111111111111

        left_pf = 1-pf_shift
        right_pf = 1+pf_shift

        """ print("Left: " + str(self.target_power*left_pf))
        print("Right: " + str(self.target_power*right_pf)) """

        self.setMotorLeft(self.target_power*left_pf)
        self.setMotorRight(-self.target_power*right_pf)
        
        time.sleep(0.1)
        
    def check_beam(self):
        if (io.input(40) and self.beam_state == False):
            print("Object cleared")
            self.object_count+=1
            print("Object count: " + str(self.object_count))
            self.beam_state=True
            print("Beam State: " + str(self.beam_state))
        elif (not io.input(40) and self.beam_state==True):
            print("Object in the way")
            self.beam_state=False
            print("Beam State: " + str(self.beam_state))

    def determine_velocity(self):
        if(self.state==PATHING_HOME):
            #Code to slow down as appoaching home
            slow="yes"
        
        # Any other time we need to slow down???
        # Gotta go fast bitch
