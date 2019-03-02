import cubic_spline_planner as spline
import pure_pursuit as tracker
import RPi.GPIO as io
import numpy as np
import math 

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
        self.target_velocity = 60
        self.v = 0
        self.turning_angle = 0  # Going straight, >0=CCW, <0=CW
        self.min_turn_radius = 0.25  # [m]
        self.wheel_base = 20.32

        # Navigation Variables
        self.state = HOME
        self.circle_completion_state=False
        self.targetCircle = 0
        self.spline = spline.Spline2D([0, 1], [0, 1])


        #Condition Variables
        self.object_count = 0
        self.beam_state=True
        self.end_time = False
        self.off_track = False
        

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
        c = self.targetCircle
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
                print("Pathing out of home to Circle: " + str(self.targetCircle))
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

                    cx=[x, circleRadii[c+1], 150]
                    cy=[y, 0, 0]

                #Continue to next circle
                else:
                    if(c <= 4 and not self.circle_completion_state):
                        print("Pathing from Circle: " + str(self.targetCircle) +" to Circle: " + str(self.targetCircle+1))
                        cx = [x, circleRadii[c], circleXY[c], 0, -circleXY[c], -
                              circleRadii[c], -circleXY[c], 0, circleXY[c]]
                        cy = [y, 0, circleXY[c], circleRadii[c], circleXY[c],
                              0, -circleXY[c], -circleRadii[c], -circleXY[c]]

                        self.targetCircle += 1
                    else:
                        if (not self.circle_completion_state):
                            print("All circles pathed, now looping around Circle: 0")
                        self.circle_completion_state=True
                        self.targetCircle=0

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

        #S = a series of steps of 1 from 0 to the end of the spline
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

        
        lastIndex = len(cx) - 1
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        target_ind = tracker.calc_target_index(state, cx, cy)

        while lastIndex > target_ind:
            #Determines new speed of the simulation based on previous speed and target speed
            ai = tracker.PIDControl(self.target_velocity, state.v)

            #Delta and target index
            di, target_ind = tracker.pure_pursuit_control(
                state, cx, cy, target_ind)

            state, yaw_change, y_change = tracker.update(state, ai, di)

            self.drive_path(yaw_change, y_change)

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)

            # Test
            assert lastIndex >= target_ind, "Cannot goal"

            self.x = cx[lastIndex]
            self.y = cy[lastIndex]
            self.yaw = state.yaw
            self.v = state.v

            self.check_beam()

    def drive_path(self, yaw_c, y_c):
        """ print("Yaw: "  + str(self.yaw))
        print("Change: " +str(yaw_c))
        print("Velocity: " + str(self.v)) """

        R = y_c/math.atan(yaw_c)

        #print(str(R))

        """ if(yaw_c>0):
            rVel = self.v*(1-(self.wheel_base/2*R))
            lVel = self.v*(1+(self.wheel_base/2*R))
        elif(yaw_c<0):
            rVel = self.v*(1+(self.wheel_base/2*R))
            lVel = self.v*(1-(self.wheel_base/2*R))
        elif(yaw_c==0):
            rVel=self.v
            lVel=self.v

        print("Left Velocity: " + str(lVel))
        print("Right Velocity: " + str(rVel)) """

        #self.setMotorLeft(delta)

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
