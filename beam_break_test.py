import RPi.GPIO as io
import time

io.setmode(io.BOARD)
io.setup(40, io.IN)

#High/true equals no object
#Low/false equals object in between
beam_state=True
object_count=0


while(1):
    if (io.input(40) and beam_state==False):
        print("Object cleared")
        object_count+=1
        print("Object count: " + str(object_count))
        beam_state=True
        print("Beam State: " + str(beam_state))
    elif (not io.input(40) and beam_state==True):
        print("Object in the way")
        beam_state=False
        print("Beam State: " + str(beam_state))

    time.sleep(0.1)