# Motor initialization----------------------------------------------------------
import RPi.GPIO as io

io.setmode(io.BOARD)

io.setwarnings(False)

left= 11
io.setup(left, io.OUT)

right= 12
io.setup(right, io.OUT)

io.output(left, False)
io.output(right, False)

io.cleanup()
