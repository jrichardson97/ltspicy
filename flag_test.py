import RPi.GPIO as io
import time

io.setmode(io.BOARD)
io.setup(37, io.OUT)


io.output(37, io.HIGH)

time.sleep(10)
