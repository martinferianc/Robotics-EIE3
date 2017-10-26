import threading
import time
from src.robot import Robot
import brickpi
import sys

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

left_bumper_port = 2
right_bumper_port = 0
config = str(sys.argv[1])
Robot = Robot(interface, pid_config_file=config)
bumpers = [None,None]

while True:
    Robot.set_speed(1)
    bumpers[0] = Robot.read_touch_sensor(left_bumper_port)
    bumpers[1] = Robot.read_touch_sensor(right_bumper_port)

    if bumpers[0] and bumpers[1]:
        Robot.stop()
        Robot.travel_straight(-3)
        Robot.rotate_right(30)

    if not bumpers[0] and bumpers[1]:
        Robot.stop()
        Robot.rotate_left(30)

    if bumpers[0] and not bumpers[1]:
        Robot.stop()
        Robot.rotate_right(30)

interface.terminate()
