import threading
import time
from src.robot import Robot
import brickpi
import sys

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()
config = str(sys.argv[1])
Robot = Robot(interface, pid_config_file=config)
bumpers = [None,None]
speeds = [6,6]
zeros = [0,0]
Robot.set_speed(speeds)
#Robot.start_debugging()

while True:
    bumpers[0] = Robot.get_bumper("left")
    bumpers[1] = Robot.get_bumper("right")

    if bumpers[0] and bumpers[1]:
        Robot.stop()
        Robot.travel_straight(-3)
        Robot.rotate_right(30)
        Robot.set_speed([-x for x in speeds])

    if not bumpers[0] and bumpers[1]:
        Robot.stop()
        Robot.rotate_left(30)
        Robot.set_speed([-x for x in speeds])

    if bumpers[0] and not bumpers[1]:
        Robot.stop()
        Robot.rotate_right(30)
        Robot.set_speed([-x for x in speeds])

interface.terminate()
