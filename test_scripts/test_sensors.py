import time
from src.robot import Robot
import brickpi
import sys

bumpers[0] = Robot.get_bumper("left")
bumpers[1] = Robot.get_bumper("right")

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()
config = str(sys.argv[1])
Robot = Robot(interface, pid_config_file=config)
bumpers = [None,None]


speeds = [5,5]
zeros = [0,0]
Robot.set_speed(speeds)
Robot.start_threading()

while True:
    bumpers[0] = Robot.get_bumper("left")
    bumpers[1] = Robot.get_bumper("right")
    print(bumpers)
    print(Robot.get_distance())
    time.sleep(1)

interface.terminate()
