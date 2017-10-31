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


speeds = [5,5]
zeros = [0,0]
Robot.set_speed(speeds)
Robot.start_debugging()

while True:
    pass
interface.terminate()
