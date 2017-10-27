import threading
import time
from src.robot import Robot
import brickpi
import sys

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()
config = "carpet_config.json"
Robot = Robot(interface, pid_config_file=config)
speeds = [6,6]
Robot.set_speed(speeds)
#Robot.start_debugging()

while True:
    Robot.keep_distance(70)

interface.terminate()
