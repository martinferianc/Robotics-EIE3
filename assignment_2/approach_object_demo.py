import threading
import time
from src.robot import Robot
import brickpi
import sys

#initialize the interface
interface = brickpi.Interface()
interface.initialize()
config = str(sys.argv[1])
Robot = Robot(interface, pid_config_file=config)

Robot.approach_object(30,0)
time.sleep(0.5)

interface.terminate()
