import threading
import time
from src.robot import Robot
import brickpi
import sys
"""
    argv[1]: 1 for wall on the left side
        2 for wall on the right side
"""
#Initialize the interface
interface=brickpi.Interface()
interface.initialize()
config = "carpet_config.json"
Robot = Robot(interface, pid_config_file=config)
speeds = [6,6]
wall_location = int(sys.argv[1])
Robot.set_speed(speeds)
#Robot.start_debugging()

if(wall_location == 1):
    Robot.set_ultra_pose(90)
elif(wall_location == 2):
    Robot.set_ultra_pose(-90)
else:
    raise Exception("Can not change Ultrasound pose!")

while True:
    Robot.keep_distance(30, 6, wall_location)
    time.sleep(0.5)

interface.terminate()
