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

print("wall location?\n\t1:Left side\n\t2:Right side")
while True:
    try:
        wall_location = int(input())
        if(wall_location != 1 or wall_location != 2):
            raise Exception("not a valid wall location!")
        return
    except Exception, e:
        print("Unable to read input: {}".format(str(e)))

if(wall_location == 1):
    Robot.set_ultra_pose(90)
elif(wall_location == 2):
    Robot.set_ultra_pose(-90)
else
    raise Exception("Can not change Ultrasound pose!")

while True:
    Robot.keep_distance(30, 6, wall_location)
    time.sleep(1)

interface.terminate()
