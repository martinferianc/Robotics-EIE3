import threading
import time
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

Robot = Robot(interface, pid_config_file="paper_config.json")
for i in range(4):
    time.sleep(1)
    Robot.travel_straight(40)
    time.sleep(1)
    Robot.rotate_left(90)

interface.terminate()
