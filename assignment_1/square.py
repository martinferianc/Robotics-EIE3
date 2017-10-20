import threading
import time
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

Robot = Robot(interface)
for i in range(4):
    Robot.travel_straight(40)
    Robot.rotate_left(90)

interface.terminate()
