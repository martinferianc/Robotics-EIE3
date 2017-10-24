import threading
import time
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

Robot = Robot(interface, "paper_config.json")

Robot.travel_straight(40)
time.sleep(5)
Robot.travel_straight(-40)
time.sleep(5)
Robot.rotate_left(90)
time.sleep(5)
Robot.rotate_right(90)

interface.terminate()
