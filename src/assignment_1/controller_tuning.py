import threading
import time
from robot import Robot
import brickpi
import math

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()
interface.startLogging("motor_position_1.log")

#Initialize the Robot
Robot = Robot(interface)

while True:
    angle = float(input("Enter a angle to rotate degrees: "))
    angle = math.radians(angle)
    Robot.rotate(angle)
    print "Destination reached!"


interface.stopLogging()
interface.terminate()
