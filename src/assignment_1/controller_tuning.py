import threading
import time
from robot import Robot
import brickpi
import math

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()


#Initialize the Robot
Robot = Robot(interface)

angle = float(input("Enter a angle to rotate degrees: "))
radians = math.radians(angle)
Robot.calibrate(radians,angle)
print "Destination reached!"


interface.terminate()
