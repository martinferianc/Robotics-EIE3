import threading
import time
import sys
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()
interface.startLogging("motor_position_1.log")

Robot = Robot(interface)


Robot.move_wheels([float(sys.argv[1]),float(sys.argv[2])])

interface.stopLogging()
interface.terminate()
