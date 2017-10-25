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


Robot.set_ultra_pose(float(sys.argv[1]))

interface.stopLogging()
interface.terminate()
