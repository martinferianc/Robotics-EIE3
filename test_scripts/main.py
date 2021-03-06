import threading
import time
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()
interface.startLogging("motor_position_1.log")

Robot = Robot(interface)
Robot.move_wheels()

interface.stopLogging()
interface.terminate()
