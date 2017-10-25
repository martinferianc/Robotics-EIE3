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

while True:
	print(Robot.read_ultrasonic_sensor())
	time.sleep(0.5)

interface.stopLogging()
interface.terminate()
