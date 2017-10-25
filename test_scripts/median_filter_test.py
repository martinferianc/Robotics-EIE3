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
last_result = 255
while True:
	result = Robot.median_filtered_ultrasonic()
	#if (result!=last_result):
	print(result)
	last_result = result
	time.sleep(0.5)
interface.stopLogging()
interface.terminate()
