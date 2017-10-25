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
	print("Right sensor: {0}, Left sensor:{1}".format(Robot.read_touch_sensor(0), Robot.read_touch_sensor(2)))
	time.sleep(0.5)

interface.stopLogging()
interface.terminate()
