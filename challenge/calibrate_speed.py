from __future__ import division
import threading
import time
from src.robot import Robot
from src.drawing import Map, Canvas
import brickpi


#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

Robot = Robot(interface,
              pid_config_file="carpet_config.json",
              mcl=False,
              threading=False
              )

Robot.set_speed(speeds=[6,6],k=1)
raw_input("Press enter when max speed has been hit.")
Robot.start_obstacle_detection()

Robot.stop()
Robot.stop_threading()
interface.terminate()
