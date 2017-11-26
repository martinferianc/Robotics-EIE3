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
              pid_config_file="speed_config.json",
              mcl=False,
              threading=False
              )

speed_l = float(input("Enter left wheel speed:"))
speed_r = float(input("Enter right wheel speed:"))
Robot.set_speed(speeds=[speed_l,speed_r],k=1)
raw_input("Press enter when max speed has been hit.")
Robot.start_obstacle_detection()
time.sleep(10)

Robot.set_speed(speeds=[0,0])
Robot.stop_threading()
interface.terminate()
raw_input("Press enter to terminate.")
