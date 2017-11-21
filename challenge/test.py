from __future__ import division
import threading
import time
from src.robot import Robot
from src.drawing import Map, Canvas
import brickpi


#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

starting_y = int(input("Enter rough starting y coordinate:"))

Robot = Robot(interface,
              pid_config_file="speed_config.json",
              mcl=False,
              threading=False,
              x=0,
              y=starting_y,
              theta=0
              )

Robot.start_obstacle_detection(interval=0.05)

Robot.challenge()
