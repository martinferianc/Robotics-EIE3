from __future__ import division
import threading
import time
from src.robot import Robot
from src.drawing import Map, Canvas
import brickpi


#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

Canvas = Canvas()
Map = Map(Canvas)

Robot = Robot(interface,
              pid_config_file="speed_config.json",
              mcl=False,
              threading=False,
              x=0,
              y=0,
              theta=0,
              mode="line",
              canvas = Canvas,
              planning = True
              )

Robot.start_obstacle_detection(interval=2)

Robot.start_challenge(interval = 1)
