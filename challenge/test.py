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
              threading=True
              )
