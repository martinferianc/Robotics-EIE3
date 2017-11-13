from __future__ import division
import threading
import time
from src.robot import Robot
from src.drawing import Map, Canvas
import brickpi


#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

PARTICLES = None

MAP = [[0,0,"O"],
       [0,168,"A"],
       [84,168,"B"],
       [84,126,"C"],
       [84,210,"D"],
       [168,210,"E"],
       [168,84,"F"],
       [210,84,"G"],
       [210,0,"H"]]

POINTS = [(84,30),
          (180,30),
          (180,53),
          (138,54),
          (138,168),
          (114,168),
          (114,84),
          (84,84),
          (84,30),
          (0,0)]

Robot = Robot(interface,
              pid_config_file="carpet_config.json",
              Map=MAP,
              mcl=True,
              x=86,
              y=30,
              mode="continuous",
              theta=0,
              threading=True
              )

Canvas = Canvas()
Map = Map(Canvas)
Map.add_wall((0,0,0,168))        # a
Map.add_wall((0,168,84,168))     # b
Map.add_wall((84,126,84,210))    # c
Map.add_wall((84,210,168,210))   # d
Map.add_wall((168,210,168,84))   # e
Map.add_wall((168,84,210,84))    # f
Map.add_wall((210,84,210,0))     # g
Map.add_wall((210,0,0,0))        # h
Map.draw()


for i in range(1,len(POINTS)):
    if (POINTS[i-1][0] == POINTS[i][0]):
        diff_y = POINTS[i-1][1] - POINTS[i][1]
        y = POINTS[i-1][1]
        x = POINTS[i][0]
        for runs in range(math.ceil(diff_y % 20)):
            if diff_y-y>20:
                y+= 20
            else:
                y+=diff_y-y
            Robot.navigate_to_waypoint(x,y)
            PARTICLES = Robot.get_state()
            Canvas.drawParticles(PARTICLES)
    if (POINTS[i-1][1] == POINTS[i][1]):
        diff_x = POINTS[i-1][0] - POINTS[i][0]
        y = POINTS[i][1]
        x = POINTS[i-1][0]
        for runs in range(math.ceil(diff_x % 20)):
            if diff_x-x>20:
                x+= 20
            else:
                x+=diff_x-x
            Robot.navigate_to_waypoint(x,y)
            PARTICLES = Robot.get_state()
            Canvas.drawParticles(PARTICLES)

Robot.stop_threading()
interface.terminate()
