#!/usr/bin/env python
import time
import random
import math
import numpy as np



def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)) # in cm
def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)) # in cm
def calcW():
    return random.random()
def calcTheta():
    return random.randint(0,360)

class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size    # in cm
        self.canvas_size = 768         # in pixels
        self.margin      = 0.05*map_size
        self.scale       = self.canvas_size/(map_size+2*self.margin)

    def drawLine(self,line):
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print "drawLine:" + str((x1,y1,x2,y2))

    def drawCross(self,point, size_x = 2, size_y = 2, offset_y=0):
        x1_a = self.__screenX(point[0]-size_x)
        y1 = self.__screenY(point[1]+offset_y)
        x1_b = self.__screenX(point[0]+size_x)
        print "drawLine:" + str((x1_a,y1,x1_b,y1))
        x2 = self.__screenX(point[0])
        y2_a = self.__screenY(point[1]-size_y+offset_y)
        y2_b = self.__screenY(point[1]+size_y+offset_y)
        print "drawLine:" + str((x2,y2_a,x2,y2_b))

    def drawParticles(self,data,offset_y=0):
        display = [(self.__screenX(d[0][0])+d[1],self.__screenY(d[0][1]+offset_y)+d[1]) for d in data]
        print "drawParticles:" + str(display)
    def __screenX(self,x):
        return (x + self.margin)*self.scale
    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

class Map:
    def __init__(self, canvas):
        self.walls = [];
        self.canvas = canvas
    def add_wall(self,wall):
        self.walls.append(wall)
    def clear(self):
        self.walls = []
    def draw(self):
        for wall in self.walls:
            self.canvas.drawLine(wall)
