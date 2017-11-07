#A vizualizer for sampling.py's data on where the trajectory will be give altitude control
#Matt Schmittle
import pygame
from pygame.locals import *
import sys
import ast
import copy as cp
import numpy as np

pygame.init()
size = [1500,800]
screen = pygame.display.set_mode(size)
color = (0,0,0) 
white = [255,255,255]
zoom = 20.0
screen.fill(white)

log = open("log_sample.txt","r")
array = log.readline()
array = ast.literal_eval(array)
log.close()

o_x = 30
o_y = 30
#vel_rows = [-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9,10]
vel_rows = [-5,-4,-3,-2,-1,0,1,2,3,4,5]
accel_rows = [-5,-4,-3,-2,-1,0,1,2,3,4,5]
accel_range = len(accel_rows) 
vel_range = len(vel_rows) 
cmds = [-5,-2.5,0,2.5,5]

def origin(x):
    return [o_x + x[0]+30,o_y-x[1]+30]

def process(array):
    events = []
    last_index = -1
    for item in array:
        if last_index == item[0]:
            events[last_index].append(item)
        else:
            last_index = item[0]
            events.append([])

    base = [None]*accel_range
    for i in range(accel_range):
        base[i]=[None]*vel_range

    lookup_table = {'0':cp.deepcopy(base),'1':cp.deepcopy(base),'2':cp.deepcopy(base),'3':cp.deepcopy(base),'4':cp.deepcopy(base)}
    section = lookup_table['1']
    for event in events:
        section = lookup_table[str(event[0][1])]
        vel = int(round(event[0][3]))
        accel = int(round(event[0][4]))
        if abs(accel)>max(accel_rows):
            if accel < 0:
                accel = -1* max(accel_rows)
            else:
                accel = max(accel_rows)
        if abs(vel)>max(vel_rows):
            if vel<0:
                vel = -1 * max(vel_rows)
            else:
                vel = max(vel_rows)
        vpos = vel_rows.index(vel)
        apos = accel_rows.index(accel)
        if section[apos][vpos] == None:
            section[apos][vpos] = []

        section[apos][vpos].append(event)
        lookup_table[str(event[0][1])] = section

    return lookup_table

runs = process(array)
print runs.keys()
index = 0
while(True):
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_0:
                index = 0 
            if event.key == pygame.K_1:
                index = 1 
            if event.key == pygame.K_2:
                index = 2 
            if event.key == pygame.K_3:
                index = 3 
            if event.key == pygame.K_4:
                index = 4 
            screen.fill(white)
            myfont = pygame.font.SysFont('Times New Roman', 30)
            textsurface = myfont.render('Command: '+str(cmds[index])+"m/s", False, (0, 0, 0))
            screen.blit(textsurface,(size[0]/2,30))
            myfont = pygame.font.SysFont('Times New Roman', 10)
            run = runs[str(index)]
            for x in range(accel_range):
                for y in range(vel_range):
                    o_x = x*(size[0]/accel_range)
                    o_y = y*(size[1]/vel_range)
                    points = run[x][y]
                    if points != None:
                        textsurface = myfont.render('A: '+str(accel_rows[x]) +" " + "V: " + str(vel_rows[y]), False, (0, 0, 0))
                        screen.blit(textsurface,(o_x,o_y+25))
                        for point in points:
                            for subpoint in point:
                                position = [int(zoom*subpoint[2][0]),int(zoom*subpoint[2][1])]
                                pygame.draw.circle(screen,color,origin(position),1)
                                pygame.display.update()
        if event.type == QUIT:
             pygame.quit()
             sys.exit()
