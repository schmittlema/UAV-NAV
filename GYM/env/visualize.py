import pygame
from pygame.locals import *
import sys
import ast

pygame.init()
size = [1300,1000]
screen = pygame.display.set_mode(size)
s = pygame.Surface(size, pygame.SRCALPHA)
color = (0,0,0) 
white = [255,255,255]
zoom = 100
screen.fill(white)

log = open("log_sample.txt","r")
array = log.readline()
array = ast.literal_eval(array)
log.close()

def origin(x):
    return [size[0]/2 + x[0],size[1]-300 -x[1]]

def process(array):
    events = []
    last_index = -1
    for item in array:
        if last_index == item[0]:
            events[last_index][0].append(item)
        else:
            last_index = item[0]
            events.append([[]])
    return events

runs = process(array)[0]
while(True):
    for event in pygame.event.get():
        s.fill((255,255,255))
        for run in runs:
            position = [int(zoom*run[2][0]),int(zoom*run[2][1])]
            print position
            pygame.draw.circle(screen,color,origin(position),3)
        pygame.display.update()
        if event.type == QUIT:
             pygame.quit()
             sys.exit()
