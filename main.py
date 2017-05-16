import gym
from gym import wrappers
import GYM
import threading
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(threshold='nan')

env = gym.make('GazeboQuadEnv-v0')
env = wrappers.Monitor(env,'/tmp/test',force=True)

def main():
    global env
    print "main started"
    env.env.wait_until_start()
    print "starting back forth"
    
    for i in range(0,3):
        act = 1
        print "forth"
        observation,reward,done,info = env.step(act)

        act = 2
        print"back"
        observation,reward,done,info = env.step(act)
        print "DONE: " + str(done)

    observation,reward,done,info = env.step(0)
    plt.imshow(observation,cmap="gray")
    plt.show()

t = threading.Thread(target=main,args =())
t.daemon = True
t.start()
counter = 15
while counter != 0:
   counter = counter -1
   print counter
   time.sleep(1)
env.reset()
env.env.start()

