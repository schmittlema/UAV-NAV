import gym
import GYM
import pymp
import time

env = gym.make('GazeboQuadEnv-v0')

def main():
    print "main started"
    #env.get_data()
    #counter = 30
    #env.takeoff()
    #while counter !=0:
    #    print counter
    #    counter = counter-1
    #    time.sleep(1)
    #env.observe()
    #env._reset()

with pymp.Parallel(2) as p:
    if p.thread_num == 0:
	env.start()
    else:
        main()

