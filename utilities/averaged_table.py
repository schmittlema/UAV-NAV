#!/usr/bin/env python
'''
This script creates a simple table from environemnt monitoring 
data obtained from outdir, which has been 
created when calling a gazebo environment monitoring function in a test.

Args: 

  arg1       Creates a table using 'arg1' as average size delimiter.

Examples:

  python display_image.py 20

'''
import os
import gym
from gym import wrappers
import itertools
import sys

def expand(lst, n):
    lst = [[i]*n for i in lst]
    lst = list(itertools.chain.from_iterable(lst))
    return lst

if __name__ == '__main__':

    outdir = '/tmp/test'
    data_key = 'episode_rewards'
    mod1 = int(sys.argv[1])

    results = wrappers.monitoring.load_results(outdir)
    data =  results[data_key]

    avg_data = []

    for i, val in enumerate(data):
        if i%mod1==0:
            if (i+mod1) < len(data):
                avg =  sum(data[i:i+mod1])/mod1
                avg_data.append(avg)

    print ("Printing averaged graph with interval="+str(mod1)+" ...")
    print ("----------------")
    for i, val in enumerate(avg_data):
        print str(i*mod1)+"-"+str(i*mod1+mod1)+" | "+str(val)
    print ("----------------")
