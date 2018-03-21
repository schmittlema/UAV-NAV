from __future__ import division

import gym
import numpy as np
import random
import tensorflow as tf
import tensorflow.contrib.slim as slim
import matplotlib.pyplot as plt
import scipy.misc
import os
from gym import wrappers
import GYM
import threading
import time
import cv2
import rospy
np.set_printoptions(threshold='nan')

#PARAMETERS
#-------------------------------------------------------------------------

#I mean actions
n_classes = 5
buff_size = 1000000
batch_size = 32 #How many experiences to use for each training step.
update_freq = 4 #How often to perform a training step.
y = .99 #Discount factor on the target Q-values
startE = 1 #Starting chance of random action
endE = 0.1 #Final chance of random action
anneling_steps = 40000 #How many steps of training to reduce startE to endE.
num_episodes = 720 #How many episodes of game environment to train network with.
load_model = False #Whether to load a saved model.
path = "/root/log-obst/logfile-auto-world" #The path to save our model to.
tau = 0.001 #Rate to update target network toward primary network
learningrate = 0.001
steps_till_training = 10000 #Steps network takes before training so it has a batch to sample from
accuracy = 0.3
step_length = 0.1
#--------------------------------------------------------------------------

class network():
    def __init__(self):
        self.data = tf.placeholder(shape=[None,2500],dtype=tf.float32)
        self.input =  tf.reshape(self.data,shape=[-1,50,50,1]) 
        with tf.name_scope("layer1"):
                self.conv1 = slim.conv2d(inputs=self.input,num_outputs=32,kernel_size=[8,8],stride=[4,4],padding='VALID', biases_initializer=None)

	with tf.name_scope("layer2"):
                self.conv2 = slim.conv2d(inputs=self.conv1,num_outputs=64,kernel_size=[4,4],stride=[2,2],padding='VALID', biases_initializer=None)

        #with tf.name_scope("layer3"):
        #        self.conv3 = slim.conv2d(inputs=self.conv2,num_outputs=64,kernel_size=[4,4],stride=[2,2],padding='VALID', biases_initializer=None)

        #with tf.name_scope("layer4"):
        #        self.conv4 = slim.conv2d(inputs=self.conv3,num_outputs=n_classes,kernel_size=[2,2],stride=[1,1],padding='VALID', biases_initializer=None)
        with tf.name_scope("layer3"):
                self.feed_forward1 = slim.fully_connected(tf.reshape(self.conv2,[-1,1024]),1024)

        with tf.name_scope("layer4"):
                self.Qout = slim.fully_connected(self.feed_forward1,n_classes)
                
		
        self.predict = tf.argmax(self.Qout,1)
        
        #Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
        self.targetQ = tf.placeholder(shape=[None],dtype=tf.float32)
        self.actions = tf.placeholder(shape=[None],dtype=tf.int32)
        self.actions_onehot = tf.one_hot(self.actions,n_classes,dtype=tf.float32)
        
        self.Q = tf.reduce_sum(tf.multiply(self.Qout, self.actions_onehot), axis=1)
        
	with tf.name_scope("loss"):
		self.td_error = tf.square(self.targetQ - self.Q)
		self.loss = tf.reduce_mean(self.td_error)

	with tf.name_scope("train"):
		self.trainer = tf.train.AdamOptimizer(learning_rate=learningrate)
		self.updateModel = self.trainer.minimize(self.loss)


def processState(states):
    return np.reshape(states,[2500])

def main():
	tf.reset_default_graph()
	mainQN = Qnetwork()

	saver = tf.train.Saver()

	#Set the rate of random action decrease. 
	e = startE
	stepDrop = (startE - endE)/anneling_steps

	#create lists to contain total rewards and steps per episode
	jList = []
	rList = []
	total_steps = 0
	rAll_t = tf.Variable(0.0)
	j_t = tf.Variable(0.0)
	d_t = tf.Variable(0.0)
	successes = tf.Variable(0)
	collisions = tf.Variable(0)
	auto_steps = tf.Variable(0.0)
	network_steps = tf.Variable(0.0)

	tf.summary.scalar('Reward',rAll_t)
	tf.summary.scalar('Episode_length',j_t)
	tf.summary.scalar('Episode_distance',d_t)
	tf.summary.scalar('Number_of_successes_total',successes)
	tf.summary.scalar('Number_of_collisions',collisions)

	tf.summary.scalar('Percentage_of_autopilot_steps',auto_steps)
	tf.summary.scalar('Percentage_of_network_steps',network_steps)

	#Make a path for our model to be saved in.
	if not os.path.exists(path):
		os.makedirs(path)

	with tf.Session() as sess:
            init = tf.global_variables_initializer()
	    sess.run(init)
	    merged_summary = tf.summary.merge_all()
	    writer = tf.summary.FileWriter(path)

	    writer.add_graph(sess.graph)

            env.env.wait_until_start()
            for i in range(num_episodes):
		#Reset environment and get first new observation
		rAll = 0
		j=0
		_ = sess.run(mainQN.updateModel,feed_dict={mainQN.data:input_data,Y: input_y})
			
                #Periodically save the model. 
		sess.run([tf.assign(rAll_t,rAll),tf.assign(j_t,j),tf.assign(successes,env.env.successes),tf.assign(collisions,env.env.collisions),tf.assign(auto_steps,env.env.auto_steps/j),tf.assign(network_steps,env.env.network_steps/j),tf.assign(d_t,env.env.episode_distance)])
                env.env.auto_steps = 0
                env.env.network_steps = 0
		summury = sess.run(merged_summary)
		writer.add_summary(summury,i)
		writer.flush()

                if i % 1000 == 0:
                    saver.save(sess,path+'/model-'+str(i)+'.cptk')
                    print("Saved Model")
                if len(rList) % 10 == 0:
                    print(total_steps,np.mean(rList[-10:]), e)
	    saver.save(sess,path+'/model-final'+str(i)+'.cptk')
	    env.close()
	    env.env.close()
	    writer.close()
            print "DONE!"

if __name__ == "__main__":
    env = gym.make('GazeboQuadEnv-v0')
    env = wrappers.Monitor(env,path,force=True)
    t = threading.Thread(target=main,args =())
    t.daemon = True
    t.start()

    env.reset()
env.env.start()
