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
num_episodes = 10 #How many episodes of game environment to test network with.
path = "/home/ubuntu/loging/log-supervised/logfile-redo-safe3" #The path to save our model to.
tau = 0.001 #Rate to update target network toward primary network
step_length = 0.1
learning_rate = 0.001
augment = False 
dropout_uncertainty = False
#--------------------------------------------------------------------------
class network():
    def __init__(self):
	    # declare the training data placeholders
	    self.drop = tf.placeholder(dtype=tf.bool)
            self.xavier = tf.contrib.layers.xavier_initializer(uniform=True,seed=None,dtype=tf.float32)
	    self.x = tf.placeholder(shape=[None,30000],dtype=tf.float32)
	    self.y = tf.placeholder(shape=[None,5],dtype=tf.float32)
            self.input = tf.reshape(self.x,shape=[-1,100,100,3])
            # Convolutional Layer #1. Outputs [-1,100,100,32]
            self.conv1 = tf.layers.conv2d(
                  inputs=self.input,
                  filters=32,
                  kernel_size=[5, 5],
                  padding="same",
                  activation=tf.nn.relu)

            self.conv1_size = tf.shape(self.conv1)
            # Pooling Layer #1.Outputs [-1,50,50,32]
            self.pool1 = tf.layers.max_pooling2d(inputs=self.conv1, pool_size=[2, 2], strides=2)

            self.pool1_size = tf.shape(self.pool1)

            # Convolutional Layer #2 [-1,50,50,64]
            self.conv2 = tf.layers.conv2d(
                  inputs=self.pool1,
                  filters=64,
                  kernel_size=[5, 5],
                  padding="same",
                  activation=tf.nn.relu)

            self.conv2_size = tf.shape(self.conv2)

            # Pooling Layer #2 [-1,25,25,64]
            self.pool2 = tf.layers.max_pooling2d(inputs=self.conv2, pool_size=[2, 2], strides=2)

            self.pool2_size = tf.shape(self.pool2)

            # Dense Layer
            self.pool2_flat = tf.reshape(self.pool2, [-1, 25 * 25 * 64])
            self.dense = tf.layers.dense(inputs=self.pool2_flat, units=1024, activation=tf.nn.relu)
            self.dense2 = tf.layers.dense(inputs=self.dense, units=512, activation=tf.nn.relu)
            self.dropout = tf.layers.dropout(inputs=self.dense2, rate=0.2, training=self.drop)

            # Logits Layer
            self.logits = tf.layers.dense(inputs=self.dropout, units=5)

	    self.predict = tf.nn.softmax(self.logits)

	    #loss
	    #with tf.name_scope("loss"):
	    #	self.loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=self.logits, labels=self.y) )

	    #with tf.name_scope("train"):
		#self.trainer = tf.train.AdamOptimizer(learning_rate=learning_rate)
		#self.updateModel = self.trainer.minimize(self.loss)
	    
	    #with tf.name_scope("testing"):
        	#correct = tf.equal(tf.argmax(self.predict, 1), tf.argmax(self.y, 1))
        	#self.accuracy = tf.reduce_mean(tf.cast(correct, 'float'))



def processState(states):
    return np.reshape(states,[30000])

def main():
	tf.reset_default_graph()
	mainQN = network()

	saver = tf.train.Saver()

	#create lists to contain total rewards and steps per episode
	jList = []
	total_steps = 0
        augtf = tf.Variable(0.0)
	j_t = tf.Variable(0.0)
	d_t = tf.Variable(0.0)
	successes = tf.Variable(0)
	collisions = tf.Variable(0)
	auto_steps = tf.Variable(0.0)
	interventions = tf.Variable(0.0)

	tf.summary.scalar('Episode_length',j_t)
	tf.summary.scalar('Episode_distance',d_t)
	tf.summary.scalar('Number_of_successes_total',successes)
	tf.summary.scalar('Number_of_collisions',collisions)
	tf.summary.scalar('Number_of_Augments_Per_Episode',augtf)

	tf.summary.scalar('Steps',auto_steps)
	tf.summary.scalar('Number of Interventions',interventions)

	#Make a path for our model to be saved in.
	if not os.path.exists(path):
		os.makedirs(path)

	with tf.Session() as sess:
            init = tf.global_variables_initializer()
	    sess.run(init)
	    merged_summary = tf.summary.merge_all()
            saver = tf.train.import_meta_graph('/home/ubuntu/log-supervised/log-1/model-original/model-final.cptk.meta')
            saver.restore(sess,"/home/ubuntu/log-supervised/log-1/model-original/model-final.cptk")
            print "Modelled Restored"
	    writer = tf.summary.FileWriter(path)

	    #writer.add_graph(sess.graph)

            env.env.wait_until_start()
            print "Starting Testing..."
            for i in range(num_episodes):
                print "EPISODE:",i
	        #Reset environment and get first new observation
                aug = 0
		j=0
                s = env.reset()
                s = processState(s)
                d = False
                last_request = rospy.Time.now() 
                while not d: #If the agent takes longer than 200 moves to reach either of the blocks, end the trial.
                    if rospy.Time.now() - last_request > rospy.Duration.from_sec(step_length):
                        env.env.next_move = False
                        j+=1
                        #Choose an action by greedily (with e chance of random action) from the Q-network
                        a = sess.run(tf.argmax(mainQN.predict,1),feed_dict={mainQN.drop:False,mainQN.x:[s]})[0]
                        #For Debugging
                        #decision = sess.run(mainQN.predict,feed_dict={mainQN.drop:False,mainQN.x:[s]})[0]
                        #print decision
                        
                        #DAgger
                        if augment:
                            if dropout_uncertainty:
                                drop1 = np.array(sess.run(mainQN.predict,feed_dict={mainQN.drop:True,mainQN.x:[s]})[0])
                                drop2 = np.array(sess.run(mainQN.predict,feed_dict={mainQN.drop:True,mainQN.x:[s]})[0])
                                drop3 = np.array(sess.run(mainQN.predict,feed_dict={mainQN.drop:True,mainQN.x:[s]})[0])
                                drop4 = np.array(sess.run(mainQN.predict,feed_dict={mainQN.drop:True,mainQN.x:[s]})[0])
                                drop5 = np.array(sess.run(mainQN.predict,feed_dict={mainQN.drop:True,mainQN.x:[s]})[0])
                                aug += env.env.augment_dropout(drop1,drop2,drop3,drop4,drop5)
                            else:
                                raw_v = np.array(sess.run(mainQN.logits,feed_dict={mainQN.drop:False,mainQN.x:[s]})[0])
                                aug += env.env.augment(raw_v)

                        s1,r,d,info = env.step(a)
                        s1 = processState(s1)
                        total_steps += 1
                        last_request = rospy.Time.now() 
                        s = s1
			

                print "AUGMENTS:",aug
                #Periodically save the model. 
		sess.run([tf.assign(j_t,j),tf.assign(successes,env.env.successes),tf.assign(collisions,env.env.collisions),tf.assign(auto_steps,env.env.auto_steps/j),tf.assign(interventions,env.env.num_interventions),tf.assign(augtf,aug),tf.assign(d_t,env.env.episode_distance)])
                print "updated stats"
                aug = 0
                env.env.safe_speed = 0
                env.env.safe_speed_count = 0
                env.env.auto_steps = 0
                env.env.network_steps = 0
		summury = sess.run(merged_summary)
		writer.add_summary(summury,i)
		writer.flush()

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
