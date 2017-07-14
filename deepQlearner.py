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
np.set_printoptions(threshold='nan')

#PARAMETERS
#-------------------------------------------------------------------------

#Network Structure: feed forward 4x10x10x10x5
#Input = x,y position and velocity
#Output = Q values for forward, back, left, right, hold
n_nodes_h1 = 5
n_nodes_h2 = 5
n_nodes_h3 = 5

#I mean actions
n_classes = 3
input_size = 2

buff_size = 1000000
batch_size = 32 #How many experiences to use for each training step.
update_freq = 4 #How often to perform a training step.
y = .99 #Discount factor on the target Q-values
startE = 1 #Starting chance of random action
endE = 0.1 #Final chance of random action
anneling_steps = 500000 #How many steps of training to reduce startE to endE.
num_episodes = 10000 #How many episodes of game environment to train network with.
max_epLength = 200 #The max allowed length of our episode.
load_model = False #Whether to load a saved model.
path = "../log/logfile-exp-11" #The path to save our model to.
h_size = 512 #The size of the final convolutional layer before splitting it into Advantage and Value streams.
tau = 0.001 #Rate to update target network toward primary network
learningrate = 0.001
steps_till_training = 10000 #Steps network takes before training so it has a batch to sample from
#--------------------------------------------------------------------------

class Qnetwork():
    def __init__(self):
        #The network recieves a frame from the game, flattened into an array.
        #It then resizes it and processes it through four convolutional layers.
	self.data = tf.placeholder('float',[None,input_size],name="input") #input data
	with tf.name_scope("layer1"):
		hidden_1_layer = {'weights':tf.Variable(tf.random_normal([input_size,n_nodes_h1]),name="W"),'biases':tf.Variable(tf.random_normal([n_nodes_h1]),name="B")}
		l1 = tf.add(tf.matmul(self.data,hidden_1_layer['weights']),hidden_1_layer['biases'])
		l1 = tf.nn.relu(l1)

	with tf.name_scope("layer2"):
		hidden_2_layer = {'weights':tf.Variable(tf.random_normal([n_nodes_h1,n_nodes_h2]),name="W"),'biases':tf.Variable(tf.random_normal([n_nodes_h2]),name="B")}
		l2 = tf.add(tf.matmul(l1,hidden_2_layer['weights']),hidden_2_layer['biases'])
		l2 = tf.nn.relu(l2)
    
	with tf.name_scope("layer3"):
		hidden_3_layer = {'weights':tf.Variable(tf.random_normal([n_nodes_h2,n_nodes_h3]),name="W"),'biases':tf.Variable(tf.random_normal([n_nodes_h3]),name="B")}
		l3 = tf.add(tf.matmul(l2,hidden_3_layer['weights']),hidden_3_layer['biases'])
		l3 = tf.nn.relu(l3)

	with tf.name_scope("output_layer"):
		output_layer = {'weights':tf.Variable(tf.random_normal([n_nodes_h3,n_classes]),name="W"),'biases':tf.Variable(tf.random_normal([n_classes]),name="B")}
    
    	self.Qout = tf.add(tf.matmul(l3,output_layer['weights']),output_layer['biases'])
		
		#Max q val (best action)
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



class experience_buffer():
    def __init__(self, buffer_size = buff_size):
        self.buffer = []
        self.buffer_size = buffer_size
    
    def add(self,experience):
        if len(self.buffer) + len(experience) >= self.buffer_size:
            self.buffer[0:(len(experience)+len(self.buffer))-self.buffer_size] = []
        self.buffer.extend(experience)
            
    def sample(self,size):
        return np.reshape(np.array(random.sample(self.buffer,size)),[size,5])


def updateTargetGraph(tfVars,tau):
    total_vars = len(tfVars)
    op_holder = []
    for idx,var in enumerate(tfVars[0:total_vars//2]):
        op_holder.append(tfVars[idx+total_vars//2].assign((var.value()*tau) + ((1-tau)*tfVars[idx+total_vars//2].value())))
    return op_holder

def updateTarget(op_holder,sess):
    for op in op_holder:
        sess.run(op)

def main():
	tf.reset_default_graph()
	mainQN = Qnetwork()
	targetQN = Qnetwork()

	saver = tf.train.Saver()

	trainables = tf.trainable_variables()

	targetOps = updateTargetGraph(trainables,tau)

	myBuffer = experience_buffer()

	#Set the rate of random action decrease. 
	e = startE
	stepDrop = (startE - endE)/anneling_steps

	#create lists to contain total rewards and steps per episode
	jList = []
	rList = []
	total_steps = 0
	rAll_t = tf.Variable(0.0)
	j_t = tf.Variable(0.0)
	successes = tf.Variable(0)

	tf.summary.scalar('reward',rAll_t)
	tf.summary.scalar('episode_length',j_t)
	tf.summary.scalar('number_of_successes_total',successes)

	#Make a path for our model to be saved in.
	if not os.path.exists(path):
		os.makedirs(path)

	with tf.Session() as sess:
	    if load_model == True:
		print('Loading Model...')
		ckpt = tf.train.get_checkpoint_state(path)
		saver.restore(sess,ckpt.model_checkpoint_path)

            init = tf.global_variables_initializer()
	    sess.run(init)
	    merged_summary = tf.summary.merge_all()
	    writer = tf.summary.FileWriter(path)

	    writer.add_graph(sess.graph)

            env.env.wait_until_start()
            for i in range(num_episodes):
	        episodeBuffer = experience_buffer()
		#Reset environment and get first new observation
                s = env.reset()
                d = False
		rAll = 0
		j=0
                #The Q-Network
                while j < max_epLength: #If the agent takes longer than 200 moves to reach either of the blocks, end the trial.
		    j+=1
                    #Choose an action by greedily (with e chance of random action) from the Q-network
                    if np.random.rand(1) < e or total_steps < steps_till_training:
                        a = np.random.randint(0,n_classes)
                    else:
                        a = sess.run(mainQN.predict,feed_dict={mainQN.data:[s]})[0]
                    s1,r,d,info = env.step(a)
                    total_steps += 1
                    episodeBuffer.add(np.reshape(np.array([s,a,r,s1,d]),[1,5])) #Save the experience to our episode buffer.
                    
                    
                    if e > endE and total_steps > steps_till_training:
                        e -= stepDrop
                        
                    if total_steps % (update_freq) == 0 and total_steps > steps_till_training:
                        trainBatch = myBuffer.sample(batch_size) #Get a random batch of experiences.
                        #Below we perform the Double-DQN update to the target Q-values
                        Q1 = sess.run(mainQN.predict,feed_dict={mainQN.data:np.vstack(trainBatch[:,3])})
                        Q2 = sess.run(targetQN.Qout,feed_dict={targetQN.data:np.vstack(trainBatch[:,3])})
                        end_multiplier = -(trainBatch[:,4] - 1)
                        doubleQ = Q2[range(batch_size),Q1]
                        targetQ = trainBatch[:,2] + (y*doubleQ * end_multiplier)
                        #Update the network with our target values.
                        _ = sess.run(mainQN.updateModel,feed_dict={mainQN.data:np.vstack(trainBatch[:,0]),mainQN.targetQ:targetQ, mainQN.actions:trainBatch[:,1]})
                            
                        updateTarget(targetOps,sess) #Set the target network to be equal to the primary network.
                    rAll+=r
                    s = s1
                    
                    if d:
                       break
		    
                myBuffer.add(episodeBuffer.buffer)
                jList.append(j)
                rList.append(rAll)

                #Periodically save the model. 
		sess.run([tf.assign(rAll_t,rAll),tf.assign(j_t,j),tf.assign(successes,env.env.successes)])
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

if __name__ == "__main__":
    env = gym.make('GazeboQuadEnv-v0')
    env = wrappers.Monitor(env,path,force=True)
    t = threading.Thread(target=main,args =())
    t.daemon = True
    t.start()

    env.reset()
    env.env.start()
