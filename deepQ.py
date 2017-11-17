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
path = "/root/log-obst/logfile-exp-1" #The path to save our model to.
tau = 0.001 #Rate to update target network toward primary network
learningrate = 0.001
steps_till_training = 10000 #Steps network takes before training so it has a batch to sample from
accuracy = 0.3
step_length = 1
#--------------------------------------------------------------------------

class Qnetwork():
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

class experience_buffer():
    def __init__(self, buffer_size = buff_size):
        self.buffer = []
        self.buffer_size = buffer_size
    
    def add(self,experience):
        if len(self.buffer) + len(experience) >= self.buffer_size:
            self.buffer[0:(len(experience)+len(self.buffer))-self.buffer_size] = []
        self.buffer.extend(experience)

    def sample(self,size):
        #print np.array(random.sample(self.buffer,size)).shape
        return np.reshape(np.array(random.sample(self.buffer,size)),[size,5])

def processState(states):
    return np.reshape(states,[2500])


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
	collisions = tf.Variable(0)
	auto_steps = tf.Variable(0.0)
	network_steps = tf.Variable(0.0)

	tf.summary.scalar('Reward',rAll_t)
	tf.summary.scalar('Episode_length',j_t)
	tf.summary.scalar('Number_of_successes_total',successes)
	tf.summary.scalar('Number_of_collisions',collisions)

	tf.summary.scalar('Percentage_of_autopilot_steps',auto_steps)
	tf.summary.scalar('Percentage_of_network_steps',network_steps)

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
                s = processState(s)
                d = False
		rAll = 0
		j=0
                #The Q-Network
                last_request = rospy.Time.now() 
                while not d: #If the agent takes longer than 200 moves to reach either of the blocks, end the trial.
                    if rospy.Time.now() - last_request > rospy.Duration.from_sec(step_length):
                        env.env.next_move = False
                        j+=1
                        #Choose an action by greedily (with e chance of random action) from the Q-network
                        if np.random.rand(1) < e or total_steps < steps_till_training:
                            a = np.random.randint(0,n_classes)
                        else:
                            a = sess.run(mainQN.predict,feed_dict={mainQN.data:[s]})[0]
                        s1,r,d,info = env.step(a)
                        s1 = processState(s1)
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
                            print "TRAINED!"
                        rAll+=r
                        s = s1
                        last_request = rospy.Time.now() 
                        
                myBuffer.add(episodeBuffer.buffer)
                jList.append(j)
                rList.append(rAll)

                #Periodically save the model. 
		sess.run([tf.assign(rAll_t,rAll),tf.assign(j_t,j),tf.assign(successes,env.env.successes),tf.assign(collisions,env.env.collisions),tf.assign(auto_steps,env.env.auto_steps/j),tf.assign(network_steps,env.env.network_steps/j)])
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
