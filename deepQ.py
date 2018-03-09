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
buff_size = 40000
batch_size = 8 #How many experiences to use for each training step.
trace_length = 8
update_freq = 4 #How often to perform a training step.
y = .99 #Discount factor on the target Q-values
startE = 1 #Starting chance of random action
endE = 0.1 #Final chance of random action
anneling_steps = 40000 #How many steps of training to reduce startE to endE.
num_episodes = 3000 #How many episodes of game environment to train network with.
load_model = False #Whether to load a saved model.
path = "/root/log-obst/logfile-rnn-color-large-image" #The path to save our model to.
tau = 0.001 #Rate to update target network toward primary network
learningrate = 0.001
steps_till_training = 10000 #Steps network takes before training so it has a batch to sample from
#accuracy = 0.3
step_length = 0.1
h_size = 7744 #1024 #units from output of rnn to fully connected layer
image_size = 30000
#--------------------------------------------------------------------------

class Qnetwork():
    def __init__(self,rnn_cell,myScope):
        #2500 mono
        self.data = tf.placeholder(shape=[None,image_size],dtype=tf.float32)
        #1 channel mono
        self.input =  tf.reshape(self.data,shape=[-1,100,100,3]) 
        with tf.name_scope(myScope +"_conv1"):
                self.conv1 = slim.conv2d(inputs=self.input,num_outputs=32,kernel_size=[8,8],stride=[4,4],padding='VALID', biases_initializer=None)

	with tf.name_scope(myScope +"_conv2"):
                self.conv2 = slim.conv2d(inputs=self.conv1,num_outputs=64,kernel_size=[4,4],stride=[2,2],padding='VALID', biases_initializer=None)

        self.trainLength = tf.placeholder(dtype=tf.int32)
        #We take the output from the final convolutional layer and send it to a recurrent layer.
        #The input must be reshaped into [batch x trace x units] for rnn processing, 
        #and then returned to [batch x units] when sent through the upper levles.
        self.batch_size = tf.placeholder(dtype=tf.int32,shape=[])
        self.convFlat = tf.reshape(slim.flatten(self.conv2),[self.batch_size,self.trainLength,h_size])
        self.state_in = rnn_cell.zero_state(self.batch_size, tf.float32)
        self.rnn,self.rnn_state = tf.nn.dynamic_rnn(\
                inputs=self.convFlat,cell=rnn_cell,dtype=tf.float32,initial_state=self.state_in,scope=myScope+'_rnn')
        self.rnn = tf.reshape(self.rnn,shape=[-1,h_size])

        with tf.name_scope(myScope+"_ff3"):
                self.feed_forward1 = slim.fully_connected(self.rnn,h_size)

        with tf.name_scope(myScope + "_ff4"):
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
        if len(experience) > trace_length:
            self.buffer.append(experience)
            if len(self.buffer) + 1 >= self.buffer_size:
                self.buffer[0:(1+len(self.buffer))-self.buffer_size] = []

    def sample(self,batch_size,trace_length):
        sampled_episodes = random.sample(self.buffer,batch_size)
        sampledTraces = []
        for episode in sampled_episodes:
            point = np.random.randint(0,len(episode)+1-trace_length)
            sampledTraces.append(episode[point:point+trace_length])
        sampledTraces = np.array(sampledTraces)
        return np.reshape(sampledTraces,[batch_size*trace_length,5])


def processState(states):
    #2500 mono
    return np.reshape(states,[image_size])

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
        cell = tf.contrib.rnn.BasicLSTMCell(num_units=h_size,state_is_tuple=True)
        cellT = tf.contrib.rnn.BasicLSTMCell(num_units=h_size,state_is_tuple=True)
	mainQN = Qnetwork(cell,'main')
	targetQN = Qnetwork(cellT,'target')

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
        Total_steps = tf.Variable(0)
	rAll_t = tf.Variable(0.0)
	j_t = tf.Variable(0.0)
	d_t = tf.Variable(0.0)
	successes = tf.Variable(0)
	collisions = tf.Variable(0)
	auto_steps = tf.Variable(0.0)
	network_steps = tf.Variable(0.0)

	tf.summary.scalar('Reward',rAll_t)
	tf.summary.scalar('Total_steps',Total_steps)
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
	        episodeBuffer = [] 
		#Reset environment and get first new observation
                s = env.reset()
                s = processState(s)
                d = False
		rAll = 0
                state = (np.zeros([1,h_size]),np.zeros([1,h_size])) #Reset the recurrent layer's hidden state
		j=0
                #The Q-Network
                last_request = rospy.Time.now() 
                while not d: #If the agent takes longer than 200 moves to reach either of the blocks, end the trial.
                    if rospy.Time.now() - last_request > rospy.Duration.from_sec(step_length):
                        env.env.next_move = False
                        j+=1
                        #Choose an action by greedily (with e chance of random action) from the Q-network
                        if np.random.rand(1) < e or total_steps < steps_till_training:
                            state1 = sess.run(mainQN.rnn_state,feed_dict={mainQN.data:[s/255.0],mainQN.trainLength:1,mainQN.state_in:state,mainQN.batch_size:1})
                            a = np.random.randint(0,n_classes)
                        else:
                            a, state1 = sess.run([mainQN.predict,mainQN.rnn_state],feed_dict={mainQN.data:[s/255.0],mainQN.trainLength:1,mainQN.state_in:state,mainQN.batch_size:1})
                            a = a[0]
                        s1,r,d,info = env.step(a)
                        s1 = processState(s1)
                        total_steps += 1
                        episodeBuffer.append(np.reshape(np.array([s,a,r,s1,d]),[1,5])) #Save the experience to our episode buffer.

                        if e > endE and total_steps > steps_till_training:
                            e -= stepDrop
                            
                        if total_steps % (update_freq) == 0 and total_steps > steps_till_training:
                            updateTarget(targetOps,sess) #Set the target network to be equal to the primary network.
                            state_train = (np.zeros([batch_size,h_size]),np.zeros([batch_size,h_size])) 
                            trainBatch = myBuffer.sample(batch_size,trace_length) #Get a random batch of experiences.
                            #Below we perform the Double-DQN update to the target Q-values
                            Q1 = sess.run(mainQN.predict,feed_dict={mainQN.data:np.vstack(trainBatch[:,3]/255.0),mainQN.trainLength:trace_length,mainQN.state_in:state_train,mainQN.batch_size:batch_size})

                            Q2 = sess.run(targetQN.Qout,feed_dict={targetQN.data:np.vstack(trainBatch[:,3]/255.0),targetQN.trainLength:trace_length,targetQN.state_in:state_train,targetQN.batch_size:batch_size})
                            end_multiplier = -(trainBatch[:,4] - 1)
                            doubleQ = Q2[range(batch_size*trace_length),Q1]
                            targetQ = trainBatch[:,2] + (y*doubleQ * end_multiplier)
                            #Update the network with our target values.
                            sess.run(mainQN.updateModel,feed_dict={mainQN.data:np.vstack(trainBatch[:,0]/255.0),mainQN.targetQ:targetQ,\
                            mainQN.actions:trainBatch[:,1],mainQN.trainLength:trace_length,\
                            mainQN.state_in:state_train,mainQN.batch_size:batch_size})
                                
                            print "TRAINED!"
                            #env.close()
                            #env.env.close()
                            #writer.close()

                        rAll+=r
                        s = s1
                        state = state1
                        last_request = rospy.Time.now() 
                        
                bufferArray = np.array(episodeBuffer)
                episodeBuffer = list(zip(bufferArray))
                myBuffer.add(episodeBuffer)
                jList.append(j)
                rList.append(rAll)

                #Periodically save the model. 
		sess.run([tf.assign(rAll_t,rAll),tf.assign(j_t,j),tf.assign(successes,env.env.successes),tf.assign(collisions,env.env.collisions),tf.assign(auto_steps,env.env.auto_steps/j),tf.assign(network_steps,env.env.network_steps/j),tf.assign(d_t,env.env.episode_distance),tf.assign(Total_steps,total_steps)])
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
