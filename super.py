import tensorflow as tf
import tensorflow.contrib.slim as slim
import numpy as np
import random

# Python optimisation variables
learning_rate = 0.001
epochs = 10
batch_size = 100
classes = 5
path = "/root/log-supervised/log-1"

class network():
    def __init__(self):
	    # declare the training data placeholders
	    self.x = tf.placeholder(shape=[None,30000],dtype=tf.float32)
	    self.y = tf.placeholder(shape=[None,5],dtype=tf.float32)
	    with tf.name_scope("layer1"):
		self.conv1 = slim.conv2d(inputs=self.x,num_outputs=32,kernel_size=[8,8],stride=[4,4],padding='VALID', biases_initializer=None)

	    with tf.name_scope("layer2"):
		self.conv2 = slim.conv2d(inputs=self.conv1,num_outputs=64,kernel_size=[4,4],stride=[2,2],padding='VALID', biases_initializer=None)

		#with tf.name_scope("layer3"):
		#        self.conv3 = slim.conv2d(inputs=self.conv2,num_outputs=64,kernel_size=[4,4],stride=[2,2],padding='VALID', biases_initializer=None)

		#with tf.name_scope("layer4"):
		#        self.conv4 = slim.conv2d(inputs=self.conv3,num_outputs=n_classes,kernel_size=[2,2],stride=[1,1],padding='VALID', biases_initializer=None)
	    with tf.name_scope("layer3"):
		 self.feed_forward1 = slim.fully_connected(tf.reshape(self.conv2,[-1,1024]),1024)

	    with tf.name_scope("layer4"):
		 self.out = slim.fully_connected(self.feed_forward1,classes)
	    
	    self.predict = tf.argmax(self.out,1)

	    #loss
	    with tf.name_scope("loss"):
	    	self.loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=self.predict, labels=self.y) )

	    with tf.name_scope("train"):
		self.trainer = tf.train.AdamOptimizer(learning_rate=learning_rate)
		self.updateModel = self.trainer.minimize(self.loss)
	    
	    with tf.name_scope("testing"):
        	correct = tf.equal(tf.argmax(self.predict, 1), tf.argmax(y, 1))
        	accuracy = tf.reduce_mean(tf.cast(correct, 'float'))

class sub_data():
    def __init__(self):
	self.labels = []
	self.images = []

    def next_batch(self,batch_size):
	#samples without replacement
	sample_x = []
	sample_y = []
	choice = np.arange(0,len(self.labels))
	sample = random.sample(choice,batch_size)
	for i in range(0,len(sample)):
		sample_x.append(self.images[sample[i]])
		sample_y.append(self.labels[sample[i]])
		
	return sample_x,sample_y

class data():
    def __init__(self):
	self.train = sub_data()
	self.test = sub_data()

	filenames = ["train_output.txt","train_input.txt","test_output.txt","test_input.txt"]
	for i in range(0,len(filenames):
	    with open(filenames[i],'r') as infile:
	        for line in infile:
		    if line[0] == "{":
			if i = 0:
			    self.train.labels.append(eval(line))
			if i = 1:
			    self.train.images.append(eval(line))
			if i = 2:
			    self.test.labels.append(eval(line))
			if i = 3:
			    self.test.labels.append(eval(line))

# finally setup the initialisation operator
init_op = tf.global_variables_initializer()

#initialize nn
network = network()

#initialize data
date = data()

# add a summary to store the accuracy
loss = tf.Variable(0.0)
tf.summary.scalar('Loss', accuracy)

merged = tf.summary.merge_all()
writer = tf.summary.FileWriter(path)
writer.add_graph(sess.graph)

# start the session
with tf.Session() as sess:
# initialise the variables
	sess.run(init_op)
	total_batch = int(len(data.train.labels) / batch_size)
	for epoch in range(epochs):
	    avg_loss = 0
	    for i in range(total_batch):
		batch_x, batch_y = data.train.next_batch(batch_size=batch_size)
		_, c = sess.run([network.optimiser, network.loss], feed_dict={x: batch_x, y: batch_y})
		avg_loss += c / total_batch
	    print("Epoch:", (epoch + 1), "loss =", "{:.3f}".format(avg_loss))
	    
	    sess.run([tf.assign(loss,avg_loss)])
	    summary = sess.run(merged)
	    writer.add_summary(summary, epoch)
	    writer.flush()

	print("\nTraining complete!")
	writer.add_graph(sess.graph)
	print(sess.run(network.accuracy, feed_dict={x: data.test.images, y: data.test.labels}))
