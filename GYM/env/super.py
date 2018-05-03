import tensorflow as tf
import tensorflow.contrib.slim as slim
import numpy as np
import random

# Python optimisation variables
learning_rate = 0.001
epochs = 5
batch_size = 100
classes = 5
path = "/home/ubuntu/log-supervised/log-1"
data_path = "/home/ubuntu/Training_data/"
augment_data = False

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
            self.dropout = tf.layers.dropout(
                inputs=self.dense2, rate=0.4, training=self.drop)

            # Logits Layer
            self.logits = tf.layers.dense(inputs=self.dropout, units=5)

	    self.predict = tf.nn.softmax(self.logits)

	    #loss
	    with tf.name_scope("loss"):
	    	self.loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=self.logits, labels=self.y) )

	    with tf.name_scope("train"):
		self.trainer = tf.train.AdamOptimizer(learning_rate=learning_rate)
		self.updateModel = self.trainer.minimize(self.loss)
	    
	    with tf.name_scope("testing"):
        	correct = tf.equal(tf.argmax(self.predict, 1), tf.argmax(self.y, 1))
        	self.accuracy = tf.reduce_mean(tf.cast(correct, 'float'))

class sub_data():
    def __init__(self):
        self.test_data = False
	self.labels = []
	self.images = []

    def next_batch(self,batch_size):
	#samples without replacement
	sample_x = []
	sample_y = []
	choice = np.arange(0,len(self.labels))
	sample = random.sample(choice,batch_size)
        forward_count = 0.0
	for i in range(0,len(sample)):
            if self.labels[sample[i]][2] == 1 and self.test_data:
                forward_count+=1.0
            sample_x.append(self.images[sample[i]])
            sample_y.append(self.labels[sample[i]])
        if self.test_data:
            print "Forward Count:", forward_count, forward_count/float(batch_size)
		
	return sample_x,sample_y

class data():
    def __init__(self):
        print "reading data"
	self.train = sub_data()
	self.test = sub_data()

	filenames = ["train_output.txt","train_input.txt","test_output.txt","test_input.txt"]
	#filenames = ["train_output.txt","train_input.txt"]
        for i in range(0,len(filenames)):
	    with open(data_path+filenames[i],'r') as infile:
	        for line in infile:
                    if i == 0:
                        self.train.labels.append(eval(line))
                        if augment_data:
                            self.train.labels.append(np.flip(eval(line),0))
                    if i == 1:
                        self.train.images.append(eval(line))
                        if augment_data:
                            self.train.images.append(np.flip(eval(line),0))
                    if i == 2:
                        self.test.labels.append(eval(line))
                    if i == 3:
                        self.test.images.append(eval(line))


tf.reset_default_graph()

#initialize nn
network = network()

#initialize data
data = data()

data.test.test_data = True

# add a summary to store the accuracy
loss = tf.Variable(0.0)
tf.summary.scalar('Loss', loss)
#For saving model
saver = tf.train.Saver()

# start the session
print "Number of Training Instances:",len(data.train.labels)
print "Starting training..."
with tf.Session() as sess:
    # initialise the variables
    init_op = tf.global_variables_initializer()
    sess.run(init_op)
    merged = tf.summary.merge_all()
    writer = tf.summary.FileWriter(path)
    writer.add_graph(sess.graph)
    tbatch_x, tbatch_y = data.test.next_batch(batch_size=500)
    print("Test-Set Accuracy:",sess.run(network.accuracy, feed_dict={network.x: tbatch_x, network.y: tbatch_y,network.drop:False}))
    print
    total_batch = int(len(data.train.labels) / batch_size)
    for epoch in range(epochs):
        avg_loss = 0
        for i in range(total_batch):
            batch_x, batch_y = data.train.next_batch(batch_size=batch_size)
            _, c = sess.run([network.updateModel, network.loss], feed_dict={network.x: batch_x, network.y: batch_y,network.drop:True})
            #c1,p1,c2,p2 = sess.run([network.conv1_size,network.pool1_size,network.conv2_size,network.pool2_size],feed_dict={network.x : batch_x, network.y:batch_y})
            #print c1,p1,c2,p2
            avg_loss += c / total_batch
        print("Epoch:", (epoch + 1), "loss =", "{:.3f}".format(avg_loss))

        tbatch_x, tbatch_y = data.test.next_batch(batch_size=500)
        print("Test-Set Accuracy:",sess.run(network.accuracy, feed_dict={network.x: tbatch_x, network.y: tbatch_y,network.drop:False}))
        print
        sess.run([tf.assign(loss,avg_loss)])
        summary = sess.run(merged)
        writer.add_summary(summary, epoch)
        writer.flush()

    print("\nTraining complete!")
    batch_size = 500
    total_batch = int(len(data.test.labels) / batch_size)
    saver.save(sess,path+'/model/model-final.cptk')
    print("Model saved!")
    avg_accuracy = 0
    for i in range(total_batch):
        tbatch_x, tbatch_y = data.test.next_batch(batch_size=batch_size)
        c = sess.run(network.accuracy, feed_dict={network.x: tbatch_x, network.y: tbatch_y,network.drop:False}) 
	avg_accuracy += c / total_batch
    print("Test-Set Accuracy:",avg_accuracy)
