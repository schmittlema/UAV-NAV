import tensorflow as tf

sess = tf.Session()

a = 0
a_t = tf.Variable(1)
init = tf.global_variables_initializer()
tf.summary.scalar('alpha',a_t)
merge = tf.summary.merge_all()
writer = tf.summary.FileWriter("/root/UAV-NAV/log/logfile-test")

sess.run(init)
for i in range(0,10):
    a+=1
    sess.run(tf.assign(a_t,a))
    summary = sess.run(merge)
    writer.add_summary(summary,i)

