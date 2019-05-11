# coding=utf-8
import csv
import pandas as pd
from utils import plot
import tensorflow as tf

x = tf.placeholder(tf.float32, shape = [105])
W = tf.Variable(initial_value = tf.truncated_normal(shape = [105]))
y = tf.multiply(x, W)
y_pred = tf.placeholder(tf.float32, shape = [105])
loss = tf.reduce_mean(tf.square(y_pred - y))
optimizer = tf.train.GradientDescentOptimizer(0.01)
grad_vars = optimizer.compute_gradients(loss, [W])
train_op = optimizer.apply_gradients(grads_and_vars = grad_vars)
# g = tf.gradients(loss, [W])
# optimizer.apply_gradients()


if __name__ == '__main__':
    data = pd.read_csv('/home/mtb/Downloads/sat.csv')
    high_gpa = data.values[:, 0]
    univ_gpa = data.values[:, -1]

    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())
        for i in range(10000):
            # y_val = sess.run([loss, optimizer], {x:high_gpa, y_pred: univ_gpa})
            y_val = sess.run([loss, train_op], {x: high_gpa, y_pred: univ_gpa})
            print y_val[0]

    # plot.plot_curve(high_gpa, univ_gpa)
