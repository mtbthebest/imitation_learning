# coding=utf-8

import os
import numpy as np
import tensorflow as tf
from tf_utils import Model, FullyConnected, Input, Conv2d, DeConv2d



# # inputs = tf.placeholder(tf.float32, shape = [None, 7, 7, 3])
# # conv1 = Conv2d(inputs = inputs, kernel_size = 3, filters = 32, stride = 2, name = 'conv1').out
# # conv1_pool = max_pool_with_argmax(conv1, 2, name = 'pool1')
#
#
# # deconv1 = DeConv2d(conv1, kernel_size = 3,filters = 3, stride = 2, name = 'deconv1').out
#
# # loss = tf.nn.softmax_cross_entropy_with_logits(labels = inputs,logits = deconv1)
# #
# # optimizer = tf.train.AdamOptimizer(0.001).minimize(loss)
#
#
# # def pool(k = 0, C=4):
# #     for j in range(2):
# #         for i in range(2):
# #             t = np.max(a[:, j * stride:(j + 1) * stride, i * stride:(i + 1) * stride, k])
# #             arg = np.argmax(a[:, j * stride:(j + 1) * stride, i * stride:(i + 1) * stride, k])
# #             print arg
# #             if arg%2 == 1:
# #                 m = 2*i + 1
# #                 n = 2*j + (arg-1)/2
# #             else:
# #                 m = 2*i + 0
# #                 n = 2*j + arg/2
# #             indices = (n * M + m) * C
# #             print(m,n,indices)s
# img_placeholder = tf.placeholder(tf.float32,shape=[None, 4,4,3])
#
# pool = tf.nn.max_pool_with_argmax(img_placeholder,ksize=(1,2,2,1),strides=(1,2,2,1), padding="SAME")
#
#
#
# def unpool(inputs, indices, stride=2):
#     with tf.variable_scope("deconv"):
#         # batch_size = tf.shape(inputs, out_type=tf.int64)[0]
#         # pool_shape = [2,2,3]
#         # output_shape = tf.reduce_prod(tf.shape(inputs, out_type=tf.int64)) * tf.square(
#         #     tf.constant(stride, dtype=tf.int64))
#         # pool_ = tf.reshape(inputs, [-1])
#         #
#         # batch_range = tf.range(tf.cast(batch_size, tf.int64))
#         # pool_size = tf.reduce_prod(tf.shape(inputs, out_type=tf.int64)[1:])
#         #
#         # ind_batch_dup = tf.map_fn(
#         #     lambda x: tf.cast(x, tf.int64) * pool_size * tf.constant([1],
#         #                                                              shape=[pool_shape[0], pool_shape[1],
#         #                                                                     pool_shape[2]], dtype=tf.int64),
#         #     batch_range)
#         #
#         # indices = ind_batch_dup + indices
#         # indices = tf.reshape(indices, [-1, ])
#         # print output_shape
#         #
#         # ret = tf.scatter_nd(indices=indices, updates=pool_, shape=tf.constant([0], shape=24,dtype=tf.int64))
#         # # ret = tf.reshape(ret, shape=[-1, pool_shape[0] * stride, pool_shape[1] * stride])
#         #
#         # return pool_
#
#         flat_input_size = tf.size(inputs)
#         batch_range = tf.reshape(tf.range(tf.cast(tf.shape(inputs)[0], dtype=indices.dtype), dtype=indices.dtype),
#                                  shape=[-1, 1, 1, 1])
#
#         b = tf.ones_like(indices) * batch_range
#         b = tf.reshape(b, [flat_input_size, 1])
#         indices_ = tf.reshape(tf.convert_to_tensor(indices), [flat_input_size, 1])
#         indices_ = tf.concat([b, indices_], 1)
#
#         x_ = tf.reshape(tf.convert_to_tensor(inputs), [flat_input_size])
#         # ret = tf.scatter_nd(indices_, x_, shape=[96 ])
#         # print ret
#         # ret = tf.reshape(ret, [-1,4, 4, 3])
#         return indices_
#
#
#
#
# if __name__ == '__main__':
#     img = np.load('/home/mtb/img.npy')
#     graph = tf.get_default_graph()
#     config = tf.ConfigProto()
#     config.gpu_options.allow_growth = True
#
#     print img
#     with tf.Session(graph=graph, config=config) as sess:
#         channel1 = img[:,:,:,0]
#         print channel1
#         a = sess.run(pool, {img_placeholder : img})
#         print(a[0])
#         # print img
#         # print(a[0])
#         # print(a[1])
#         # c= img.reshape([-1])
#         # print c.shape
#         # res = unpool(a[0],a[1])
#         # print a[0]
#         # print a[0].shape
#         # b = sess.run(res)
#         # np.save("indices",b)
#






#---------------------Test for batch normLzation ----------------------
#
#
# x = tf.placeholder(tf.float32, [None,255 ,255,3])
#
# batch_norm = tf.layers.batch_normalization(x)
#
# # mean = tf.reduce_mean(x)
# # variance = tf.
# mean, variance = tf.nn.moments(x,[0,1,2,3])
#
#
# X_= (x - mean)/ tf.sqrt(variance + 1e-9)
#
#
# if __name__ == '__main__':
#     in_ =  np.random.normal(loc= 7.0, scale=2.0, size=[3,255,255,3])
#     # print in_
#     with tf.Session() as sess:
#         tf.global_variables_initializer().run()
#         # m = sess.run(batch_norm,{x: in_})
#         m = sess.run([batch_norm, X_],{x: in_})
#         # print(m[0].shape)
#         n = sess.run([mean,variance], {x: in_})
#         print n
        # print(m[1])
        # m2 = np.mean(in_)
        # v2 = np.var(in_)
        # print (m2)
        # print (v2)


# t = Input('float32', [None,50])
# variables = np.random.normal(size = [10, 50])
model = Model()
input = model.add(Input('float32', (256,256,3)))
cnn1 = model.add(Conv2d(kernel_size=(3,3),filters=32, strides=(2,2)))
cnn1 = model.add(DeConv2d(kernel_size=(3,3),filters=3, strides=(2,2), name='out1'))
# fc1 = model.add(FullyConnected(neurons=50))
# fc2 = model.add(FullyConnected(neurons=100))
# fc3= model.add(FullyConnected(neurons=10))
# fc3= model.add(FullyConnected(neurons=10))
# model.get_variables()
model.get_tensors(name='out1')
print tf.trainable_variables()
# fc2 = model.add(FullyConnected(input=fc1, neurons=50, activation='relu'), reuse=True)

#
#
# a = FullyConnected(t, 50).out
# print a
