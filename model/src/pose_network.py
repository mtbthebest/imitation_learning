# coding=utf-8
"""Pose network"""

import os
import tensorflow as tf
import numpy as np
from utils import pathname, mkdir, Img
from utils import get_folders
from bullet import Interface, Point, Pose, Euler
from tf_utils import model
import scipy.misc
import time

DATASET = '/data/barry/IMITATION/dataset/THROW/TJR'
EVALUATION_PATH = '/data/barry/IMITATION/dataset/THROW/DEMO'
RES_PATH = '/data/barry/IMITATION/dataset/THROW/POSENET_AUTO_ENCODER_FOR_IRLGAN'
DATASET_NUM = 1000

WIDTH = 256
HEIGHT = 256
CHANNELS = 3
BATCH_SIZE = 32
EPISODES = 100000
STEPS = 1472

MODELS_DIR = '/home/barry/hdd/IMITATION/models/visuomotor'

# Testing parameters
ROBOT_URDF_PATH = '/data/barry/IMITATION/sim_ws/pybullet_ws/models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'
END_EFFECTOR_LINK = 'jaco_eef_link'
TABLE = '/data/barry/IMITATION/sim_ws/pybullet_ws/models/cube_table/cube_table.urdf'
CUBE_URDF = '/data/barry/IMITATION/sim_ws/pybullet_ws/models/cube/red_cube.urdf'
BASKET_URDF = '/data/barry/IMITATION/sim_ws/pybullet_ws/models/basket/basket.urdf'

ACTIVATIONS = {'relu': tf.nn.relu,
               'leaky_relu': tf.nn.leaky_relu,
               'softmax': tf.nn.softmax,
               'tanh': tf.nn.tanh,
               'sigmoid': tf.nn.sigmoid,
               'linear': None
               }


def Conv2d(inputs, kernel, filters, stride, padding='SAME', activation='relu', name="cnn"):
    with tf.variable_scope(name):
        out = tf.layers.conv2d(inputs=inputs, filters=filters, kernel_size=(kernel, kernel),
                               strides=(stride, stride), activation=ACTIVATIONS[activation], padding=padding,
                               name=name)
        # out = tf.cond(batch_norm, lambda: tf.layers.batch_normalization(out, training=phase_training,name=name), lambda: out)

        # if activation is not None:
        #     out = ACTIVATIONS[activation](out)

    return out


def DeConv2d(inputs, kernel, filters, stride, padding='SAME', activation='relu', name="cnn"):
    with tf.variable_scope(name):
        out = tf.layers.conv2d_transpose(inputs=inputs, filters=filters, kernel_size=(kernel, kernel),
                                         strides=(stride, stride), activation=ACTIVATIONS[activation],
                                         padding=padding,
                                         name=name)
        # out = tf.cond(batch_norm, lambda: tf.layers.batch_normalization(out, training=phase_training, name=name),
        #               lambda: out)
        #
        # if activation is not None:
        #     out = ACTIVATIONS[activation](out)
    return out


def max_pool(inputs, pool_size=2, stride=2, name="max_pool"):
    pool, indices = tf.nn.max_pool_with_argmax(input=inputs, ksize=(1, pool_size, pool_size, 1),
                                               strides=(1, stride, stride, 1), padding='SAME', name=name)

    return pool, indices


def unpool(inputs, indices, stride=2, name="unpool"):
    input_shape = inputs.get_shape().as_list()
    strides = [1, stride, stride, 1]
    output_shape = (input_shape[0],
                    input_shape[1] * strides[1],
                    input_shape[2] * strides[2],
                    input_shape[3])

    flat_output_shape = [output_shape[0], np.prod(output_shape[1:])]
    flat_input_size = tf.size(inputs)
    batch_range = tf.reshape(tf.range(tf.cast(tf.shape(inputs)[0], dtype=indices.dtype), dtype=indices.dtype),
                             shape=[tf.shape(inputs)[0], 1, 1, 1])
    b = tf.ones_like(indices) * batch_range

    b = tf.reshape(b, [flat_input_size, 1])
    indices_ = tf.reshape(indices, [flat_input_size, 1])
    indices_ = tf.concat([b, indices_], 1)

    inputs_ = tf.reshape(inputs, [flat_input_size])
    ret = tf.scatter_nd(indices_, inputs_, shape=[tf.cast(tf.shape(inputs)[0], tf.int64), flat_output_shape[1]])
    ret = tf.reshape(ret, [tf.shape(inputs)[0], output_shape[1], output_shape[2], output_shape[3]])

    return ret


def Network(inputs, scope_name):
    conv1 = Conv2d(inputs=inputs, filters=32, kernel=3, stride=1, activation='leaky_relu',
                   name=scope_name + '/conv1')

    conv2 = Conv2d(inputs=conv1, filters=64, kernel=3, stride=2, activation='leaky_relu',
                   name=scope_name + '/conv2')

    conv3 = Conv2d(inputs=conv2, filters=64, kernel=3, stride=1, activation='leaky_relu',
                   name=scope_name + '/conv3')

    conv4 = Conv2d(inputs=conv3, filters=96, kernel=3, stride=2, activation='leaky_relu',
                   name=scope_name + '/conv4')

    conv5 = Conv2d(inputs=conv4, filters=128, kernel=3, stride=1, activation='leaky_relu',
                   name=scope_name + '/conv5')

    conv6 = Conv2d(inputs=conv5, filters=128, kernel=3, stride=2, activation='leaky_relu',
                   name=scope_name + '/conv6')
    conv7 = Conv2d(inputs=conv6, filters=256, kernel=3, stride=2, activation='leaky_relu',
                   name=scope_name + '/conv7')

    conv8 = Conv2d(inputs=conv7, filters=8, kernel=3, stride=2, activation='leaky_relu',
                   name=scope_name + '/conv8')

    deconv1 = DeConv2d(inputs=conv8, filters=256, kernel=3, stride=2, activation='leaky_relu',
                       name=scope_name + '/deconv1')
    deconv2 = DeConv2d(inputs=deconv1, filters=128, kernel=3, stride=2, activation='leaky_relu',
                       name=scope_name + '/deconv2')
    deconv3 = DeConv2d(inputs=deconv2, filters=128, kernel=3, stride=2, activation='leaky_relu',
                       name=scope_name + '/deconv3')
    deconv4 = DeConv2d(inputs=deconv3, filters=96, kernel=3, stride=1, activation='leaky_relu',
                       name=scope_name + '/deconv4')
    deconv5 = DeConv2d(inputs=deconv4, filters=64, kernel=3, stride=2, activation='leaky_relu',
                       name=scope_name + '/deconv5')
    deconv6 = DeConv2d(inputs=deconv5, filters=64, kernel=3, stride=1, activation='leaky_relu',
                       name=scope_name + '/deconv6')
    deconv7 = DeConv2d(inputs=deconv6, filters=32, kernel=3, stride=2, activation='leaky_relu',
                       name=scope_name + '/deconv7')
    deconv8 = DeConv2d(inputs=deconv7, filters=3, kernel=3, stride=1, activation='sigmoid',
                       name=scope_name + '/deconv8')

    return conv8, deconv8


class AutoEncoder:
    def __init__(self, sess, width=256, height=256, channels=3,
                 learning_rate=0.001, model_name=None, graph=None):
        self.sess = sess
        self.width = width
        self.height = height
        self.channels = channels
        self.learning_rate = learning_rate
        self.img_features, self.latent_vector, self.img_logits = self.create_network('autoencoder')

        # # self.loss = tf.reduce_mean(
        # #     tf.nn.sigmoid_cross_entropy_with_logits(labels=self.img_labels, logits = self.img_logits))
        #
        self.loss = tf.reduce_mean(
            tf.square(self.img_logits - self.img_features))

        self.optimizer = tf.train.AdamOptimizer(self.learning_rate).minimize(self.loss)

        self.model = model(sess, MODELS_DIR, model_name=model_name)
        self.summary = self.model.summary(graph=graph, **dict(loss=self.loss))
        self.eval_indice = len(get_folders(pathname(RES_PATH, **dict(flag=1))))

    def create_network(self, scope_name):
        with tf.variable_scope(scope_name):
            I = tf.placeholder(dtype=tf.float32, shape=(None, self.width, self.height, self.channels))
            # B_N= tf.placeholder(tf.bool)
            # P_T = tf.placeholder(tf.bool)
            Z, O = Network(I, 'encoder')
        # return I,B_N,P_T, Z, O
        return I, Z, O

    def evaluate(self, img_features):
        eval_path = mkdir(RES_PATH, 'EVAL_%d' % (self.eval_indice + 1))
        img_path = mkdir(eval_path, 'IMG')
        IMG = Img()

        eval_img = self.sess.run(self.img_logits, {
            self.img_features: img_features
        })
        i=0
        for rgb in eval_img:
            rgb *= 255.0
            rgb = rgb.astype(np.uint8)

            IMG.img_from_array(rgb, pathname(img_path, 'img_%d.png' % i))
            i+=1

        self.eval_indice += 1

    def train(self, img):
        return self.sess.run([self.optimizer, self.loss, self.summary.run()], {
            self.img_features: img
        })

    def save_model(self):
        self.model.save()


def load_data(serialized_features):
    parsed_features = tf.parse_single_example(serialized_features, {
        "prev_img": tf.FixedLenFeature((), tf.string),
        "step": tf.FixedLenFeature((), tf.string)}
                                              )
    parsed_features['prev_img'] = tf.decode_raw(parsed_features['prev_img'], tf.uint8)
    parsed_features['prev_img'] = tf.reshape(parsed_features['prev_img'], [WIDTH, HEIGHT, CHANNELS])
    parsed_features['prev_img'] = tf.cast(parsed_features['prev_img'], tf.float32)
    parsed_features['prev_img'] /= 255.0

    return parsed_features


def data_func(shape):
    filenames = tf.placeholder(tf.string, shape=shape)
    dataset = tf.data.TFRecordDataset(filenames)
    dataset = dataset.map(load_data)
    dataset = dataset.repeat()
    dataset = dataset.batch(BATCH_SIZE)
    iterator = dataset.make_initializable_iterator()
    next_features = iterator.get_next()

    return filenames, iterator, next_features


if __name__ == '__main__':
    graph = tf.get_default_graph()
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    with tf.Session(graph=graph, config=config) as sess:
        filenames, iterator, next_features = data_func(shape=(DATASET_NUM,))
        sess.run(iterator.initializer, {filenames: [pathname(DATASET, 'demo_%d.tfrecord' % i)
                                                    for i in range(DATASET_NUM)]})

        filenames_eval, iterator_eval, next_features_eval = data_func(shape=(100,))
        sess.run(iterator_eval.initializer, {filenames_eval: [pathname(DATASET, 'demo_%d.tfrecord' % i)
                                                    for i in range(DATASET_NUM, DATASET_NUM+100)]})

        autoencoder = AutoEncoder(sess, WIDTH, HEIGHT, CHANNELS,
                                  model_name='posenet_autoencoder_for_irlgan',
                                  graph=graph
                                  )
        if not autoencoder.model.restore():
            sess.run(tf.global_variables_initializer())

        for i in range(2500):
            features = sess.run(next_features)

            _, loss, summary = autoencoder.train(features['prev_img'])
            autoencoder.summary.add(summary)
            print(i, loss)

            if i % STEPS == 0:
                autoencoder.save_model()

            if i % 500 == 0:
                features = sess.run(next_features_eval)

                autoencoder.evaluate(features['prev_img'])
