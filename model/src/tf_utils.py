import tensorflow as tf
from tensorflow.contrib.rnn import MultiRNNCell, BasicLSTMCell, LSTMCell
from tensorflow.contrib.layers import variance_scaling_initializer
from utils import mkdir, get_folders, pathname, is_file, folder_exists
import datetime
from tensorflow.python.training import moving_averages
from math import ceil

ACTIVATIONS = {'relu': tf.nn.relu,
               'softmax': tf.nn.softmax,
               'tanh': tf.nn.tanh,
               'sigmoid': tf.nn.sigmoid,
               'linear': None
               }


def model(sess, train_dir, model_name=None, vars = None):
    if model_name is None:
        model_folder = datetime.datetime.now().strftime("%Y_%b_%d_%H_%M")
        model_dirname = mkdir(train_dir, model_folder, 'model')
        log_dirname = mkdir(train_dir, model_folder, 'summary')
    else:
        model_folder = pathname(train_dir, model_name, **dict(flag=1))
        model_dirname = pathname(model_folder, 'model', **dict(flag=1))
        log_dirname = pathname(model_folder, 'summary', **dict(flag=1))

    saver = tf.train.Saver(vars)

    def save(filename='model.ckpt'):
        saver.save(sess, pathname(model_dirname, filename))

    def restore(model_path=None, filename='model.ckpt'):
        if model_path is None:
            model_path = model_dirname
        if is_file(pathname(model_path, 'checkpoint')):
            saver.restore(sess, pathname(model_path, filename))
            print('Layers weights restored')
            return True
        else:
            print('Layers weights not restored')
            return False

    def summary(graph=None, **kwargs):
        for key, values in kwargs.items():
            tf.summary.scalar(key, values)
        writer = tf.summary.FileWriter(log_dirname, graph=graph)
        merged = tf.summary.merge_all()

        def run():
            return merged

        def add(summaries, step=None):
            writer.add_summary(summaries, global_step=step)

        summary.run = run
        summary.add = add

        return summary

    def get_folder_name():
        return pathname(train_dir, model_folder)

    model.save = save
    model.restore = restore
    model.summary = summary
    model.folder = get_folder_name

    return model


class FullyConnected:
    """Fully connexion"""

    def __init__(self, inputs, neurons, initializer=variance_scaling_initializer(), activation='relu',
                 name='fully_connected'):
        self.input = inputs
        with tf.variable_scope(name):
            W = tf.get_variable('weights', shape=[self.input.shape[-1], neurons], initializer=initializer)
            b = tf.get_variable('bias', shape=[neurons], initializer=initializer)
            self.out = ACTIVATIONS[activation](tf.matmul(self.input, W) + b) if ACTIVATIONS[activation] else tf.matmul(
                self.input, W) + b
        self.weights = {'weights': W, 'bias': b}


class Conv2d:
    """Convolutional neural network"""

    def __init__(self, inputs, kernel_size, filters, stride=1,
                 padding='SAME', activation='relu', initializer=variance_scaling_initializer(),
                 name='conv', use_max_pool=False):
        self.input = inputs
        with tf.variable_scope(name):
            W = tf.get_variable('weights', shape=[kernel_size, kernel_size, int(self.input.shape[-1]), filters],
                                initializer=initializer)
            b = tf.get_variable('bias', shape=[filters], initializer=initializer)
            self.out = ACTIVATIONS[activation](
                tf.nn.bias_add(
                    tf.nn.conv2d(self.input, W, strides=(1, stride, stride, 1), padding=padding), b)
            ) if ACTIVATIONS[activation] else tf.nn.bias_add(
                tf.nn.conv2d(self.input, W, strides=(1, stride, stride, 1), padding=padding), b)
            if use_max_pool:
                self.out = max_pool(self.out, stride, name=name)

        self.weights = {'weights': W, 'bias': b}


def max_pool(inputs, stride=1, padding='SAME', name='pool'):
    """Max pooling"""
    with tf.variable_scope(name):
        return tf.nn.max_pool(inputs, [1, stride, stride, 1],
                              strides=[1, stride, stride, 1], padding=padding, name=name)


def batch_normalize(inputs, beta=0.0, gamma=1.0, epsilon=1e-05, decay=0.9, training=True, trainable=True,
                    name='batch_norm'):
    input_shape = inputs.get_shape().as_list()

    with tf.variable_scope(name):
        beta = tf.get_variable('beta', shape=input_shape[-1:], initializer=tf.constant_initializer(beta),
                               trainable=trainable)
        gamma = tf.get_variable('gamma', shape=input_shape[-1:], initializer=tf.constant_initializer(gamma),
                                trainable=trainable)
        axis = list(range(len(input_shape) - 1))
        batch_mean, batch_variance = tf.nn.moments(inputs, axis)

        moving_mean = tf.get_variable('moving_mean', shape=input_shape[-1:], initializer=tf.constant_initializer(0.0),
                                      trainable=False)
        moving_variance = tf.get_variable('moving_variance', shape=input_shape[-1:],
                                          initializer=tf.constant_initializer(1.0), trainable=False)

        def update():
            update_moving_mean = moving_averages.assign_moving_average(moving_mean, batch_mean, decay,
                                                                       zero_debias=False)
            update_moving_variance = moving_averages.assign_moving_average(moving_variance, batch_variance, decay,
                                                                           zero_debias=False)

            with tf.control_dependencies([update_moving_mean, update_moving_variance]):
                return tf.identity(batch_mean), tf.identity(batch_variance)

        mean, variance = tf.cond(training,
                                 update,
                                 lambda: moving_mean, moving_variance)

        output = tf.nn.batch_normalization(inputs, mean, variance, beta, gamma, epsilon, name=name)

    return output


class DeConv2d:
    """DeConvolutional neural network"""

    def __init__(self, inputs, kernel_size, filters, stride=1,
                 padding='SAME', activation='relu', initializer=variance_scaling_initializer(),
                 name='deconv', padding_size=0):
        self.input = inputs
        with tf.variable_scope(name):
            W = tf.get_variable('weights', shape=[kernel_size, kernel_size, filters, int(self.input.shape[-1])],
                                initializer=initializer)
            b = tf.get_variable('bias', shape=[filters], initializer=initializer)

            strides = (1, stride, stride, 1)
            output_shape = int(ceil(int(stride * (int(self.input.shape[-2]) - 1) + kernel_size - 2 * padding_size)))

            self.out = ACTIVATIONS[activation](tf.nn.bias_add(
                tf.nn.conv2d_transpose(self.input, W, output_shape=tf.stack(
                    [tf.shape(self.input)[0], output_shape, output_shape, filters]), strides=strides,
                                       padding=padding), b)) if ACTIVATIONS[activation] else tf.nn.bias_add(
                tf.nn.conv2d_transpose(self.input, W, output_shape=tf.stack(
                    [tf.shape(self.input)[0], output_shape, output_shape, filters]), strides=strides,
                                       padding=padding), b)

        self.weights = {'weights': W, 'bias': b}

#
# class LSTM:
#     """LSTM network"""
#
#     def __init__(self, inputs, neurons, layers=1, dtype=tf.float32, name='lstm'):
#         self.input = inputs
#         with tf.variable_scope(name):
#             cell = BasicLSTMCell(num_units=neurons)
#             if layers > 1:
#                 cell = MultiRNNCell([BasicLSTMCell(num_units=layers) for _ in range(layers)])
#             self.out, self.states = tf.nn.dynamic_rnn(cell=cell, inputs=self.input, dtype=dtype)
#         self.weights = {'weights': cell.variables[0], 'bias': cell.variables[1]}

class LSTM:
    """LSTM network"""

    def __init__(self, inputs, neurons, dtype = tf.float32, name = 'lstm'):
        self.input = inputs
        with tf.variable_scope(name):
            cell = LSTMCell(num_units = neurons)
            self.out, self.states = tf.nn.dynamic_rnn(cell = cell, inputs = self.input, dtype = dtype)


def _int64_feature(value):
    return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))


def _bytes_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))


def normalize_image(img_array):
    return tf.image.per_image_standardization(img_array)
