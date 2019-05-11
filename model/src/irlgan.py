# coding=utf-8
"""Pose network"""

import os
import tensorflow as tf
import numpy as np
from utils import pathname, mkdir, Img, get_folders
from tensorflow.contrib.layers import variance_scaling_initializer
from bullet import Interface, Point, Pose, Euler
from tf_utils import model, FullyConnected, LSTM
import scipy.misc
import time

DATASET = '/data/barry/IMITATION/dataset/THROW/TJR'
EVALUATION_PATH = '/data/barry/IMITATION/dataset/THROW/DEMO'
RES_PATH = '/data/barry/IMITATION/dataset/THROW/IRLGAN'
# DATASET_NUM = 1000
DATASET_NUM = 400

WIDTH = 256
HEIGHT = 256
CHANNELS = 3
BATCH_SIZE = 64
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
               'softmax': tf.nn.softmax,
               'tanh': tf.nn.tanh,
               'sigmoid': tf.nn.sigmoid,
               'leaky_relu': tf.nn.leaky_relu,
               'linear': None
               }

names = ['conv%dblock'%i for i in range(1,6)]
filters =[64,128,256,512,512]
blocks_num=[2,2,3,3,3]

filters_deconv =[3,64,128,256,512]

Inputs = []
Pool_indices =[]


def Conv2d(inputs, kernel, filters, stride, padding = 'SAME', activation = 'relu', name = "cnn"):
    with tf.variable_scope(name):
        out = tf.layers.conv2d(inputs = inputs, filters = filters, kernel_size = (kernel, kernel),
                               strides = (stride, stride), activation = ACTIVATIONS[activation], padding = padding,
                               name = name)

    return out


def DeConv2d(inputs, kernel, filters, stride, padding = 'SAME', activation = 'relu', name = "cnn"):
    with tf.variable_scope(name):
        out = tf.layers.conv2d_transpose(inputs = inputs, filters = filters, kernel_size = (kernel, kernel),
                                         strides = (stride, stride), activation = ACTIVATIONS[activation],
                                         padding = padding,
                                         name = name)
    return out


def max_pool(inputs, pool_size = 2, stride = 2, name = "max_pool"):
    pool, indices = tf.nn.max_pool_with_argmax(input = inputs, ksize = (1, pool_size, pool_size, 1),
                                               strides = (1, stride, stride, 1), padding = 'SAME', name = name)

    return pool, indices


def unpool(inputs, indices, stride = 2, name = "unpool"):
    input_shape = inputs.get_shape().as_list()
    strides = [1, stride, stride, 1]
    output_shape = (input_shape[0],
                    input_shape[1] * strides[1],
                    input_shape[2] * strides[2],
                    input_shape[3])

    flat_output_shape = [output_shape[0], np.prod(output_shape[1:])]
    flat_input_size = tf.size(inputs)
    batch_range = tf.reshape(tf.range(tf.cast(tf.shape(inputs)[0], dtype = indices.dtype), dtype = indices.dtype),
                             shape = [tf.shape(inputs)[0], 1, 1, 1])
    b = tf.ones_like(indices) * batch_range

    b = tf.reshape(b, [flat_input_size, 1])
    indices_ = tf.reshape(indices, [flat_input_size, 1])
    indices_ = tf.concat([b, indices_], 1)

    inputs_ = tf.reshape(inputs, [flat_input_size])
    ret = tf.scatter_nd(indices_, inputs_, shape = [tf.cast(tf.shape(inputs)[0], tf.int64), flat_output_shape[1]])
    ret = tf.reshape(ret, [tf.shape(inputs)[0], output_shape[1], output_shape[2], output_shape[3]])

    return ret


def Conv2DBlock(inputs, filter, block_num, name, flag = 'conv2D'):
    for i in range(block_num):
        if flag == 'conv2D':
            out = Conv2d(inputs, kernel = 3, filters = filter, stride = 1, name = name + '-' + str(i))
        elif flag == 'deconv2D':
            out = DeConv2d(inputs, kernel = 3, filters = filter, stride = 1, name = name + '-' + str(i))
        else:
            raise ValueError('Define the correct flag')
        inputs = out
        Inputs.append(out)


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

        # self.model = model(sess, MODELS_DIR, model_name=model_name)
        # self.summary = self.model.summary(graph=graph, **dict(loss=self.loss))
        # self.eval_indice = len(get_folders(pathname(RES_PATH, **dict(flag=1))))

    def create_network(self, scope_name):

        with tf.variable_scope(scope_name):
            input = tf.placeholder(dtype=tf.float32, shape=(None, self.width, self.height, self.channels))
            Inputs.append(input)

            for (block_name,filter, block_num) in zip(names, filters, blocks_num):
                Conv2DBlock(Inputs[-1], filter, block_num, block_name)
                pool, pool_indices = max_pool(Inputs[-1])


                Inputs.append(pool)
                Pool_indices.append(pool_indices)



            fc6 = Conv2d(Inputs[-1], kernel=7, filters=512, stride=1, padding='VALID', name='fc6')
            Inputs.append(fc6)
            # fc7 = Conv2d(Inputs[-1], kernel=1, filters=512,stride=1, padding='VALID',name='fc7')
            # Inputs.append(fc7)

            deconv = DeConv2d(Inputs[-1],7,filters=512,stride=1, padding='VALID', name='deconv_fc6')
            Inputs.append(deconv)
            upsampling = unpool(Inputs[-1], Pool_indices[-1])
            Inputs.append(upsampling)
            Pool_indices.pop()

            Conv2DBlock(Inputs[-1], 512, 3, 'de' + 'conv5', flag='deconv2D')

            upsampling = unpool(Inputs[-1], Pool_indices[-1])
            Inputs.append(upsampling)
            Pool_indices.pop()
            Conv2DBlock(Inputs[-1], 512, 2, 'de' + 'conv4', flag='deconv2D')

            Conv2DBlock(Inputs[-1], 256, 1, 'de' + 'conv4-1', flag='deconv2D')

            upsampling = unpool(Inputs[-1], Pool_indices[-1])
            Inputs.append(upsampling)
            Pool_indices.pop()
            Conv2DBlock(Inputs[-1], 256, 2, 'de' + 'conv3', flag='deconv2D')
            Conv2DBlock(Inputs[-1], 128, 1, 'de' + 'conv3-1', flag='deconv2D')

            upsampling = unpool(Inputs[-1], Pool_indices[-1])
            Inputs.append(upsampling)
            Pool_indices.pop()
            Conv2DBlock(Inputs[-1], 128, 1, 'de' + 'conv2', flag='deconv2D')
            Conv2DBlock(Inputs[-1], 64, 1, 'de' + 'conv2-1', flag='deconv2D')

            upsampling = unpool(Inputs[-1], Pool_indices[-1])
            Inputs.append(upsampling)
            Pool_indices.pop()
            Conv2DBlock(Inputs[-1], 64, 1, 'de' + 'conv1', flag='deconv2D')

            out = DeConv2d(Inputs[-1], 3, filters=3, stride=1, padding='SAME', name='recons_img', activation='sigmoid')
        return Inputs[0], fc6, out

    def get_latent_vector(self, img):
        vec= self.sess.run(self.latent_vector, {
            self.img_features:  img
        })

        return vec

    def restore(self, variables):
        trainable_variables = tf.trainable_variables()

        for vars in variables:

            indice = trainable_variables.index(list(filter(lambda v: vars in v.name,trainable_variables))[0])
            op = tf.assign(trainable_variables[indice],variables[vars])

            sess.run(op)


def logsigmoid(a):
    r"""Computes softplus: `log(exp(a) + 1)`"""
    return -tf.nn.softplus(-a)

def logit_bernoulli_entropy(logits):
    ent = (1. - tf.nn.sigmoid(logits)) * logits - logsigmoid(logits)
    return ent


class GAIL:
    def __init__(self, sess, width = 256, height = 256, channels = 3,
                 learning_rate = 0.0001, z_shape = 8, j_shape = 9, model_name = None, graph = None, coeff=0.002):
        self.sess = sess
        self.width = width
        self.height = height
        self.channels = channels
        self.learning_rate = learning_rate
        self.z_shape = z_shape
        self.j_shape = j_shape

        self.z_o, self.z_g, self.z_t = self.gen_network('generator')
        self.p_t = tf.placeholder(tf.float32, shape = (None, self.j_shape))

        self.std = tf.exp(self.logstds)

        self.mse_loss = tf.reduce_mean(tf.square(self.p_t - self.action_mean))
        lh_loss = 0.5 * tf.reduce_sum(tf.square((self.p_t - self.action_mean) / self.std), axis = -1) \
                  + 0.5 * np.log(2.0 * np.pi) * tf.to_float(tf.shape(self.p_t)[-1]) + tf.reduce_sum(self.logstds,
                                                                                                    axis = -1)
        self.lh_loss = tf.reduce_mean(lh_loss)

        self.optimizer = tf.train.AdamOptimizer(learning_rate).minimize(self.lh_loss)

        self.model = model(sess, MODELS_DIR, model_name = model_name)
        self.summary = self.model.summary(graph = graph, **dict(mse_loss = self.mse_loss,
                                                                lh_loss = self.lh_loss))
        self.eval_indice = len(get_folders(pathname(RES_PATH, **dict(flag = 1))))


    def gen_network(self, scope_name):
        with tf.variable_scope(scope_name) as scope:
            z_o = tf.placeholder(tf.float32, shape = [None, 2,2,512])
            z_g = tf.placeholder(tf.float32, shape = [None, 2,2,512])
            z_t = tf.placeholder(tf.float32, shape = [None, 2,2,512])

            z_o_reshape = tf.reshape(z_o, shape=[-1, 2 * 2 * 512])
            z_g_reshape = tf.reshape(z_g, shape=[-1, 2 * 2 * 512])
            z_t_reshape = tf.reshape(z_t, shape=[-1, 2 * 2 * 512])

            hidden_size = [2048,1096,512,256]

            def build_net(input, name):
                with tf.variable_scope(name):
                    fc = input
                    for i in range(len(hidden_size)):
                        fc = FullyConnected(fc, hidden_size[i], name = name + '_' + str(i)).out
                return fc

            fc_o = build_net(z_o_reshape, 'zo')
            fc_g = build_net(z_g_reshape, 'zg')
            fc_t = build_net(z_t_reshape, 'zt')

            fc_concat = tf.concat([fc_o, fc_g, fc_t], axis=1)

            fc1 = FullyConnected(fc_concat, 512, name = 'fc1').out
            fc2 = FullyConnected(fc1, 512, name = 'fc2').out
            fc3 = FullyConnected(fc2, 256, name = 'fc3').out

            fc6 = tf.expand_dims(fc3, axis = 2)
            LSTM_NN = LSTM(fc6, neurons = 256).out
            self.action_mean = FullyConnected(LSTM_NN[:, -1], neurons = self.j_shape, name = 'fc7',
                                              activation = 'tanh').out

            logstds = tf.get_variable('logstds', shape = [1, 9], initializer = tf.constant_initializer(0.0),
                                      trainable = True)
            self.logstds = tf.tile(logstds, tf.stack((tf.shape(self.action_mean)[0], 1)))

        return z_o, z_g, z_t

    def train(self, z_o, z_g, z_t, p_t):
        return self.sess.run([self.optimizer, self.mse_loss, self.lh_loss, self.summary.run()], {
            self.z_o: z_o,
            self.z_g: z_g,
            self.z_t: z_t,
            self.p_t: p_t
        })

    def save_model(self):
        self.model.save()


class GAIL2:
    def __init__(self, sess, width = 256, height = 256, channels = 3,
                 learning_rate = 0.0001, z_shape = 8, j_shape = 9, model_name = None, graph = None, coeff = 0.002):
        self.sess = sess
        self.width = width
        self.height = height
        self.channels = channels
        self.learning_rate = learning_rate
        self.z_shape = z_shape
        self.j_shape = j_shape

        self.img, self.p_t, self.p_t_1 = self.gen_network('generator')
        # self.p_t = tf.placeholder(tf.float32, shape = (None, self.j_shape))
        # self.std = tf.exp(self.logstds)
        # self.var = self.std ** 2

        # self.loss = 0.5 * tf.reduce_sum(tf.square((self.p_t - self.action_mean) / self.var)) \
        #             + 0.5 * np.log(2.0 * np.pi) * tf.to_float(tf.shape(self.p_t)[0]) \
        #             + tf.reduce_sum(self.logstds)

        self.std = tf.exp(self.logstds)
        # self.var = tf.square(self.std)
        self.p_t_1_l = tf.placeholder(tf.float32, (None, 9))


        self.mse_loss = tf.reduce_mean(tf.square(self.p_t_1 - self.p_t_1_l))
        lh_loss = 0.5 * tf.reduce_sum(tf.square((self.p_t_1_l - self.p_t_1) / self.std), axis=-1) \
               + 0.5 * np.log(2.0 * np.pi) * tf.to_float(tf.shape(self.p_t_1)[-1]) + tf.reduce_sum(self.logstds, axis=-1)
        self.lh_loss = tf.reduce_mean(lh_loss)

        self.optimizer = tf.train.AdamOptimizer(learning_rate).minimize(self.lh_loss)

        self.model = model(sess, MODELS_DIR, model_name = model_name)
        self.summary = self.model.summary(graph = graph, **dict(mse_loss = self.mse_loss,
                                                                lh_loss = self.lh_loss))
        self.eval_indice = len(get_folders(pathname(RES_PATH, **dict(flag = 1))))

    def gen_network(self, scope_name):
        with tf.variable_scope(scope_name) as scope:
            I = tf.placeholder(dtype = tf.float32, shape = (None, self.width, self.height, self.channels))

            conv1 = Conv2d(I, 5, 64, stride = 4, name = 'conv1').out

            conv2 = Conv2d(conv1, 3, 64, stride = 4, name = 'conv2').out

            conv3 = Conv2d(conv2, 3, 96, stride = 4, name = 'conv3').out

            conv4 = Conv2d(conv3, 3, 128, stride = 2, name = 'conv4').out

            conv5 = Conv2d(conv4, 3, 256, stride = 2, name = 'conv5').out

            conv_flatten = tf.contrib.layers.flatten(conv5)
            J = tf.placeholder(dtype = tf.float32, shape = [None, 9])

            FC1 = FullyConnected(tf.concat([conv_flatten, J], axis = 1), neurons = 256, name = 'FC1_I').out

            FC = tf.expand_dims(FC1, axis = 2)

            LSTM_NN = LSTM(FC, neurons = 256).out

            FC2 = FullyConnected(LSTM_NN[:, -1], neurons = 256, name = 'FC2').out

            J_1 = FullyConnected(FC2, neurons = 9, activation = 'tanh',
                                 name = 'velocities').out
            logstds = tf.get_variable('logstds', shape=[1,9], initializer=tf.constant_initializer(0.0), trainable=True)
            self.logstds = tf.tile(logstds, tf.stack((tf.shape(J_1)[0], 1)))

        return I, J, J_1

    def train(self, img, p_t, p_t_1):
        return self.sess.run([self.optimizer, self.mse_loss, self.lh_loss, self.summary.run(), self.p_t_1], {
            self.img: img,
            self.p_t: p_t,
            self.p_t_1_l: p_t_1,
        })

    def evaluate(self):
        indice = np.random.randint(1000, 1020)
        folder = pathname(EVALUATION_PATH, 'demo_%d' % indice)
        scene_objects = 'scene_objects.npy'
        with Interface(mode = 'direct') as I:
            robot = I.load_model(ROBOT_URDF_PATH)
            table = I.load_model(TABLE, Pose(Point(0.4, 0.0, 0.05), Euler(1.57, 0.0, 0.0)),
                                 fixed_base = True, scaling = 1.0)
            I.set_camera_pose(0.6, 155, -50, [0.2, 0.0, 0.5])
            scene = np.load(os.path.join(folder, scene_objects))[2:, 0]
            block = I.load_model(CUBE_URDF, Pose(Point(*scene[0])), fixed_base = False)
            basket = I.load_model(BASKET_URDF, Pose(Point(*scene[1])), fixed_base = False)

            img = np.concatenate(
                [[np.load(pathname(folder, 'IMG', 'img_%d.npy' % i))[:, :, :3]] for i in range(1, STEPS + 1)])
            joints = I.movable_a_joints + I.movable_g_joints

            eval_path = mkdir(RES_PATH, 'EVAL_%d' % (self.eval_indice + 1))
            img_path = mkdir(eval_path, 'IMG')
            with open(pathname(eval_path, 'eval.txt'), 'w') as f:
                f.write('Evaluation number: %d' % indice)

            for i in range(STEPS):
                joints_configurations = [[state.jointPosition for state in I.get_joint_state(robot, joints)]]
                # rgb = self.sess.run(tf.image.per_image_standardization(img[i]))
                rgb = img[i]
                vel = self.sess.run(self.p_t_1, {
                    self.img: rgb.reshape(-1, WIDTH, HEIGHT, CHANNELS),
                    self.p_t: joints_configurations
                })
                # vel = np.concatenate([vel, [[0.0, 0.0, 0.0]]], axis=1)
                I.step_with_pose(body = robot, joints = joints, poses = vel[0])

                img_array = I.get_camera_image(WIDTH, HEIGHT)[:, :, :3]
                scipy.misc.imsave(pathname(img_path, 'img_%d.png' % i), img_array)
        self.eval_indice += 1

    def save_model(self):
        self.model.save()

def load_data(serialized_features):
    keys = {'prev_img': tf.FixedLenFeature((), tf.string),
            'next_img': tf.FixedLenFeature((), tf.string),
            'prev_jnt_conf': tf.FixedLenFeature((), tf.string),
            'next_jnt_conf': tf.FixedLenFeature((), tf.string),
            'goal_img': tf.FixedLenFeature((), tf.string),
            'initial_img': tf.FixedLenFeature((), tf.string),
            'step': tf.FixedLenFeature((), tf.string)
            }
    parsed_features = tf.parse_single_example(serialized_features, keys)

    for img_key in ['prev_img', 'goal_img', 'initial_img']:
        parsed_features[img_key] = tf.decode_raw(parsed_features[img_key], tf.uint8)
        parsed_features[img_key] = tf.reshape(parsed_features[img_key], [WIDTH, HEIGHT, CHANNELS])
        parsed_features[img_key] = tf.cast(parsed_features[img_key], tf.float32)
        parsed_features[img_key] /= 255.0

    parsed_features['next_jnt_conf'] = tf.decode_raw(parsed_features['next_jnt_conf'], tf.float64)
    parsed_features['next_jnt_conf'] = tf.cast(parsed_features['next_jnt_conf'], tf.float32)
    parsed_features['next_jnt_conf'] /= 3.14

    parsed_features['prev_jnt_conf'] = tf.decode_raw(parsed_features['prev_jnt_conf'], tf.float64)
    parsed_features['prev_jnt_conf'] = tf.cast(parsed_features['prev_jnt_conf'], tf.float32)
    parsed_features['prev_jnt_conf'] /= 3.14

    return parsed_features


def data_func():
    filenames = tf.placeholder(tf.string, shape = [DATASET_NUM])
    dataset = tf.data.TFRecordDataset(filenames)
    dataset = dataset.map(load_data)
    dataset = dataset.repeat()
    dataset = dataset.batch(BATCH_SIZE)
    iterator = dataset.make_initializable_iterator()
    next_features = iterator.get_next()

    return filenames, iterator, next_features


def get_model_variables(model_name, tensors):
    graph = tf.Graph()
    restored_variables = {}
    with tf.Session(graph= graph) as sess:
        saver = tf.train.import_meta_graph(model_name)
        saver.restore(sess,pathname(MODELS_DIR,"posenet_autoencoder_for_irlgan2/model/model.ckpt"))
        for name in tensors:
            t = graph.get_tensor_by_name(name)
            restored_variables[name] =sess.run(t)
    return restored_variables


if __name__ == '__main__':
    VARS = [u'autoencoder/conv1block-0/conv1block-0/kernel:0', u'autoencoder/conv1block-0/conv1block-0/bias:0',
            u'autoencoder/conv1block-1/conv1block-1/kernel:0', u'autoencoder/conv1block-1/conv1block-1/bias:0',
            u'autoencoder/conv2block-0/conv2block-0/kernel:0', u'autoencoder/conv2block-0/conv2block-0/bias:0',
            u'autoencoder/conv2block-1/conv2block-1/kernel:0', u'autoencoder/conv2block-1/conv2block-1/bias:0',
            u'autoencoder/conv3block-0/conv3block-0/kernel:0', u'autoencoder/conv3block-0/conv3block-0/bias:0',
            u'autoencoder/conv3block-1/conv3block-1/kernel:0', u'autoencoder/conv3block-1/conv3block-1/bias:0',
            u'autoencoder/conv3block-2/conv3block-2/kernel:0', u'autoencoder/conv3block-2/conv3block-2/bias:0',
            u'autoencoder/conv4block-0/conv4block-0/kernel:0', u'autoencoder/conv4block-0/conv4block-0/bias:0',
            u'autoencoder/conv4block-1/conv4block-1/kernel:0', u'autoencoder/conv4block-1/conv4block-1/bias:0',
            u'autoencoder/conv4block-2/conv4block-2/kernel:0', u'autoencoder/conv4block-2/conv4block-2/bias:0',
            u'autoencoder/conv5block-0/conv5block-0/kernel:0', u'autoencoder/conv5block-0/conv5block-0/bias:0',
            u'autoencoder/conv5block-1/conv5block-1/kernel:0', u'autoencoder/conv5block-1/conv5block-1/bias:0',
            u'autoencoder/conv5block-2/conv5block-2/kernel:0', u'autoencoder/conv5block-2/conv5block-2/bias:0',
            u'autoencoder/fc6/fc6/kernel:0', u'autoencoder/fc6/fc6/bias:0',
            u'autoencoder/deconv_fc6/deconv_fc6/kernel:0', u'autoencoder/deconv_fc6/deconv_fc6/bias:0',
            u'autoencoder/deconv5-0/deconv5-0/kernel:0', u'autoencoder/deconv5-0/deconv5-0/bias:0',
            u'autoencoder/deconv5-1/deconv5-1/kernel:0', u'autoencoder/deconv5-1/deconv5-1/bias:0',
            u'autoencoder/deconv5-2/deconv5-2/kernel:0', u'autoencoder/deconv5-2/deconv5-2/bias:0',
            u'autoencoder/deconv4-0/deconv4-0/kernel:0', u'autoencoder/deconv4-0/deconv4-0/bias:0',
            u'autoencoder/deconv4-1/deconv4-1/kernel:0', u'autoencoder/deconv4-1/deconv4-1/bias:0',
            u'autoencoder/deconv4-1-0/deconv4-1-0/kernel:0', u'autoencoder/deconv4-1-0/deconv4-1-0/bias:0',
            u'autoencoder/deconv3-0/deconv3-0/kernel:0', u'autoencoder/deconv3-0/deconv3-0/bias:0',
            u'autoencoder/deconv3-1/deconv3-1/kernel:0', u'autoencoder/deconv3-1/deconv3-1/bias:0',
            u'autoencoder/deconv3-1-0/deconv3-1-0/kernel:0', u'autoencoder/deconv3-1-0/deconv3-1-0/bias:0',
            u'autoencoder/deconv2-0/deconv2-0/kernel:0', u'autoencoder/deconv2-0/deconv2-0/bias:0',
            u'autoencoder/deconv2-1-0/deconv2-1-0/kernel:0', u'autoencoder/deconv2-1-0/deconv2-1-0/bias:0',
            u'autoencoder/deconv1-0/deconv1-0/kernel:0', u'autoencoder/deconv1-0/deconv1-0/bias:0',
            u'autoencoder/recons_img/recons_img/kernel:0', u'autoencoder/recons_img/recons_img/bias:0']
    variables = get_model_variables(
        model_name = pathname(MODELS_DIR, "posenet_autoencoder_for_irlgan2/model/model.ckpt.meta"),
        tensors = VARS)
    tf.global_variables_initializer()

    graph = tf.reset_default_graph()
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    with tf.Session(graph = graph, config = config) as sess:
        filenames, iterator, next_features = data_func()
        sess.run(iterator.initializer, {filenames: [pathname(DATASET, 'demo_%d.tfrecord' % i)
                                                    for i in range(DATASET_NUM)]})

        autoencoder = AutoEncoder(sess, WIDTH, HEIGHT, CHANNELS, graph = graph)
        gail = GAIL(sess, WIDTH, HEIGHT, CHANNELS, model_name = 'irlgan', graph = graph)

        sess.run(tf.global_variables_initializer())

        autoencoder.restore(variables)
        #
        # if not gail.model.restore():
        #     sess.run(tf.global_variables_initializer())

        for i in range(1000000):
            features = sess.run(next_features)
            z_o = autoencoder.get_latent_vector(features['initial_img'])
            z_g = autoencoder.get_latent_vector(features['goal_img'])
            z_t = autoencoder.get_latent_vector(features['prev_img'])

            # _, loss, summary, p = gail.train(z_o, z_g, z_t, features['next_jnt_conf'])
            _, mse_loss, lh_loss, summary = gail.train(z_o,
                                                       z_g,
                                                       z_t,
                                                       features['next_jnt_conf'])
            gail.summary.add(summary)
            print(i, mse_loss, lh_loss)

            if i % STEPS == 0:
                gail.save_model()

            # if i % 5000 == 0 and i>0:
            #     gail.evaluate()
