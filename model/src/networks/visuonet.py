# coding=utf-8
"""Visuomotor network"""

import os
import tensorflow as tf
import numpy as np
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from tf_utils import FullyConnected, Conv2d, model, LSTM
from utils import pathname, mkdir, Img
from utils import Pd, get_files, get_folders
# from bullet import Interface, Point, Pose, Euler
import scipy.misc

DATASET = '/data/barry/IMITATION/DATASET/THROW/VISUOMOTOR'
EVALUATION_PATH = '/data/barry/IMITATION/DATASET/THROW/DEMO'
RES_PATH = '/data/barry/IMITATION/EVALUATION/VISUONET'
DATASET_NUM = 1000

WIDTH = 256
HEIGHT = 256
CHANNELS = 3
BATCH_SIZE = 64
EPISODES = 100000
STEPS = 1472

MODELS_DIR = '/home/barry/hdd/IMITATION/MODELS'

# Testing parameters
ROBOT_URDF_PATH = '/data/barry/IMITATION/sim_ws/pybullet_ws/models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'
END_EFFECTOR_LINK = 'jaco_eef_link'
TABLE = '/data/barry/IMITATION/sim_ws/pybullet_ws/models/cube_table/cube_table.urdf'
CUBE_URDF = '/data/barry/IMITATION/sim_ws/pybullet_ws/models/cube/red_cube.urdf'
BASKET_URDF = '/data/barry/IMITATION/sim_ws/pybullet_ws/models/basket/basket.urdf'


class VisuoNet:
    def __init__(self, sess, width = 256, height = 256, channels = 3,
                 action_dim = 9, learning_rate = 0.0001, model_name = None, graph = None,export = False):
        self.sess = sess
        self.width = width
        self.height = height
        self.channels = channels
        self.action_dim = action_dim
        self.learning_rate = learning_rate
        self.rgb, self.conf, self.vel, = self.create_network('visuonet')

        if not export:
            self.velocities = tf.placeholder(tf.float32, [None, 9])
            self.loss = tf.reduce_mean(tf.square(self.velocities - self.vel)) #+ 0.001 * tf.reduce_mean(tf.square(self.gripper - self.vel[:,6]))
            self.optimizer = tf.train.AdamOptimizer(self.learning_rate).minimize(self.loss)

            self.model = model(sess, MODELS_DIR, model_name = model_name)
            self.summary = self.model.summary(graph = graph, **dict(loss = self.loss))
            self.eval_indice = len(get_folders(pathname(RES_PATH, **dict(flag = 1))))

    def create_network(self, scope_name):
        with tf.variable_scope(scope_name):
                I = tf.placeholder(dtype=tf.float32, shape=(None, self.width, self.height, self.channels))

                I_norm = I / 255.0
                conv1 = Conv2d(I_norm, 5, 64, stride=4, name='conv1').out

                conv2 = Conv2d(conv1, 3, 64, stride=4, name='conv2').out

                conv3 = Conv2d(conv2, 3, 96, stride=4, name='conv3').out

                conv4 = Conv2d(conv3, 3, 128, stride=2, name='conv4').out

                conv5 = Conv2d(conv4, 3, 256, stride=2, name='conv5').out

                conv_flatten = tf.contrib.layers.flatten(conv5)
                J = tf.placeholder(dtype=tf.float32, shape=[None, self.action_dim])

                FC1 = FullyConnected(tf.concat([conv_flatten, J], axis=1), neurons=256, name='FC1_I').out

                FC = tf.expand_dims(FC1, axis=2)

                LSTM_NN = LSTM(FC, neurons=256).out


                VEL = FullyConnected(LSTM_NN[:, -1], neurons=self.action_dim, activation='tanh',
                                     name='velocities').out

        return I, J, VEL

    def evaluate(self):
        # folder = DATA_PATH + 'Trial_950'

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
                vel = self.sess.run(self.vel, {
                    self.rgb: rgb.reshape(-1, WIDTH, HEIGHT, CHANNELS),
                    self.conf: joints_configurations
                })
                # vel = np.concatenate([vel, [[0.0, 0.0, 0.0]]], axis=1)
                I.step(body = robot, values = vel)

                img_array = I.get_camera_image(WIDTH, HEIGHT)[:, :, :3]
                scipy.misc.imsave(pathname(img_path, 'img_%d.png' % i), img_array)
        self.eval_indice += 1

    def evaluate2(self):
            # folder = DATA_PATH + 'Trial_950'

            indice = np.random.randint(1000, 1020)
            folder = pathname(EVALUATION_PATH, 'demo_%d' % indice)
            scene_objects = 'scene_objects.npy'
            with Interface(mode='direct') as I:
                robot = I.load_model(ROBOT_URDF_PATH)
                table = I.load_model(TABLE, Pose(Point(0.4, 0.0, 0.05), Euler(1.57, 0.0, 0.0)),
                                     fixed_base=True, scaling=1.0)
                I.set_camera_pose(0.6, 155, -50, [0.2, 0.0, 0.5])
                scene = np.load(os.path.join(folder, scene_objects))[2:, 0]
                block = I.load_model(CUBE_URDF, Pose(Point(*scene[0])), fixed_base=False)
                basket = I.load_model(BASKET_URDF, Pose(Point(*scene[1])), fixed_base=False)

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
                    vel = self.sess.run(self.vel, {
                        self.rgb: rgb.reshape(-1, WIDTH, HEIGHT, CHANNELS),
                        self.conf: joints_configurations
                    })
                    # vel = np.concatenate([vel, [[0.0, 0.0, 0.0]]], axis=1)
                    # print vel
                    # import sys
                    # sys.exit(0)
                    # print vel[0][6]
                    I.step_with_gripper(robot, vel[0][:6], vel[0][-1])

                    img_array = I.get_camera_image(WIDTH, HEIGHT)[:, :, :3]
                    scipy.misc.imsave(pathname(img_path, 'img_%d.png' % i), img_array)
            self.eval_indice += 1

    def train(self, rgb, joints_configurations, velocities):
        return self.sess.run([self.optimizer, self.loss, self.summary.run(), self.vel], {self.rgb: rgb,
                                                                                         self.conf: joints_configurations,
                                                                                         self.velocities: velocities,
                                                                                         })

    def save_model(self):
        self.model.save()

    def get_velocities(self, rgb, jnt_conf):
        return self.sess.run(self.vel, {self.rgb: rgb,
                                        self.conf: jnt_conf})

def load_data(serialized_features):
    parsed_features = tf.parse_single_example(serialized_features,
                                              {"img": tf.FixedLenFeature((), tf.string),
                                               "joints_conf": tf.FixedLenFeature((), tf.string),
                                               "velocities": tf.FixedLenFeature((), tf.string),
                                               "step": tf.FixedLenFeature((), tf.string)}
                                              )
    parsed_features['img'] = tf.decode_raw(parsed_features['img'], tf.uint8)
    parsed_features['img'] = tf.reshape(parsed_features['img'], [WIDTH, HEIGHT, CHANNELS])
    parsed_features['joints_conf'] = tf.decode_raw(parsed_features['joints_conf'], tf.float64)
    parsed_features['velocities'] = tf.decode_raw(parsed_features['velocities'], tf.float64)
    return parsed_features


def data_func():
    filenames = tf.placeholder(tf.string, shape=[DATASET_NUM])
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

    with tf.Session(graph = graph, config = config) as sess:
        filenames, iterator, next_features = data_func()
        sess.run(iterator.initializer, {filenames: [pathname(DATASET, 'demo_%d.tfrecord' % i)
                                                    for i in range(DATASET_NUM)]})

        visuo_net = VisuoNet(sess, WIDTH, HEIGHT, CHANNELS,
                             model_name = 'visuonet8',
                             graph = graph
                             )

        if not visuo_net.model.restore():
            sess.run(tf.global_variables_initializer())

        for i in range(EPISODES * int(STEPS / BATCH_SIZE)):
            features = sess.run(next_features)

            _, loss, summary, vel= visuo_net.train(features['img'],
                                               features['joints_conf'],
                                               features['velocities'])

            visuo_net.summary.add(summary)
            print(i, loss)

            if i % STEPS == 0:
                visuo_net.save_model()

            if i % 25000 == 0:
                visuo_net.evaluate()
