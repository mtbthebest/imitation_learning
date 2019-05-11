# coding=utf-8
""" A Network for trajectory generation """
import os
import tensorflow as tf
import numpy as np
from tf_utils import FullyConnected, Conv2d, model, LSTM
from utils import pathname, mkdir, Img
from utils import Pd, get_files, get_folders
from bullet import Interface, Point, Pose, Euler
import scipy.misc
import time

DATASET = '/data/barry/IMITATION/dataset/THROW/TJR'
EVALUATION_PATH = '/data/barry/IMITATION/dataset/THROW/DEMO'
RES_PATH = '/data/barry/IMITATION/dataset/THROW/POSE_NET_EVAL'
DATASET_NUM = 1000

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

LAMBDA = 0.1


def CNN(I, scope_name):
    I_norm = I / 255.0

    conv1 = Conv2d(I_norm, 5, 64, stride = 4, name = scope_name + '/conv1').out

    conv2 = Conv2d(conv1, 3, 64, stride = 4, name = scope_name + '/conv2').out

    conv3 = Conv2d(conv2, 3, 96, stride = 4, name = scope_name + '/conv3').out

    conv4 = Conv2d(conv3, 3, 128, stride = 2, name = scope_name + '/conv4').out

    conv5 = Conv2d(conv4, 3, 256, stride = 2, name = scope_name + '/conv5').out

    return conv5

class PoseNetwork:
    def __init__(self, sess, width = 256, height = 256, channels = 3,
                 action_dim = 6, learning_rate = 0.01,
                 model_name = None, graph = None):
        self.sess = sess
        self.width = width
        self.height = height
        self.channels = channels
        self.action_dim = action_dim
        self.learning_rate = learning_rate
        self.prev_rgb, self.goal_rgb, self.prev_jnt, self.next_jnt = self.create_network('visuomotor')

        self.next_jnt_pred = tf.placeholder(tf.float32, [None, self.action_dim])

        self.loss = tf.reduce_mean(tf.square(self.next_jnt - self.next_jnt_pred))
        self.optimizer = tf.train.AdamOptimizer(self.learning_rate).minimize(self.loss)

        self.model = model(sess, MODELS_DIR, model_name = model_name)
        self.summary = self.model.summary(graph = graph, **dict(loss = self.loss))
        self.eval_indice = len(get_folders(pathname(RES_PATH, **dict(flag = 1))))

    def create_network(self, scope_name):
        with tf.variable_scope(scope_name):
            ACT_IMG = tf.placeholder(dtype = tf.float32, shape = (None, self.width, self.height, self.channels))

            GOAL_IMG = tf.placeholder(dtype = tf.float32, shape = (None, self.width, self.height, self.channels))

            # I = tf.concat([ACT_IMG, GOAL_IMG], axis = 3)
            ACT_CNN = CNN(ACT_IMG, 'prev')
            GOAL_CNN = CNN(GOAL_IMG, 'goal')

            img_flatten = tf.contrib.layers.flatten(ACT_CNN)
            goal_flatten = tf.contrib.layers.flatten(GOAL_CNN)

            ACT_JNT = tf.placeholder(dtype = tf.float32, shape = [None, self.action_dim])

            FC1 = FullyConnected(tf.concat([img_flatten, goal_flatten, ACT_JNT], axis = 1), neurons = 256,
                                 name = 'FC1_I').out

            FC = tf.expand_dims(FC1, axis = 2)

            LSTM_NN = LSTM(FC, neurons = 256).out

            FC2 = FullyConnected(LSTM_NN[:, -1], neurons = 256, name = 'FC2').out

            NEXT_JNT = FullyConnected(FC2, neurons = self.action_dim, activation = 'linear', name = 'pose').out

        return ACT_IMG, GOAL_IMG, ACT_JNT, NEXT_JNT

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
            goal = img[-1]
            joints = I.movable_a_joints + I.movable_g_joints

            eval_path = mkdir(RES_PATH, 'EVAL_%d' % (self.eval_indice + 1))
            img_path = mkdir(eval_path, 'IMG')
            with open(pathname(eval_path, 'eval.txt'), 'w') as f:
                f.write('Evaluation number: %d' % indice)

            for i in range(STEPS):
                joints_configurations = [[state.jointPosition for state in I.get_joint_state(robot, joints)]]
                rgb = I.get_camera_image(WIDTH, HEIGHT)[:, :, :3]
                poses = self.sess.run(self.next_jnt, {
                    self.prev_rgb: rgb.reshape(-1, WIDTH, HEIGHT, CHANNELS),
                    self.goal_rgb: goal.reshape(-1, WIDTH, HEIGHT, CHANNELS),
                    self.prev_jnt: joints_configurations
                })

                I.step_with_pose(body = robot, joints = joints, poses = poses[0])

                # img_array = I.get_camera_image(WIDTH, HEIGHT)[:, :, :3]
                scipy.misc.imsave(pathname(img_path, 'img_%d.png' % i), rgb)
        self.eval_indice += 1

    def train(self, prev_rgb, goal_rgb, prev_jnt, next_jnt):
        return self.sess.run([self.optimizer, self.loss, self.summary.run()], {self.prev_rgb: prev_rgb,
                                                                               self.goal_rgb: goal_rgb,
                                                                               self.prev_jnt: prev_jnt,
                                                                               self.next_jnt_pred: next_jnt})

    def save_model(self):
        self.model.save()


def load_data(serialized_features):
    keys = {'prev_img': tf.FixedLenFeature((), tf.string),
            'next_img': tf.FixedLenFeature((), tf.string),
            'prev_jnt_conf': tf.FixedLenFeature((), tf.string),
            'next_jnt_conf': tf.FixedLenFeature((), tf.string),
            'goal_img': tf.FixedLenFeature((), tf.string),
            'step': tf.FixedLenFeature((), tf.string)
            }
    parsed_features = tf.parse_single_example(serialized_features, keys)

    for img_key in ['prev_img', 'next_img', 'goal_img']:
        parsed_features[img_key] = tf.decode_raw(parsed_features[img_key], tf.uint8)
        parsed_features[img_key] = tf.reshape(parsed_features[img_key], [WIDTH, HEIGHT, CHANNELS])
    parsed_features['prev_jnt_conf'] = tf.decode_raw(parsed_features['prev_jnt_conf'], tf.float64)
    parsed_features['next_jnt_conf'] = tf.decode_raw(parsed_features['next_jnt_conf'], tf.float64)

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

if __name__ == '__main__':
    graph = tf.get_default_graph()

    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True

    with tf.Session(graph = graph, config = config) as sess:
        filenames, iterator, next_features = data_func()
        sess.run(iterator.initializer,
                 {filenames: [pathname(DATASET, 'demo_%d.tfrecord' % i)
                              for i in range(DATASET_NUM)]
                  })

        pose_net = PoseNetwork(sess, WIDTH, HEIGHT, CHANNELS,
                               action_dim = 9,
                               model_name = 'pose_net', graph = graph)

        if not pose_net.model.restore():
            sess.run(tf.global_variables_initializer())

        for i in range(EPISODES * int(STEPS / BATCH_SIZE)):
            features = sess.run(next_features)

            _, loss, summary = pose_net.train(features['prev_img'],
                                              features['goal_img'],
                                              features['prev_jnt_conf'],
                                              features['next_jnt_conf'])
            print(i, loss)

            pose_net.summary.add(summary)

            if i % STEPS == 0:
                pose_net.save_model()

            if i % 86000 == 0 and i>0:
                pose_net.evaluate()

    # graph = tf.get_default_graph()
    # with tf.Session(graph = graph) as sess:
    #     pose_net = PoseNetwork(sess, WIDTH, HEIGHT, CHANNELS,
    #                            action_dim = 9,
    #                            model_name = 'pose_net', graph = graph)
