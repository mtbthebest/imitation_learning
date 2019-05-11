# coding=utf-8
"""Prepocessing with TF.data.record.writer"""

import tensorflow as tf
from tf_utils import _int64_feature, _bytes_feature, normalize_image
from utils import pathname, Img, get_folders, get_files, mkdir, plot_img_from_array
import os
import numpy as np
from collections import OrderedDict
import pandas as pd
import scipy.misc
import time

# -----------------Testing -----------------#
# DIR = '/home/mtb/sim_ws/pybullet_ws/data/Trial_950/'
# IMG_PATH = 'IMG'
# JOINT_PATH = '/home/mtb/sim_ws/pybullet_ws/data/Trial_950/joints_configurations.npy'
# VEL_PATH = '/home/mtb/sim_ws/pybullet_ws/data/Trial_950/velocities.npy'


# -----------------------Server----------------------#
DIR = '/data/barry/IMITATION/dataset/THROW/DEMO'

RESOLUTION = 256
DATA_NUM = 1472


def read_tf_records(in_file,indice =0):
    record_iterator = tf.python_io.tf_record_iterator(path = in_file)
    TRIAL_PATH = pathname(DIR,'Trial_%d'%indice)
    img_files = get_files(pathname(TRIAL_PATH, 'IMG'), key = lambda order: int(order[4:-4]))
    img_array = np.concatenate([[np.load(img_file)[:, :, :3]] for img_file in img_files[1:1473]])
    vel_array = np.load(pathname(TRIAL_PATH, 'velocities.npy'))[1:1473]
    joint_array = np.load(pathname(TRIAL_PATH, 'joints_configurations.npy'))[0:1472]
    RES_PATH = pathname(DIR, '..','TEST_%d' %indice)
    mkdir(pathname(RES_PATH, 'SIM'))
    mkdir(pathname(RES_PATH, 'TF'))
    for s in ['TF', 'SIM']:
        for elem in ['IMG', 'JOINTS', 'VELOCITIES']:
            mkdir(pathname(RES_PATH, s, elem))
            mkdir(pathname(RES_PATH, s, elem))
            mkdir(pathname(RES_PATH, s, elem))


    for i, string_record in enumerate(record_iterator):
        example = tf.train.Example()
        example.ParseFromString(string_record)
        img_tf = np.fromstring(example.features.feature['img'].bytes_list.value[0], dtype = np.uint8).reshape(256, 256, 3)

        scipy.misc.imsave(pathname(RES_PATH,'TF', 'IMG', 'img_%d.png'%i), img_tf)
        scipy.misc.imsave(pathname(RES_PATH,'SIM', 'IMG', 'img_%d.png'%i), img_array[i])

        pose_tf = np.fromstring(example.features.feature['joint_conf'].bytes_list.value[0], dtype = np.float64)
        np.save(pathname(RES_PATH,'TF', 'JOINTS', 'joints_%d'%i), pose_tf)
        np.save(pathname(RES_PATH,'SIM', 'JOINTS', 'joints_%d'%i), joint_array[i])

        vel_tf = np.fromstring(example.features.feature['velocities'].bytes_list.value[0], dtype = np.float64)
        np.save(pathname(RES_PATH, 'TF', 'VELOCITIES', 'velocities_%d' % i), vel_tf)
        np.save(pathname(RES_PATH, 'SIM', 'VELOCITIES', 'velocities_%d' % i), vel_array[i])

def check(indice= 0):
    TEST_PATH = '/data/barry/IMITATION/dataset/THROW/TEST_%d'%indice

    joint_sim =  np.concatenate([[np.load(pathname(TEST_PATH,'SIM', 'JOINTS','joints_%d.npy')%j)] for j in range(1472)])
    joint_tf =  np.concatenate([[np.load(pathname(TEST_PATH,'TF', 'JOINTS','joints_%d.npy')%j)] for j in range(1472)])

    for i in range(5):
        k = np.random.randint(0,1472)
        print('k: ', k)

        print joint_sim[k]
        print joint_tf[k]


def write_tf_records_db(out_file, img_array, joint_conf_array, velocities_array,gripper_array, step_array):
      with tf.python_io.TFRecordWriter(out_file) as writer:
        for k in range(step_array.shape[0]):
            image = img_array[k]
            example = tf.train.Example(features = tf.train.Features(feature = {
                'img': _bytes_feature(image.tostring()),
                'joints_conf': _bytes_feature(joint_conf_array[k].tostring()),
                'velocities': _bytes_feature(velocities_array[k].tostring()),
                'gripper': _bytes_feature(gripper_array[k].tostring()),
                'step': _bytes_feature(step_array[k].tostring())
            }))
            writer.write(example.SerializeToString())


def create_visuomotor_dataset():
    # for datasets in  get_files(pathname(DIR, '..', 'RECORDS', 'demo_%d.tfrecord'),
    #                            key = lambda f : int(f[5:-9])):

    filename_list = get_files(pathname('/home/mtb/sim_ws/pybullet_ws/data/Trial_950/IMG'),
                              key = lambda f: int(f[4:-4]))
    q = tf.FIFOQueue(capacity = 5, dtypes = tf.string)
    enqueue_op = q.enqueue_many([filename_list])

    qr = tf.train.QueueRunner(q, [enqueue_op] * 1)
    tf.train.add_queue_runner(qr)

    data = q.dequeue()

    # data = tf.Print(data, data = [q.size(),data], message = 'Remaining file in q')
    file_queued = data


def create_queue():
    filename_list = get_files(pathname('/home/mtb/sim_ws/pybullet_ws/data/Trial_950/IMG'),
                              key = lambda f: int(f[4:-4]))
    q = tf.FIFOQueue(capacity = 2, dtypes = tf.string)
    enqueue_op = q.enqueue([filename_list])

    data = q.dequeue()

    # data = tf.Print(data, data = [data], message = 'Remaining file in q')
    file_queued = data

    with tf.Session() as sess:
        # coord = tf.train.Coordinator()
        # threads = tf.train.start_queue_runners(coord = coord)
        sess.run(enqueue_op)
        queue = sess.run(file_queued)

        # sess.run(file_queued)
        # sess.run(file_queued)
        # sess.run(file_queued)
        # for t in range(10):
        #     queue = sess.run(file_queued)
        #     print queue
        # coord.request_stop()
        # coord.join(threads)

class Visuomotor:
    def set_visuomotor_dataset(self):
        for i, trial_fold in enumerate(get_folders(DIR, key = lambda name: int(name[5:]))):
            print(i)
            IMG = pathname(trial_fold, 'IMG')
            # scipy.misc.imsave('/data/barry/IMITATION/dataset/THROW/TEST/img_%d.png'%i, np.load(pathname(IMG, 'img_1472.npy'))[:,:,:3])
            img_files = get_files(IMG, key = lambda order: int(order[4:-4]))
            img_array = np.concatenate([[np.load(img_file)[:, :, :3]] for img_file in img_files[1:DATA_NUM + 1]])
            joint_conf_array = np.load(pathname(trial_fold, 'joints_configurations.npy'))[0:DATA_NUM]
            velocities = np.load(pathname(trial_fold, 'velocities.npy'))
            velocities_array =velocities[1:DATA_NUM + 1]
            gripper_array = np.zeros(DATA_NUM)

            first_gripper = velocities_array[:,6]
            closing, opening = np.min(np.where(first_gripper > 0.9)), np.min(np.where(first_gripper < -0.9))
            gripper_array[closing:opening] = 1.0


            assert velocities_array.shape[0] == joint_conf_array.shape[0] == img_array.shape[0] == gripper_array.shape[0] # == velocities_conf.shape[0]

            write_tf_records_db(pathname(DIR, '..', 'VISUOMOTOR_GRP', 'demo_%d.tfrecord' % i),
                                        img_array,
                                        joint_conf_array,
                                        velocities_array,
                                gripper_array=gripper_array,
                               step_array= np.array(['step_%d_%d'%(i, j) for j in range(DATA_NUM)]))


def write_tjr_dataset():
    for i, trial_fold in enumerate(get_folders(DIR, key=lambda name: int(name[5:]))):
        print(i)
        IMG = pathname(trial_fold, 'IMG')
        # scipy.misc.imsave('/data/barry/IMITATION/dataset/THROW/TEST/img_%d.png'%i, np.load(pathname(IMG, 'img_1472.npy'))[:,:,:3])
        img_files = get_files(IMG, key = lambda order: int(order[4:-4]))

        img_array = np.concatenate([[np.load(img_file)[:, :, :3]]
                                    for img_file in img_files[0:DATA_NUM + 1]])
        prev_img_array = img_array[0:DATA_NUM]
        next_img_array = img_array[1:DATA_NUM + 1]
        goal_array = img_array[-1]
        initial_img_array = img_array[0]

        joint_conf_array = np.load(pathname(trial_fold, 'joints_configurations.npy'))[0:DATA_NUM + 1]
        prev_joint_conf_array = joint_conf_array[0:DATA_NUM]
        next_joint_conf_array = joint_conf_array[1:DATA_NUM + 1]

        vel_conf = np.load(pathname(trial_fold, 'velocities.npy'))[0:DATA_NUM]
        assert prev_img_array.shape[0] == next_img_array.shape[0] == \
               prev_joint_conf_array.shape[0] == next_joint_conf_array.shape[0] \
               == vel_conf.shape[0]  # == velocities_conf.shape[0]

        step_array = np.array(['step_%d_%d' % (i, j) for j in range(DATA_NUM)])

        with tf.python_io.TFRecordWriter(pathname(DIR, '..', 'TJR', 'demo_%d.tfrecord' % i)) as writer:
            for k in range(step_array.shape[0]):
                example = tf.train.Example(features=tf.train.Features(feature={
                    'prev_img': _bytes_feature(prev_img_array[k].tostring()),
                    'next_img': _bytes_feature(next_img_array[k].tostring()),
                    'prev_jnt_conf': _bytes_feature(prev_joint_conf_array[k].tostring()),
                    'next_jnt_conf': _bytes_feature(next_joint_conf_array[k].tostring()),
                    'goal_img': _bytes_feature(goal_array.tostring()),
                    'initial_img': _bytes_feature(initial_img_array.tostring()),
                    'vel_conf': _bytes_feature(vel_conf[k].tostring()),
                    'step': _bytes_feature(step_array[k].tostring())
                }))
                writer.write(example.SerializeToString())



if __name__ == '__main__':
    write_tjr_dataset()
