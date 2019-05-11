#!/usr/bin/env	python

import os
import tensorflow as tf
import numpy as np
from collections import OrderedDict

from model_states_client import set_states
import random
import rospy
from bot_utils.utils import Csv, Plot

RANDOM_SEED = 1234
BUFFER_SIZE = 100
MINIBATCH_SIZE = 25

EPISODES = 1000
STEPS = 10
ACTOR_LEARNING_RATE = 0.001
CRITIC_LEARNING_RATE = 0.01
EXPLORE = 10000.0
GAMMA_FACTOR = 0.95

PARAM_FILE = '../param/training_parameters_2018_06_20.csv'
SUM_FILE = '../SUM_DIR/summary_2018_06_20.csv'

TAU = 0.01

cube_map_pose = OrderedDict([('cube_blue_1', [1.5, -0.3, 0.80]), ('cube_blue_2', [1.5, -0.05, 0.80]),
                             ('cube_red_1', [1.5, 0.2, 0.80]), ('cube_red_2', [1.7, -0.15, 0.80]),
                             ('cube_green_1', [1.65, 0.05, 0.80]), ('cube_green_2', [1.6, 0.32, 0.80])
                             ])
#
# cube_map_pose = OrderedDict([('cube_blue_1', [1.4, -0.3, 0.80]), ('cube_blue_2', [1.4, -0.3, 1.5]),
#                              ('cube_red_1', [1.4, 0.2, 0.83]), ('cube_red_2', [1.6, 0.2, 0.83]),
#                              ('cube_green_1', [1.5, 0.0, 0.83]), ('cube_green_2', [1.5, 0.32, 0.83])
#                              ])

reoder_pose = OrderedDict([('cube_blue_1', [1.8, 0.0, 0.80]), ('cube_blue_2', [1.8, -0.15, 0.80]),
                           ('cube_red_1', [1.8, 0.15, 0.80]), ('cube_red_2', [1.8, 0.3, 0.80]),
                           ('cube_green_1', [1.8, -0.3, 0.80]), ('cube_green_2', [2.0, 0.32, 0.80])
                           ])
SUM_DIR = '../SUM_DIR'
TRAIN_DIR = '../TRAIN_DIR'


class DDPG(object):
	def __init__(self):
		pass
	
	@staticmethod
	def execute(sess, actor, critic, train = True):
		sess.run(tf.global_variables_initializer())
		saver = tf.train.Saver()
		summary_writer = tf.summary.FileWriter(SUM_DIR)
		
		summary_ops = tf.summary.merge_all()
		if os.path.isfile(TRAIN_DIR + '/checkpoint'):
			saver.restore(sess, TRAIN_DIR + '/model_pick.ckpt')
			print('Restored')
		from demo import Demo, GazeboClient
		rospy.init_node('demo')
		env = Demo()
		if train:
			actor.update_target_network()
			critic.update_target_network()
			replaybuffer = Memory(BUFFER_SIZE, RANDOM_SEED)
			
			if os.path.isfile(PARAM_FILE):
				data = Csv(PARAM_FILE).read()
				epsilon = float(data['epsilon'][-1])
				start = int(data['episode'][-1])
			else:
				epsilon = 0.8
				start = 0
			
			gazebo_client = GazeboClient(['ground_plane', 'fetch'])
			
			gazebo_client.delete_gazebo_sdf_models()
			
			load_models = env.gazebo_client.initial_load_gazebo_models()
			
			gazebo_client.skip_models.append('building_0')
			gazebo_client.skip_models.append('cafe_table_scaled_0')
			for i in range(start, EPISODES):
				#
				# for i in range(EPISODES):
				delete_models_name = [names for names in gazebo_client.get_model_states().name if
				                      names.startswith('cube')]
				delete_models_path = [names[:-2] for names in delete_models_name]
				env.gazebo_client.delete_gazebo_sdf_models(delete_models_name)
				env.gazebo_client.shuffle_models(delete_models_name, delete_models_path)
				
				env.cubes_bottom_top()
				
				s_t = env.reset()
				
				total_action_attempt = 0
				total_grasp_attempt = 0
				total_place_attempt = 0
				total_grasp_success = 0
				total_place_success = 0
				total_reward = 0.0
				
				if s_t is not None:
					
					for j in range(STEPS):
						epsilon -= 0.7 / 1000.0
						if np.random.random() > epsilon:
							a_type = "Exploit"
							a = actor.predict(s_t.reshape(-1, 84, 84, 3)).reshape(2, 3)
						else:
							a_type = "Explore"
							a = np.random.random_sample([2, 3])
						
						action = np.argmax(a, axis = 1)
						print(action)
						s_t1, r, terminal, update, grasp_attempt, \
						grasp_success, place_attempt, place_success = env.step(list(action))
						total_action_attempt += 1
						
						try:
							total_reward += r
						except:
							pass
						total_grasp_attempt += int(grasp_attempt)
						total_grasp_success += int(grasp_success)
						total_place_attempt += int(place_attempt)
						total_place_success += int(place_success)
						
						# print('j: ', j,'Rewards: ', r)
						if update:
							replaybuffer.add(s_t.reshape([84, 84, 3]), a, r, terminal, s_t1.reshape([84, 84, 3]))
						if replaybuffer.size() >= MINIBATCH_SIZE:
							s_batch, a_batch, r_batch, t_batch, s2_batch = replaybuffer.sample_batch(MINIBATCH_SIZE)
							target_q = critic.predict_target(np.array(s2_batch).reshape([-1, 84, 84, 3]),
							                                 np.array(a_batch).reshape([-1, 2, 3]))
							y_i = []
							for k in range(MINIBATCH_SIZE):
								if t_batch[k]:
									y_i.append(r_batch[k])
								else:
									y_i.append(r_batch[k] + GAMMA_FACTOR * target_q[k])
							
							critic.train(np.array(s_batch).reshape([-1, 84, 84, 3]),
							             np.array(a_batch).reshape([-1, 2, 3]),
							             np.reshape(y_i, (-1, 1)))
							
							a_outs = actor.predict(np.array(s_batch).reshape([-1, 84, 84, 3]))
							grads = critic.action_gradients(np.array(s_batch).reshape([-1, 84, 84, 3]), a_outs)
							actor.train(np.array(s_batch).reshape([-1, 84, 84, 3]), grads[0])
							actor.update_target_network()
							critic.update_target_network()
						
						if terminal:
							break
						s_t = s_t1
					
					saver.save(sess, TRAIN_DIR + '/model_pick.ckpt')
					Csv(PARAM_FILE).write(headers = ['epsilon', 'episode'], rows = [[epsilon], [i]], mode = 'w')
					Csv(SUM_FILE).write(headers = ['episode', 'rewards', 'total_action_attempt', 'total_grasp_attempt',
					                               'total_grasp_success', 'total_place_attempt', 'total_place_success'],
					                    rows = [[int(i)], [float(total_reward)], [int(total_action_attempt)],
					                            [int(total_grasp_attempt)], [int(total_grasp_success)],
					                            [int(total_place_attempt)], [int(total_place_success)]],
					                    mode = 'a')
					try:
						print ('Episode %d , Reward: %f , Epsilon: %f' % (i, total_reward, epsilon))
					except:
						pass


def conv2d(input, weight_shape, bias_shape):
	stdev = weight_shape[0] * weight_shape[1] * weight_shape[2]
	W = tf.get_variable("W", initializer = tf.truncated_normal(shape = weight_shape, stddev = 2 / np.sqrt(stdev)))
	bias_init = tf.constant_initializer(value = 0)
	b = tf.get_variable("b", bias_shape, initializer = bias_init)
	conv_out = tf.nn.conv2d(input, W, strides = [1, 4, 4, 1], padding = 'VALID')
	return tf.nn.relu(tf.nn.bias_add(conv_out, b))


class ActorNetwork(object):
	
	def __init__(self, sess, state_dim, action_dim, learning_rate, tau):
		self.sess = sess
		self.s_dim = state_dim
		self.a_dim = action_dim
		self.learning_rate = learning_rate
		self.tau = tau
		
		self.actor_inputs_dirt, self.actor_weights, self.actor_out = self.create_actor_network('actor_network')
		self.target_actor_inputs_dirt, self.target_actor_weights, self.target_actor_out = self.create_actor_network(
			'actor_target')
		
		self.update_target_network_params = \
			[self.target_actor_weights[i].assign(tf.multiply(self.actor_weights[i], self.tau) +
			                                     tf.multiply(self.target_actor_weights[i], 1. - self.tau))
			 for i in range(len(self.target_actor_weights))]
		
		self.action_gradient = tf.placeholder(tf.float32, [None] + self.a_dim)
		
		self.actor_gradients = tf.gradients(self.actor_out, self.actor_weights, -self.action_gradient)
		
		self.optimize = tf.train.AdamOptimizer(self.learning_rate).apply_gradients(
			zip(self.actor_gradients, self.actor_weights))
	
	def create_actor_network(self, scope_name):
		with tf.variable_scope(scope_name):
			X = tf.placeholder(tf.float32, shape = [None] + self.s_dim)
			I = tf.to_float(X) / 255.0
			with tf.variable_scope('conv1'):
				H1 = conv2d(I, [8, 8, 3, 32], [32])
			with tf.variable_scope('conv2'):
				H2 = conv2d(H1, [4, 4, 32, 64], [64])
			with tf.variable_scope('dense1'):
				H3 = tf.reshape(H2, [-1, 2, 5 * 5 * 32])
				O = tf.layers.dense(H3, self.a_dim[1], activation = tf.nn.softmax)
			
			W = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope = scope_name)
			return I, W, O
	
	def train(self, inputs, a_gradient):
		self.sess.run(self.optimize, feed_dict = {self.actor_inputs_dirt: inputs,
		                                          self.action_gradient: a_gradient})
	
	def predict(self, inputs):
		return self.sess.run(self.actor_out, feed_dict = {
			self.actor_inputs_dirt: inputs
		})
	
	def predict_target(self, inputs):
		return self.sess.run(self.target_actor_out, feed_dict = {self.target_actor_inputs_dirt: inputs})
	
	def update_target_network(self):
		self.sess.run(self.update_target_network_params)


class CriticNetwork(object):
	
	def __init__(self, sess, state_dim, action_dim, learning_rate, tau):
		self.sess = sess
		self.s_dim = state_dim
		self.a_dim = action_dim
		self.learning_rate = learning_rate
		self.tau = tau
		
		self.critic_inputs_dirt, self.critic_action, self.critic_weights, self.critic_out = self.create_critic_network(
			'critic_network')
		
		self.target_critic_inputs_dirt, self.target_critic_action, self.target_critic_weights, self.target_critic_out = self.create_critic_network(
			'critic_target')
		
		self.update_target_network_params = [
			self.target_critic_weights[i].assign(tf.multiply(self.critic_weights[i], self.tau) +
			                                     tf.multiply(self.target_critic_weights[i], 1. - self.tau))
			for i in range(len(self.target_critic_weights))]
		self.predicted_q_value = tf.placeholder(tf.float32, [None, 1])
		
		self.loss = tf.reduce_mean(tf.square(self.predicted_q_value - self.critic_out))
		self.optimize = tf.train.AdamOptimizer(self.learning_rate).minimize(self.loss)
		self.action_grads = tf.gradients(self.critic_out, self.critic_action)
	
	def create_critic_network(self, scope_name):
		with tf.variable_scope(scope_name):
			X = tf.placeholder(tf.float32, shape = [None] + self.s_dim)
			I = tf.to_float(X) / 255.0
			A = tf.placeholder(dtype = tf.float32, shape = [None, 2, 3])
			with tf.variable_scope('conv1'):
				H1 = conv2d(I, [8, 8, 3, 32], [32])
			with tf.variable_scope('conv2'):
				H2 = conv2d(H1, [4, 4, 32, 64], [64])
			with tf.variable_scope('dense_state'):
				H3 = tf.reshape(H2, [-1, 5 * 5 * 64])
				H4 = tf.layers.dense(H3, 300, activation = tf.nn.relu)
			with tf.variable_scope('dense_action'):
				H5 = tf.layers.dense(A, 300, activation = tf.nn.relu)
			with tf.variable_scope('dense_state_action'):
				H6 = tf.reduce_sum(H5, axis = 1)
				H7 = tf.layers.dense(tf.concat([H4, H6], axis = 1), 300, activation = tf.nn.relu)
			with tf.variable_scope('q_out'):
				O = tf.layers.dense(H7, 1)
			
			W = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope = scope_name)
		return I, A, W, O
	
	def train(self, inputs, action, predicted_q_value):
		return self.sess.run([self.critic_out, self.optimize, self.loss], feed_dict = {
			self.critic_inputs_dirt: inputs,
			self.critic_action: action,
			self.predicted_q_value: predicted_q_value
		})
	
	def predict(self, inputs, action):
		return self.sess.run(self.critic_out, feed_dict = {
			self.critic_inputs_dirt: inputs,
			self.critic_action: action
		})
	
	def predict_target(self, inputs, action):
		return self.sess.run(self.target_critic_out, feed_dict = {
			self.target_critic_inputs_dirt: inputs,
			self.target_critic_action: action
		})
	
	def action_gradients(self, inputs, actions):
		return self.sess.run(self.action_grads, feed_dict = {
			self.critic_inputs_dirt: inputs,
			self.critic_action: actions
		})
	
	def update_target_network(self):
		self.sess.run(self.update_target_network_params)


class Memory(object):
	
	def __init__(self, buffer_size, random_seed = 1234):
		self.buffer_size = buffer_size
		self.count = 0
		np.random.seed(random_seed)
	
	def add(self, s, a, r, t, s2):
		experience = [[s, a, r, t, s2]]
		try:
			if self.buffer.shape[0] >= self.buffer_size:
				self.buffer = np.delete(self.buffer, 0, axis = 0)
				self.concat(experience)
			else:
				self.concat(experience)
		except:
			self.concat(experience)
		self.count = self.buffer.shape[0]
	
	def size(self):
		
		return self.count
	
	def concat(self, experience):
		try:
			self.buffer = np.concatenate((self.buffer, experience), axis = 0)
		except:
			self.buffer = np.array(experience)
	
	def sample_batch(self, batch_size):
		idx = range(batch_size)
		np.random.shuffle(idx)
		batch = self.buffer[idx]
		
		s_batch = [elem.tolist() for elem in batch[:, 0]]
		a_batch = [elem.tolist() for elem in batch[:, 1]]
		r_batch = batch[:, 2]
		t_batch = batch[:, 3]
		s2_batch = [elem.tolist() for elem in batch[:, 4]]
		
		return s_batch, a_batch, r_batch, t_batch, s2_batch


class Summary(object):
	def __init__(self):
		summary = Csv('../SUM_DIR/summary.csv')
		data = summary.read()
		episodes = summary.convert(data['episodes'], int)
		total_action_attempt = summary.convert(data['total_action_attempt'], int)
		total_grasp_attempt = summary.convert(data['total_grasp_attempt'], int)
		total_grasp_success = summary.convert(data['total_grasp_success'], int)
		total_place_attempt = summary.convert(data['total_place_attempt'], int)
		total_place_success = summary.convert(data['total_place_success'], int)
		rewards = summary.convert(data['rewards'], float)
		index = 15
		Plot.plot(episodes[::index], total_place_success[::index], color = 'green',
		          axis_max = {'x': 1650, 'y': 4}, axis_min = {'x': 0, 'y': 0},
		          show = False, save_path = '../total_place_success.png',
		          labels = {'x': 'Episodes', 'y': 'total_place_success'})


if __name__ == '__main__':
	with tf.Session() as sess:
		state_dim = [84, 84, 3]
		action_dim = [2, 3]
		actor = ActorNetwork(sess, state_dim, action_dim, ACTOR_LEARNING_RATE, TAU)
		critic = CriticNetwork(sess, state_dim, action_dim, CRITIC_LEARNING_RATE, TAU)
		DDPG().execute(sess, actor, critic, train = True)

# Summary()
