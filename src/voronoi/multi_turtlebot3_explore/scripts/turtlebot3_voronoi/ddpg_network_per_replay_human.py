#! /usr/bin/env python

# this network can change both heading and velocity
import numpy as np
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input
from keras.layers import Add, Concatenate
from keras.optimizers import Adam
import keras.backend as K
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
import random
from collections import deque
import os.path
import timeit
import csv
import math
import time
import matplotlib.pyplot as plt
import scipy.io as sio
from priortized_replay_buffer import PrioritizedReplayBuffer

def batch_stack_samples(samples):
	array = np.array(samples)
	#before_current_states = np.stack(array[:,0])
	current_states = np.stack(array[:,0]).reshape((array.shape[0],-1))
	actions = np.stack(array[:,1]).reshape((array.shape[0],-1))
	rewards = np.stack(array[:,2]).reshape((array.shape[0],-1))
	new_states = np.stack(array[:,3]).reshape((array.shape[0],-1))
	dones = np.stack(array[:,4]).reshape((array.shape[0],-1))
	weights = np.stack(array[:,5]).reshape((array.shape[0],-1))
	indices = np.stack(array[:,6]).reshape((array.shape[0],-1))
	eps_d = np.stack(array[:,7]).reshape((array.shape[0],-1))



	return current_states, actions, rewards, new_states, dones, weights, indices, eps_d
	

# determines how to assign values to each state, i.e. takes the state
# and action (two-input model) and determines the corresponding value
class ActorCritic:
	def __init__(self, env, sess):
		self.env  = env
		self.sess = sess

		self.learning_rate = 0.0001
		self.epsilon = .9
		self.epsilon_decay = .99995
		self.gamma = .90
		self.tau   = .01


		self.buffer_size = 1000000
		self.batch_size = 256

		self.hyper_parameters_lambda3 = 0.2
		self.hyper_parameters_eps = 0.2
		self.hyper_parameters_eps_d = 0.4

		self.demo_size = 1000

		# ===================================================================== #
		#                               Actor Model                             #
		# Chain rule: find the gradient of chaging the actor network params in  #
		# getting closest to the final value network predictions, i.e. de/dA    #
		# Calculate de/dA as = de/dC * dC/dA, where e is error, C critic, A act #
		# ===================================================================== #

		self.memory = PrioritizedReplayBuffer() #deque(maxlen=40000)
		self.actor_state_input, self.actor_model = self.create_actor_model()
		_, self.target_actor_model = self.create_actor_model()

		self.actor_critic_grad = tf.placeholder(tf.float32,
			[None, self.env.action_space.shape[0]]) # where we will feed de/dC (from critic)

		actor_model_weights = self.actor_model.trainable_weights
		self.actor_grads = tf.gradients(self.actor_model.output,
			actor_model_weights, -self.actor_critic_grad) # dC/dA (from actor)
		grads = zip(self.actor_grads, actor_model_weights)
		self.optimize = tf.train.AdamOptimizer(self.learning_rate).apply_gradients(grads)

		# ===================================================================== #
		#                              Critic Model                             #
		# ===================================================================== #

		self.critic_state_input, self.critic_action_input, \
			self.critic_model = self.create_critic_model()
		_, _, self.target_critic_model = self.create_critic_model()

		self.critic_grads = tf.gradients(self.critic_model.output,
			self.critic_action_input) # where we calcaulte de/dC for feeding above

		# Initialize for later gradient calculations
		self.sess.run(tf.initialize_all_variables())

	# ========================================================================= #
	#                              Model Definitions                            #
	# ========================================================================= #

	def create_actor_model(self):
		state_input = Input(shape=self.env.observation_space.shape)
		h1 = Dense(500, activation='relu')(state_input)
		#h2 = Dense(1000, activation='relu')(h1)
		h2 = Dense(500, activation='relu')(h1)
		h3 = Dense(500, activation='relu')(h2)
		delta_theta = Dense(1, activation='tanh')(h3) 
		speed = Dense(1, activation='sigmoid')(h3) # sigmoid makes the output to be range [0, 1]

		#output = Dense(self.env.action_space.shape[0], activation='tanh')(h3)
		#output = Concatenate()([delta_theta])#merge([delta_theta, speed],mode='concat')
		output = Concatenate()([delta_theta, speed])
		model = Model(state_input, output)
		adam  = Adam(lr=0.0001)
		model.compile(loss="mse", optimizer=adam)
		return state_input, model

	def create_critic_model(self):
		state_input = Input(shape=self.env.observation_space.shape)
		state_h1 = Dense(500, activation='relu')(state_input)
		#state_h2 = Dense(1000)(state_h1)

		action_input = Input(shape=self.env.action_space.shape)
		action_h1    = Dense(500)(action_input)

		merged    = Concatenate()([state_h1, action_h1])
		merged_h1 = Dense(500, activation='relu')(merged)
		merged_h2 = Dense(500, activation='relu')(merged_h1)
		output = Dense(1, activation='linear')(merged_h2)
		model  = Model([state_input,action_input], output)

		adam  = Adam(lr=0.0001)
		model.compile(loss="mse", optimizer=adam)
		return state_input, action_input, model

	# ========================================================================= #
	#                               Model Training                              #
	# ========================================================================= #

	def remember(self, cur_state, action, reward, new_state, done):
		
		indice = len(self.memory.memory_data())

		target_actions = self.target_actor_model.predict(new_state)
		future_rewards = self.target_critic_model.predict([new_state, target_actions])

		rewards = reward + self.gamma* future_rewards * (1 - done)


		# get critic_loss_element_wise and actor_loss_element
		critic_values = self.critic_model.predict([cur_state, action])
		critic_loss_element = np.power((critic_values-rewards), 2)

		predicted_action = self.actor_model.predict(cur_state)
		actor_loss_element = self.critic_model.predict([cur_state, predicted_action])


		new_priorities  = critic_loss_element
		new_priorities += self.hyper_parameters_lambda3 * np.power(actor_loss_element,2)

		new_priorities += self.hyper_parameters_eps

		# a.append(new_priorities[0][0])
		# print(a)

		# new_priorities += self.hyper_parameters_eps_d

		self.memory.add(cur_state, action, reward, new_state, done, indice, new_priorities)  # add to buffer, instead of sampling batch



	def read_human_data(self):
		mat_contents = sio.loadmat('src/turtlebot_ddpg/scripts/fd_replay/play_human_data/human_data_1000.mat')
		a = mat_contents['data']
		
		for i in range(self.demo_size):
			cur_state = a[i][0:28]
			action = a[i][28:30]
			reward = a[i][30]
			new_state = a[i][31:59]
			done = a[i][59]
			cur_state = cur_state.reshape(1,28)
			action = action.reshape(1,2)
			#array_reward = np.array(reward)
			#reward = self.array_reward.reshape(1,1)
			new_state = new_state.reshape(1,28)
			indice = i
			new_priorities = 1
			action[0][1] = action[0][1]/0.26
			# print("angular velocity recorded is %s", action[0][0])
			# print("linear velocity recorded is %s", action[0][1])
			self.memory.add(cur_state, action, reward, new_state, done, indice, new_priorities)


	def _train_critic_actor(self, samples):
 
   		# 1, sample to get states, actions, rewards, new_states, dones
   		# 2, calculate weights, indices, eps_d
   		# 3, get critic_loss_element_wise
   		# 4, train critic based on weights
   		# 5, train actor based on weights
   		# 6, update target network?
   		# 7, update priorities for sampling


   		# 1, sample
		cur_states, actions, rewards, new_states, dones, weights, indices, eps_d = samples #batch_stack_samples(samples)
		target_actions = self.target_actor_model.predict(new_states)
		future_rewards = self.target_critic_model.predict([new_states, target_actions])
		rewards = rewards + self.gamma* future_rewards * (1 - dones)


		# 4, train critic based on weights
		_sample_weight = weights #(rewards/rewards).flatten()
		# print("_sample_weight is %s", _sample_weight)
		evaluation = self.critic_model.fit([cur_states, actions], rewards, verbose=0, sample_weight=_sample_weight)
		# print('\nhistory dict:', evaluation.history)


		# 5, train actor based on weights
		predicted_actions = self.actor_model.predict(cur_states)
		grads = self.sess.run(self.critic_grads, feed_dict={
			self.critic_state_input:  cur_states,
			self.critic_action_input: predicted_actions
		})[0]



		#calculate grads_weight for changing the actor model weight?
		grads_weight = grads
		for i in range(0, len(grads)):
			grads_weight[i][0] = grads[i][0]*_sample_weight[i]
			grads_weight[i][1] = grads[i][1]*_sample_weight[i]
		grads = grads_weight
		self.sess.run(self.optimize, feed_dict={
			self.actor_state_input: cur_states,
			self.actor_critic_grad: grads
		})
		# print("grads*weights is %s", grads)
		


		# 3, get critic_loss_element_wise
		critic_values = self.critic_model.predict([cur_states, actions])
		critic_loss_element = np.power((critic_values-rewards), 2)

		# 7, update priorities for sampling
		actor_loss_element = self.critic_model.predict([cur_states, predicted_actions])
		# print("actor_loss_element is %s", actor_loss_element)

		new_priorities  = critic_loss_element
		new_priorities += self.hyper_parameters_lambda3 * np.power(actor_loss_element,2)
		new_priorities += self.hyper_parameters_eps
		new_priorities += self.hyper_parameters_eps_d
		# print("new_priorities is %s", new_priorities)


		######################################################################
		# update priority of sampled transitions, batch_size.
		self.memory.update_priorities(indices, new_priorities)



	def read_Q_values(self, cur_states, actions):
		critic_values = self.critic_model.predict([cur_states, actions])
		return critic_values

	def train(self):
		batch_size = self.batch_size
		if len(self.memory.memory_data()) < batch_size: #batch_size:
			return
		#samples = random.sample(self.memory.memory_data(), batch_size)    # what is deque, what is random.sample? self.mempory begins with self.memory.append
		samples = self.memory.sample(1, batch_size)
		self.samples = samples
		# print("samples is %s", samples)
		# print("samples [1] is %s", samples[1])
		print("length of memory is %s", len(self.memory.memory_data()))
		# print("samples shape is %s", samples.shape)
		self._train_critic_actor(samples)


	# ========================================================================= #
	#                         Target Model Updating                             #
	# ========================================================================= #

	def _update_actor_target(self):
		actor_model_weights  = self.actor_model.get_weights()
		actor_target_weights = self.target_actor_model.get_weights()
		
		for i in range(len(actor_target_weights)):
			actor_target_weights[i] = actor_model_weights[i]*self.tau + actor_target_weights[i]*(1-self.tau)
		self.target_actor_model.set_weights(actor_target_weights)

	def _update_critic_target(self):
		critic_model_weights  = self.critic_model.get_weights()
		critic_target_weights = self.target_critic_model.get_weights()
		
		for i in range(len(critic_target_weights)):
			critic_target_weights[i] = critic_model_weights[i]*self.tau + critic_target_weights[i]*(1-self.tau)
		self.target_critic_model.set_weights(critic_target_weights)

	def update_target(self):
		self._update_actor_target()
		self._update_critic_target()

	# ========================================================================= #
	#                              Model Predictions                            #
	# ========================================================================= #

	def act(self, cur_state):  # this function returns action, which is predicted by the model. parameter is epsilon
		#self.epsilon *= self.epsilon_decay
		self.epsilon = 0.9
		eps = self.epsilon
		action = self.actor_model.predict(cur_state)
		if np.random.random() < self.epsilon:
			action[0][0] = action[0][0] + (np.random.random()-0.5)*0.4
			action[0][1] = action[0][1] + np.random.random()*0.4
			return action, eps	
		else:
			action[0][0] = (np.random.random()-0.5)*2   # angular velocity
			action[0][1] = np.random.random()   # linear velocity
			return action, eps
	def play(self, cur_state):
		return self.actor_model.predict(cur_state)
	