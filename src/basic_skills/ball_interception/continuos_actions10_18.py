import signal
import argparse
import os.path
import threading
import time
import copy
import matplotlib.pyplot as plt

import gym
import numpy as np
import tensorflow as tf
import tensorflow.contrib.eager as tfe

from architecture6 import *

#import sys
#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

class priority_leaf:
  def __init__(self, ind, value):
    self.ind = ind
    self.value = value
  def grow(self, ind, value):
    replacement = priority_tree(self.value + value)
    replacement.low = self
    replacement.high = priority_leaf(ind, value)
    return replacement
  def sample(self):
    return self

class priority_tree:
  def __init__(self, value):
    self.low = False
    self.high = False
    self.cycle = False
    self.ind = -1
    self.value = value
  def grow(self, ind, value):
    self.value += value
    if not self.low:
      self.low = priority_leaf(ind, value)
    elif not self.high:
      self.high = priority_leaf(ind, value)
    elif not self.cycle:
      if self.low.ind != -1:
        self.low = self.low.grow(ind, value)
      else:
        self.low.grow(ind, value)
    else:
      if self.high.ind != -1:
        self.high = self.high.grow(ind, value)
      else:
        self.high.grow(ind, value)
  def sample(self, value = False):
    if not value:
      value = np.random.random(self.value)
    if value > self.low.value:
      return self.high.sample(value - self.low.value)
    return self.low.sample(value)
  
  
class sequential_prioritized_memory_replay:
  def __init__(self, max_size = 600, min_useful_size = 600):
    self.list_s = []
    self.list_r = []
    self.list_d = []
    self.list_a = []
    self.list_i = []
    self.priority = []
    self.priority_total = 0
    self.length = 0
    self.priority_clip = 5
    self.max_size = max_size
    self.full_flag = threading.Lock()
    self.full_flag.acquire(False)
    self.min_useful_size = min_useful_size
    self.update_lock = threading.Lock()
  def append (self, state, reward, discount, action, priority = None, discrete = True):
    if self.length < self.max_size:
      self.length += 1
      if self.length == self.max_size or self.length == self.min_useful_size:
        self.full_flag.release()
    else:
      self.list_s.pop(0)
      self.list_r.pop(0)
      self.list_d.pop(0)
      self.list_a.pop(0)
      if (discrete):
        self.list_i.pop(0)
      self.priority_total -= self.priority.pop(0)
    self.list_s.append(state)
    self.list_r.append(reward)
    self.list_d.append(discount)
    if discrete:
      self.list_a.append(tf.squeeze(action[0]))
      self.list_i.append(tf.squeeze(action[1]))
    else:
      self.list_a.append(action[0])
      #self.list_i.append(tf.squeeze(action[1]))
    if priority:
      if priority < self.priority_clip*self.priority_total/self.length:
        self.priority.append(priority)
        self.priority_total += priority
      else:
        self.priority.append(self.priority_total/self.length)
        self.priority_total += self.priority_total/self.length
    else:
      self.priority.append(1E-2)
      self.priority_total += 1E-2
  def extend (self, values, priorities = False, discrete = True):
    with self.update_lock:
      if priorities:
        for v, p in zip(values, priorities):
          self.append(v[0],[1],v[2],v[3], p, discrete = discrete)
      else:
        for v in values:
          self.append(v[0],[1],v[2],v[3], discrete = discrete)
  def sample_batch(self, batch_size, batch_length, discrete_indicies = True):
    #multinomial needs floats for logits otherwise it complains that there is "no valid device" it is a type error not a hardware problem
    val_index = tf.clip_by_value(tf.multinomial(tf.expand_dims(self.priority,0), batch_size)[0], batch_length - 1, self.length)
    #print(val_index)
    ret_s = []
    ret_r = []
    ret_d = []
    ret_i = []
    ret_a = []
    for vi in val_index:
      viv = int(vi)
      with self.update_lock:
        ret_s.append(self.list_s[viv-batch_length+1:viv+1])
        ret_r.append(self.list_r[viv-batch_length+1:viv+1])
        ret_d.append(self.list_d[viv-batch_length+1:viv+1])
        if discrete_indicies:
          ret_i.append(self.list_i[viv-batch_length+1:viv+1])
        ret_a.append(self.list_a[viv-batch_length+1:viv+1])
    ret_s = tf.stack(ret_s, axis = 1)
    ret_r = tf.stack(ret_r, axis = 1)
    ret_d = tf.stack(ret_d, axis = 1)
    ret_a = tf.stack(ret_a, axis = 1)
    if discrete_indicies:
      ret_i = tf.stack(ret_i, axis = 1)
    return ret_s, ret_r, ret_d, ret_a, ret_i, val_index
  def update(self, indices, priority):
    for ind in range(int(indices.shape[0])):
      pind = int(indices[ind]) - priority.shape[0]
      for p in priority[:,ind]:
        self.priority_total -= self.priority[pind]
        if float(p) < self.priority_clip*self.priority_total/self.length:
          self.priority[pind] = float(p)
        else:
          self.priority[pind] = self.priority_total/self.length
        self.priority_total += self.priority[pind]
        pind += 1
  
class Learner:
  def __init__(self, in_size, action_size, buffer, discount_factor = .99, learning_rate = 1E-6, discrete = False, high_low = False, default_path = "C:/Users/nathan/Documents/AI stuff/basic testing/eager_restores"):
    if discrete:
      self.base, self.actor, self.critic = build_net(in_size, action_size)
    else:
      self.base, self.actor, self.critic = build_net(in_size, action_size)
    self.buffer = buffer
    self.saver = tfe.Saver(self.base.variables + self.actor.variables + self.critic.variables)
    self.save_path = os.path.join(default_path, "eager.ckpt")
    self.c_clip = 2 #called p hat in IMPALA paper
    self.p_clip = 2 #called c hat in IMPALA paper
    self.in_size = in_size
    self.action_size = action_size
    self.discount_factor = discount_factor
    self.similarity_constant = .001
    self.learning_rate = learning_rate
    self.optimizer = tf.train.AdamOptimizer(self.learning_rate)
    self.running = False
    self.discrete = discrete
    self.default_path = default_path
    self.high_low = high_low
  def restore(self, path):
    self.saver.restore(path)
  def save (self, path):
    self.saver.save(file_prefix = path) 
  def __call__(self, state, stochastic = True, critique = True):
    base = self.base(state, stochastic = stochastic, time_major = True)
    actions = self.actor(base, stochastic = stochastic)
    if self.discrete:
      actions = tf.nn.softmax(actions)
    else:
      actions = tf.nn.sigmoid(actions)*(self.high_low[0] - self.high_low[1]) + self.high_low[1]
    if critique:
      base_actions = tf.concat([base, actions], axis = -1)
      value_estimate = self.critic(base_actions)
      return actions, value_estimate
    return actions
  def reset(self, batch_size):
    self.base.reset(batch_size)
    self.actor.reset(batch_size)
    self.critic.reset(batch_size)
  def action_similarity(self, actions_old, actions_new, indices = False): #, actions_old_index = None
    if self.discrete:
      rho = tf.reduce_sum(actions_new * tf.one_hot(indices, self.action_size), axis = -1)/(actions_old+1E-7)
    else:
      rho = (tf.reduce_sum(actions_new - actions_old, axis = -1)*self.similarity_constant+1)**(-2)
      #this is an attempt to make the actions treat larger values as more and negative values as opposite
      #similarity = (actions_old*actions_new + self.action_size)/tf.clip_by_value(tf.reduce_sum(actions_old, axis = -1), 1E-8, 1E8)
      #return tf.clip_by_value(similarity, 0, self.c_clip), tf.clip_by_value(similarity, 0, self.p_clip)
    return tf.minimum(rho, self.c_clip), tf.minimum(rho, self.p_clip)
  def V_trace_targets(self, obs, rewards, discounts, actions, value_estimate, current_policy_actions, indices = False):
    with tf.device('/cpu'):
      values_t_plus_1 = value_estimate[1:]
      cs, clipped_rhos = self.action_similarity(actions, current_policy_actions, indices)
      # cs = tf.ones_like(actions)
      # clipped_rhos = tf.ones_like(actions)
      print("cs rho: ", cs[:,0], clipped_rhos[:,0])
      
      deltas = clipped_rhos[:-1] * (tf.to_float(tf.squeeze(rewards[:-1])) + discounts[:-1] * values_t_plus_1 - tf.squeeze(value_estimate[:-1]))

      # Note that all sequences are reversed, computation starts from the back.
      sequences = (
          tf.reverse(cs[:-1], axis=[0]),
          tf.reverse(deltas, axis=[0]),
          tf.reverse(discounts[:-1], axis = [0])
      )
      # V-trace vs are calculated through a scan from the back to the beginning
      # of the given trajectory.
      
      
      #note: V(Xs+1) term is not included in the future discount
      #this is because the V(Xs) term is already absent from the 
      #calculation and is added in
      #
      # vs = vs_minus_v_xs + value_estimate[:-1]
      #
      #after the scan
      def scanfunc(acc, sequence_item):
        c_t, delta_t, discount = sequence_item
        return delta_t + discount * c_t * acc
      initial_values = tf.zeros([rewards.shape[1]])
      vs_minus_v_xs = tf.scan(
          fn=scanfunc,
          elems=sequences,
          initializer=initial_values,
          parallel_iterations=1,
          back_prop=False,
          name='scan')
      vs_minus_v_xs = tf.reverse(vs_minus_v_xs, [0], name='vs_minus_v_xs')
      
      vs = vs_minus_v_xs + value_estimate[:-1]
      
      # plt.figure(2)
      # d, = plt.plot(deltas[:,0], label = "deltas")
      # v, = plt.plot(vs_minus_v_xs[:,0], label = "vs-xs")
      # t, = plt.plot(vs[:,0], label = "target")
      # e, = plt.plot(value_estimate[:-1,0], label = "estimate")
      # #er, = plt.plot((value_estimate[:-1,0]-vs[:,0])**2, label = "error")
      # plt.legend(handles=[d,v,t,e])
      # plt.show(block = False)
      # plt.pause(1E-12)
      # plt.clf()
      
      #if (self.discrete):
      vs_t_plus_1 = tf.concat([vs[1:], tf.expand_dims(value_estimate[-1], axis = 0)], axis = 0)
      advantage = clipped_rhos[:-1] * (tf.to_float(tf.squeeze(rewards[:-1])) + discounts[:-1] * vs_t_plus_1 - value_estimate[:-1])
      return tf.stop_gradient(vs), tf.stop_gradient(advantage), clipped_rhos#, tf.stack(TD_error, axis = 0)
      # else:
        # return tf.stop_gradient(vs)
  def learn(self, batch_size, batch_length, epochs = -1):
    epoch = 0
    self.running = True
    leshire_cat = []
    while epoch != epochs and self.running:
      #if self.discrete:
      batch_obs, batch_reward, batch_discounts, batch_actions, batch_indicies, indices = self.buffer.sample_batch(batch_size, batch_length, discrete_indicies = True)
      # else:
        # batch_indicies = False
        # batch_obs, batch_reward, batch_discounts, batch_actions, indices = self.buffer.sample_batch(batch_size, batch_length, discrete_indicies = False)
      with tfe.GradientTape(persistent = True) as tape:
        actions, value_estimate = self(batch_obs)
        value_estimate = tf.squeeze(value_estimate)
        #if self.discrete:
        VT_target, advantage, rho = self.V_trace_targets(batch_obs, batch_reward, batch_discounts, batch_actions, value_estimate, actions, indices = batch_indicies)
        # else:
          # VT_target = self.V_trace_targets(batch_obs, batch_reward, batch_discounts, batch_actions, value_estimate, actions)
        TD_loss = (tf.squeeze(value_estimate[1:]) - VT_target)**2
        critic_loss = tf.reduce_mean(TD_loss) + .01*self.base.weight_normalization()
        #actor_loss = -tf.reduce_mean(value_estimate)
        if self.discrete:
          one_hots = tf.one_hot(batch_indicies[:-1], self.action_size)
          log_prob = tf.reduce_sum(one_hots*tf.log(actions[:-1]+1E-8), axis = -1)
          action_negentropy = tf.reduce_sum(actions * tf.log(actions+1E-8))
          #print("entropy", action_entropy)
          #print("old act", batch_indicies, batch_actions[:,0]);
          actor_loss = -tf.reduce_mean(log_prob * advantage) + action_negentropy*.01 + .01*self.base.weight_normalization()
        else:
          actor_loss = -tf.reduce_mean(value_estimate) - self.base.noise_magnitude()*.0 - tf.reduce_mean(TD_loss)*.0 + tf.reduce_mean(actions)
      critic_grads = tape.gradient(critic_loss, self.critic.variables + self.base.variables)
      self.optimizer.apply_gradients(zip(critic_grads, self.critic.variables + self.base.variables), 
        global_step=tf.train.get_global_step())
      actor_grads = tape.gradient(actor_loss, self.actor.variables + self.base.variables)
      self.optimizer.apply_gradients(zip(actor_grads, self.actor.variables + self.base.variables), 
        global_step=tf.train.get_global_step())
      self.buffer.update(indices, TD_loss)
      epoch += 1
      leshire_cat.append([actor_loss, critic_loss])
      #print(epoch, critic_loss, actor_loss, self.actor.noise_magnitude())
      if epoch > 10 and epoch % 5 == 0:
        plt.figure(2)
        d, = plt.plot(value_estimate[:,0], label = "estimate")
        v, = plt.plot(VT_target[:,0], label = "value")
        #asym = self.action_similarity(batch_actions, actions, indices = batch_indicies)
        #s, = plt.plot(asym[0][:,0], label = "action_similarity")
        plt.legend(handles=[d,v])
        plt.show(block = False)
        plt.pause(1E-12)
        plt.clf()
        leshire_cat.pop(0)
        # print("advantage", advantage[:,0])
        # print("new act", actions[:,0])
        # print("representation", self.base(batch_obs[:,0]))
        # print("obs", batch_obs[:,0])
    self.saver.save(file_prefix = self.default_path)
  def terminate(self):
    self.running = False
    
      
class Actor:
  def __init__(self, in_size, action_size, buffer, master_net, discount_factor = .99, discrete = True, high_low = False):
    self.in_size = in_size
    self.action_size = action_size
    self.buffer = buffer
    self.master_net = master_net
    self.base, self.actor, self.critic = build_net(in_size, action_size)
    self.discount_factor = discount_factor
    self.running = False
    self.discrete = discrete
    self.high_low = high_low
  def __call__(self, state, stochastic = True, critique = False):
    #print("state", state.shape)
    base = self.base(state, stochastic = stochastic, time_major = True)
    actions = self.actor(base, stochastic = stochastic)
    # if self.discrete:
      # actions = tf.nn.softmax(actions)
    # else:
      # actions = tf.nn.sigmoid(actions[:action_size] + actions[action_size:] * tf.random_normal([actions.shape[:-1],self.action_size]))*(self.high_low[0] - self.high_low[1]) + self.high_low[1]
    base_actions = tf.concat([base, actions], axis = -1)
    if critique:
      value_estimate = self.critic(base_actions)
      return actions, value_estimate
    return actions
  def reset(self, batch_size):
    self.base.reset(batch_size)
    self.actor.reset(batch_size)
    self.critic.reset(batch_size)
  def refresh_vars(self):
    self.base.update(self.master_net.base)
    self.actor.update(self.master_net.actor)
    self.critic.update(self.master_net.critic)
  def act(self, env, max_frames, render = False, epochs = -1, batches = 10):
    epoch = 0
    self.running = True
    environs = [copy.deepcopy(env) for b in range(batches)]
    obs = [e.reset() for e in environs]
    #print("obs", obs)
    while epoch != epochs and self.running:
      trace = [[] for b in range(batches)]
      for _ in range(max_frames):
        if render:
          environs[0].render()
        n_obs = []
        j = 0
        action = self(tf.expand_dims(tf.stack(obs, axis = 0), axis = 0))[0]
        if self.discrete:
          action_activated = tf.softmax(action)
          action_index = tf.multinomial(action_activated, 1)
        else:
          action_activated = action * (self.high_low[0] - self.high_low[1]) + self.high_low[1]
          #print("aa ", action_activated, action, (self.high_low[0] - self.high_low[1]) + self.high_low[1], self.action_size, tf.expand_dims(tf.stack(obs, axis = 0), axis = 0).shape)
          # dist = tf.distributions.Beta(tf.exp(action[:,:self.action_size]), tf.exp(action[:,self.action_size:]))
          # action_activated = dist.sample()
          # action_pdf = tf.exp(dist.log_prob(action_activated))
          # print("pdf", action_pdf)
          # print("aa", action_activated)
          # print("alpha", tf.exp(action[:,:self.action_size]))
          # print("beta", tf.exp(action[:,self.action_size:]))
        for e in environs:
          if self.discrete:
            n_ob, reward, done, info = e.step(int(action_index[j])) 
          else:
            n_ob, reward, done, info = e.step(action_activated) 
            n_ob = tf.squeeze(n_ob)
          #print("nob", n_ob)
          n_obs.append(n_ob)
          if done:
            n_ob = e.reset()
          #print("nob", n_ob.shape)
          if self.discrete:
            trace[j].append([obs[j], reward, tf.to_float(not done) * self.discount_factor, [tf.reduce_sum(action_activated[j]*tf.one_hot(action_index[j], self.action_size)), action_index[j]]])
          else:
            trace[j].append([obs[j], reward, tf.to_float(not done) * self.discount_factor, [action_activated[j], None]])
          j += 1
        obs = n_obs
        self.reset(1)
      for t in trace:
        self.buffer.extend(t, discrete = self.discrete)
      self.refresh_vars()
      epoch += 1
  def terminate(self):
    self.running = False
      

  
def build_net(in_size, action_size):
  base_net = network()
  actor_net = network()
  critic_net = network()
  
  base_linear_units = in_size + [10,10]
  #base_residual_units = [base_linear_units[-1], 10]
  actor_units = [base_linear_units[-1], action_size]#[base_residual_units[-1], action_size]
  critic_units = [base_linear_units[-1] + action_size, 1]#[base_residual_units[-1] + action_size, 1]
  
  for i in range(len(base_linear_units) - 1):
    base_net.add(weight_normalized_linear(base_linear_units[i+1], base_linear_units[i], scope = "base_linear" + str(i)))
  # for i in range(len(base_residual_units) - 1):
    # base_net.add(GR_proxy(base_residual_units[i+1], base_residual_units[i], scope = "base_residual" + str(i)))
    #base_net.add(GRU_python(base_residual_units[i+1], base_residual_units[i], scope = "base_residual" + str(i)))
  for i in range(len(actor_units) - 1):
    actor_net.add(weight_normalized_linear(actor_units[i+1], actor_units[i], scope = "actor" + str(i)))
  for i in range(len(critic_units) - 1):
    critic_net.add(weight_normalized_linear(critic_units[i+1], critic_units[i], scope = "critic" + str(i)))
    
  return base_net, actor_net, critic_net
  
def request_stop(signal, frame):
  print('You pressed Ctrl+C!')
  global stop_requested
  stop_requested = True
  
if __name__ == "__main__":
  tf.enable_eager_execution()
  tf.train.get_or_create_global_step()
  num_actors = 3
  time_out = 0
  batch_size = 10
  batch_length = 30
  max_frames = 50
  discrete = False
  
  discount_factor = .99
  
  env = gym.make('Pendulum-v0')
  signal.signal(signal.SIGINT, request_stop)
  if discrete:
    action_space = env.action_space.n
    state_space = list(env.observation_space.shape)
    high_low = False
  else:
    action_space = len(env.action_space.high)
    state_space = [len(env.observation_space.low)]
    high_low = [env.action_space.high, env.action_space.low]
  buffer = sequential_prioritized_memory_replay()
  Learner = Learner(state_space, action_space, buffer, discrete = discrete, discount_factor = discount_factor, high_low = high_low)
  Learner_thread = threading.Thread(target = Learner.learn, 
                  args = (batch_size,batch_length,))
  Actors = [Actor(state_space, action_space, buffer, Learner, discrete = discrete, discount_factor = discount_factor, high_low = high_low) for a in range(num_actors)]
  Actor_threads = [threading.Thread(target = Actors[i].act, 
                  args = (env, max_frames, i==0,)) for i in range(num_actors)]
                  
  for At in Actor_threads:
    At.start()
  
  buffer.full_flag.acquire(True)
  Learner_thread.start()
  
  if time_out:
    time.sleep(time_out)
  
    stop_requested = True
    for A in Actors:
      A.terminate()
    Learner.terminate()
    save_path = saver.save(sess, args.save_path + "/model.ckpt")
    print("Model saved in file: %s" % save_path)
    print(model.test(make_batch(True))[0])
  else:
    while 1:
      if input()[0] == "q":
        for A in Actors:
          A.terminate()
        Learner.terminate()
  
  
  