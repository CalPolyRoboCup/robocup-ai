import tensorflow as tf
import tensorflow.contrib.eager as tfe

def max_pool_2x2(x):
  return tf.nn.max_pool(x, ksize=[1, 2, 2, 1],
                        strides=[1, 2, 2, 1], padding='SAME')


  def __init__(self, units, in_size, scope):
    if in_size != units:
      self.l0 = weight_normalized_linear(units, in_size, scope)
    else:
      lsef.l0 = False
    self.l1 = weight_normalized_linear(units, units, scope)
    self.l2 = weight_normalized_linear(units, units, scope)
    with tf.name_scope(scope):
      self.mix = tfe.Variable(tf.random_normal(stddev=0.1,shape = [units]), dtype = tf.float32, name = "mix")
    self.variable_list = self.l1.variable_list + [self.mix] + self.l2.variable_list
    if self.l0:
      self.variable_list += self.l0.variable_list
  def __call__(self, input, verbose = False):
    if self.l0:
      preprop = tf.nn.leaky_relu(self.l0(input, verbose))
    else:
      preprop = input
    l1r = tf.nn.leaky_relu(self.l1(preprop, verbose))
    l2r = tf.nn.leaky_relu(self.l2(l1r, verbose))
    #print(l1r.shape)
    #print(self.mix.shape)
    #print(input.shape)
    res = tf.nn.leaky_relu(l2r * self.mix + preprop * (1 - self.mix))
    return res
    
class network:
  def __init__(self):
    self.layers = []
    self.variables = []
  def add(self, layer):
    self.layers.append(layer)
    self.variables.extend(layer.variable_list)
  def __call__(self, inputs, resets = False, stochastic = False, time_major = True, flip_time_batch = False):
    res = inputs
    for l in self.layers:
      #print("res", res.shape)
      if type(l) is GRU_scan or type(l) is network:
        #print("res GRU", res)
        res = l(res, resets = resets, stochastic = stochastic, time_major = time_major, flip_time_batch = flip_time_batch)
      else:
        res = l(res, stochastic = stochastic)
    return res
  def update (self, network):
    for v, nv in zip(self.variables, network.variables):
      tf.assign(v, nv)
  def reset(self, batch_size):
    for l in self.layers:
      if type(l) is GRU_scan or type(l) is network:
        l.reset(batch_size)
  def weight_normalization(self):
    ret = 0
    for l in self.layers[:-1]:
      ret += l.weight_normalization()
    return ret;
  def noise_magnitude(self):
    ret = 0
    for l in self.layers:
      ret += l.noise_magnitude()
    return ret;
    
class residual_linear:
  def __init__(self, units, in_size, scope, window_size = [5,5], strides = [1, 1, 1, 1]):
    if in_size != units:
      self.l0 = weight_normalized_linear(units, in_size, scope, window_size = window_size, strides = strides)
    else:
      self.l0 = False
    self.l1 = weight_normalized_linear(units, units, scope, window_size = window_size, strides = strides)
    self.l2 = weight_normalized_linear(units, units, scope, window_size = window_size, strides = strides)
    with tf.name_scope(scope):
      self.mix = tfe.Variable(tf.random_normal(stddev=0.1, shape = [units]), dtype = tf.float32, name = "mix")
    self.variable_list = self.l1.variable_list + [self.mix] + self.l2.variable_list
    if self.l0:
      self.variable_list += self.l0.variable_list
  def __call__(self, input, stochastic = False, verbose = False):
    if self.l0:
      preprop = tf.nn.leaky_relu(self.l0(input, verbose, stochastic = stochastic))
    else:
      preprop = input
    l1r = tf.nn.leaky_relu(self.l1(preprop, verbose, stochastic = stochastic))
    l2r = tf.nn.leaky_relu(self.l2(l1r, verbose, stochastic = stochastic))
    #print(l1r.shape)
    #print(self.mix.shape)
    #print(input.shape)
    res = tf.nn.leaky_relu(l2r * self.mix + preprop * (1 - self.mix))
    return res
    
class residual_conv:
  def __init__(self, units, in_size, scope, window_size = [5,5], strides = [1, 1, 1, 1], pool = False):
    if in_size != units:
      self.l0 = weight_normalized_conv(units, in_size, scope, window_size = window_size, strides = strides)
    else:
      self.l0 = False
    self.l1 = weight_normalized_conv(units, units, scope, window_size = window_size, strides = strides)
    self.l2 = weight_normalized_conv(units, units, scope, window_size = window_size, strides = strides)
    with tf.name_scope(scope):
      self.mix = tfe.Variable(tf.random_normal(stddev=0.1, shape = [units]), dtype = tf.float32, name = "mix")
    self.variable_list = self.l1.variable_list + [self.mix] + self.l2.variable_list
    if self.l0:
      self.variable_list += self.l0.variable_list
    self.pool = pool
  def __call__(self, input, stochastic = False, verbose = False):
    if self.l0:
      preprop = tf.nn.leaky_relu(self.l0(input, verbose, stochastic = stochastic))
    else:
      preprop = input
    l1r = tf.nn.leaky_relu(self.l1(preprop, verbose, stochastic = stochastic))
    l2r = tf.nn.leaky_relu(self.l2(l1r, verbose, stochastic = stochastic))
    #print(l1r.shape)
    #print(self.mix.shape)
    #print(input.shape)
    res = tf.nn.leaky_relu(l2r * self.mix + preprop * (1 - self.mix))
    if self.pool:
      res = max_pool_2x2(res)
    return res

class residual_conv_bottleneck:
  def __init__(self, units, in_size, scope, window_size = [5,5], strides = [1, 1, 1, 1], pool = False):
    bottle_size = int(units/4)
    if in_size != units:
      self.l0 = weight_normalized_conv(bottle_size, in_size, scope, window_size = window_size, strides = strides)
    else:
      lsef.l0 = False
    self.l1 = weight_normalized_conv(bottle_size, bottle_size, scope, window_size = window_size, strides = strides)
    self.l2 = weight_normalized_conv(bottle_size, bottle_size, scope, window_size = window_size, strides = strides)
    self.l3 = weight_normalized_conv(units, bottle_size, scope, window_size = window_size, strides = strides)
    with tf.name_scope(scope):
      self.mix = tfe.Variable(tf.random_normal(stddev=0.1, shape = [units]), dtype = tf.float32, name = "mix")
    self.variable_list = self.l1.variable_list + [self.mix] + self.l2.variable_list + self.l3.variable_list
    if self.l0:
      self.variable_list += self.l0.variable_list
    self.pool = pool
  def __call__(self, input, stochastic = False, verbose = False):
    if self.l0:
      preprop = tf.nn.leaky_relu(self.l0(input, verbose, stochastic = stochastic))
    else:
      preprop = input
    l1r = tf.nn.leaky_relu(self.l1(preprop, verbose, stochastic = stochastic))
    l2r = tf.nn.leaky_relu(self.l2(l1r, verbose, stochastic = stochastic))
    l3r = tf.nn.leaky_relu(self.l3(l2r, verbose, stochastic = stochastic))
    #print(l1r.shape)
    #print(self.mix.shape)
    #print(input.shape)
    res = tf.nn.leaky_relu(l3r * self.mix + preprop * (1 - self.mix))
    if self.pool:
      res = max_pool_2x2(res)
    return res

class linear:
  def __init__(self, units, in_size, scope, activation = tf.nn.leaky_relu):
    with tf.name_scope(scope):
      self.bias = tfe.Variable(tf.constant(0.1, shape = [units]), dtype = tf.float32, name = "bias")
      self.weight = tfe.Variable(tf.random_normal(stddev=0.1,shape=[in_size] + [units]), dtype = tf.float32, name = "weight")
      
      self.bias_noise = tfe.Variable(tf.constant(.1, shape = [units]), dtype = tf.float32, name = "bias_noise")
      self.weight_noise = tfe.Variable(tf.random_normal(stddev=.1,shape=[in_size] + [units]), dtype = tf.float32, name = "weight_noise")
    self.variable_list = [self.bias, self.weight, self.weight_noise, self.bias_noise] #, self.weight_dist, self.weight_bias
    self.units = units
    self.in_size = in_size
    self.activation = activation
  def __call__(self, input, stochastic = False, verbose = False):
    if stochastic:
      weights = self.weight + self.weight_noise*tf.random_normal(shape = [self.in_size, self.units], mean = 0.0, stddev = 1.0)
      biases = self.bias + self.bias_noise*tf.random_normal(shape = [self.units], mean = 0.0, stddev = 1.0)
    else:
      weights = self.weight
      biases = self.bias
    input = tf.to_float(input)
    if len(input.shape) != 2:
      original_arrangement = list(input.shape)
      original_arrangement[-1] = self.units
      fix_dims = 1
      for ishape in input.shape[:-1]:
        fix_dims*=ishape
      input = tf.reshape(input, [fix_dims, self.in_size])
      return tf.reshape(tf.matmul(input, weights) + biases, original_arrangement)
    #print(input)
    return self.activation(tf.matmul(input, weights) + biases)
    
class weight_normalized_linear:
  def __init__(self, units, in_size, scope, activation = tf.nn.leaky_relu):
    with tf.name_scope(scope):
      self.bias = tfe.Variable(tf.constant(0.1, shape = [units]), dtype = tf.float32, name = "bias")
      self.weight = tfe.Variable(tf.random_normal(stddev=0.1,shape=[in_size] + [units]), dtype = tf.float32, name = "weight")
      
      self.bias_noise = tfe.Variable(tf.constant(.1, shape = [units]), dtype = tf.float32, name = "bias_noise")
      self.weight_noise = tfe.Variable(tf.random_normal(stddev=.1,shape=[in_size] + [units]), dtype = tf.float32, name = "weight_noise")
      
      self.weight_dist = tfe.Variable(tf.constant(.1), dtype = tf.float32, name = "weight_dist")
      self.weight_bias = tfe.Variable(tf.constant(0.0), dtype = tf.float32, name = "weight_bias")
    self.variable_list = [self.bias, self.weight, self.weight_noise, self.bias_noise, self.weight_dist, self.weight_bias] #, self.weight_dist, self.weight_bias
    self.w_reparam = 0
    self.units = units
    self.in_size = in_size
    self.activation = activation
  def weight_normalization(self):
    return tf.reduce_mean(self.w_reparam**2)
  def noise_magnitude(self):
    return tf.reduce_mean(self.weight_noise**2)+tf.reduce_mean(self.bias_noise**2);
  def __call__(self, input, stochastic = False, clip = False, verbose = False):
    if stochastic:
      weights = self.weight + self.weight_noise*tf.random_normal(shape = [self.in_size, self.units], mean = 0.0, stddev = 1.0)
      biases = self.bias + self.bias_noise*tf.random_normal(shape = [self.units], mean = 0.0, stddev = 1.0)
    else:
      weights = self.weight
      biases = self.bias
    w_mean = tf.reduce_mean(weights)
    self.w_reparam = (weights - w_mean)/(tf.reduce_mean(weights**2) - w_mean**2)*(self.weight_dist) + self.weight_bias
    #print("wnl", tf.reduce_mean((self.weight - w_mean), axis = 0))
    #print(tf.clip_by_value(tf.reduce_mean(self.weight**2) - w_mean**2, 1E-9, 1E9))
    #print(input.shape, self.w_reparam.shape, self.bias.shape)
    input = tf.to_float(input)
    if len(input.shape) != 2:
      #print(input, input.shape)
      original_arrangement = list(input.shape)
      original_arrangement[-1] = self.units
      fix_dims = 1
      for ishape in input.shape[:-1]:
        fix_dims*=ishape
      #print(input.shape)
      input = tf.reshape(input, [fix_dims, self.in_size])
      return tf.reshape(tf.matmul(input, self.w_reparam) + biases, original_arrangement)
    return self.activation(tf.matmul(input, self.w_reparam) + biases)
    
class weight_normalized_conv:
  def __init__(self, units, in_size, scope, window_size = [5,5], strides = [1, 1, 1, 1], pool = False, activation = tf.nn.leaky_relu):
    self.window_size = window_size
    self.strides = strides
    with tf.name_scope(scope):
      self.bias = tfe.Variable(tf.constant(0.1, shape = units), dtype = tf.float32, name = "bias")
      self.weight = tfe.Variable(tf.random_normal(stddev=0.1,shape = window_size + [in_size] + [units]), dtype = tf.float32, name = "weight")
      
      self.bias_noise = tfe.Variable(tf.constant(1, shape = units), dtype = tf.float32, name = "bias_noise")
      self.weight_noise = tfe.Variable(tf.random_normal(stddev=1,shape=window_size + [in_size] + [units]), dtype = tf.float32, name = "weight_noise")
      
      self.weight_dist = tfe.Variable(tf.constant(1E-3), dtype = tf.float32, name = "weight_dist")
      self.weight_bias = tfe.Variable(tf.constant(0.0), dtype = tf.float32, name = "weight_bias")
    self.variable_list = [self.bias, self.weight, self.weight_dist, self.weight_bias]
    self.pool = pool
    self.units = units
    self.in_size = in_size
    self.activation = activation
  def __call__(self, input, stochastic = False, verbose = False):
    if stochastic:
      weights = self.weight + self.weight_noise *tf.random_normal(shape = self.window_size + [self.in_size, self.units], mean = 0.0, stddev = 1.0)
      biases = self.bias + self.bias_noise *tf.random_normal(shape = self.window_size + [self.in_size, self.units], mean = 0.0, stddev = 1.0)
    else:
      weights = self.weight
      biases = self.bias
    w_mean = tf.reduce_mean(weights)
    w_reparam = (weights - w_mean)/(tf.reduce_mean(weights**2) - w_mean**2)*(self.weight_dist) + self.weight_bias
    #print(w_reparam.shape)
    #print(input.shape)
    #print(self.strides)
    res = tf.nn.conv2d(input, w_reparam, strides=self.strides, padding='SAME') + biases
    if self.pool:
      res = max_pool_2x2(res)
    return self.activation(res)
    
class GR_cell(tf.contrib.rnn.RNNCell):
  def __init__(self, units, in_size, scope, module = weight_normalized_linear):
    super().__init__(units)
    self.units = units
    self.activation = tf.nn.tanh
    self.write_gate = module(units = units, in_size = units + in_size, scope = scope + "write_gate")
    self.update = module(units = units, in_size = units + in_size, scope = scope + "update")
    self.stochastic = False
  def __call__(self, input, state):
      both = tf.concat([state, input], axis = -1)
      write_factor = tf.nn.sigmoid(self.write_gate(both, stochastic = self.stochastic))
      update = self.activation(self.update(both, stochastic = self.stochastic))
      nv = write_factor*update+state*(1-write_factor)
      return nv, nv
  @property
  def state_size(self):
    return self.units
  @property
  def output_size(self):
    return self.units
    
class GR_proxy:
  def __init__(self, size, in_size, scope, cell = GR_cell, module = weight_normalized_linear):
    #self.cell = tf.nn.rnn_cell.GRUCell(size)
    self.cell = GR_cell(size, in_size, scope, module = module)
    #self.variable_list = self.cell.variables
    self.variable_list = self.cell.write_gate.variable_list + self.cell.update.variable_list
    self.state_batch_size = 1
    self.state = tf.zeros([1,size])
    self.scope = scope
    self.size = size
  def __call__(self, input, time_major = True, stochastic = False):
    self.cell.stochastic = stochastic
    if (self.state_batch_size != input.shape[1] and time_major):
      self.state_batch_size = input.shape[1]
      self.state = tf.zeros([self.state_batch_size,self.size])
    elif (self.state_batch_size != input.shape[0] and not time_major):
      self.state_batch_size = input.shape[0]
      self.state = tf.zeros([self.state_batch_size,self.size])
    #print(input.shape, "rnn")
    outs, self.state = tf.nn.dynamic_rnn(self.cell, tf.convert_to_tensor(input), time_major=time_major, initial_state=self.state, scope="dynamic_rnn" + self.scope)
    return outs
    
class GRU_python:
  def __init__(self, units, in_size, scope, module = weight_normalized_linear):
    self.module = module
    self.units = units
    self.in_size = in_size
    self.activation = tf.nn.tanh
    #self.read_gate = module(units = units, in_size = units + in_size, scope = scope + "read_gate")
    self.write_gate = module(units = units, in_size = units + in_size, scope = scope + "write_gate")
    self.update = module(units = units, in_size = units + in_size, scope = scope + "update")
    self.last_state = tf.zeros([units])
    self.variable_list = self.update.variable_list + self.write_gate.variable_list
  def __call__(self, input, resets = False, stochastic = False, time_major = True, flip_time_batch = False):
    index = 0
    if resets != False:
      print("input top", input)
    def gru(last_state, input):
      if resets and resets[index - 1] and index:
        both = tf.concat([tf.zeros_like(last_state), input], axis = -1)
      else:
        both = tf.concat([last_state, input], axis = -1)
      write_factor = tf.nn.sigmoid(self.write_gate(both, stochastic = stochastic))
      update = self.activation(self.update(both, stochastic = stochastic))
      return write_factor*update+last_state*(1-write_factor)
    results = []
    if time_major:
      if self.last_state.shape[0] != input.shape[1]:
        self.last_state = tf.zeros([input.shape[1], self.units])
      result = self.last_state
      for batch in input:
        result = gru(result, batch)
        results.append(result)
      self.last_state = result[0]
      if flip_time_batch:
        transpose_shape = list(range(len(input.shape)))
        transpose_shape[0] = 1
        transpose_shape[1] = 0
        return tf.transpose(tf.stack(results), transpose_shape)
      return tf.stack(results)
    else:
      if self.last_state.shape[0] != input.shape[0]:
        self.last_state = tf.zeros([input.shape[0], self.units])
      result = self.last_state
      transpose_shape = list(range(len(input.shape)))
      transpose_shape[0] = 1
      transpose_shape[1] = 0
      #print(transpose_shape)
      for batch in tf.transpose(input, transpose_shape):
        result = gru(result, batch)
        results.append(result)
      self.last_state = result[0]
      if flip_time_batch:
        return tf.stack(results)
      return tf.transpose(tf.stack(results), transpose_shape)
  def reset(self, batch_size):
    self.last_state = tf.zeros([batch_size, self.units])

class GRU_scan:
  #doesn't work
  def __init__(self, units, in_size, scope, module = weight_normalized_linear):
    self.module = module
    self.units = units
    self.in_size = in_size
    self.activation = tf.nn.tanh
    #self.read_gate = module(units = units, in_size = units + in_size, scope = scope + "read_gate")
    self.write_gate = module(units = units, in_size = units + in_size, scope = scope + "write_gate")
    self.update = module(units = units, in_size = units + in_size, scope = scope + "update")
    self.last_state = tf.zeros([units])
    self.variable_list = self.update.variable_list + self.write_gate.variable_list
  def __call__(self, input, resets = False, stochastic = False, time_major = False):
    index = 0
    #print("input", input)
    def gru(last_state, input):
      #print("ls", last_state)
      #print("input", input)
      if resets and resets[index - 1] and index:
        both = tf.concat([tf.zeros_like(last_state), input], axis = -1)
      else:
        both = tf.concat([last_state, input], axis = -1)
      write_factor = tf.nn.sigmoid(self.write_gate(both, stochastic = stochastic))
      update = self.activation(self.update(both, stochastic = stochastic))
      return write_factor*update+last_state*(1-write_factor)
    if time_major:
      results = tf.scan(gru, input, initializer = self.last_state)
      self.last_state = results[-1]
    else:
      results = tf.transpose(tf.scan(gru, tf.transpose(input), initializer = self.last_state))
      self.last_state = results[-1]
    return results
  def reset():
    self.last_state = tf.zeros([units])

