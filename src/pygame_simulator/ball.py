class ball:
  def __init__(self):
    self.loc = np.array([0,0], dtype = np.float64)
    self.observed = 0
    self.velocity = np.array([0,0], dtype = np.float64)
    
    self.smoothing = 0
    self.first = True
    self.last_timestamp = 0
    self.spin = np.array([0,0], dtype = np.float64)
    self.controler = False
    self.last_controler = False
  def update(self, nloc, obs, time_elapsed = 1.0/60, time_stamp = None):
    self.observed = obs
    if (((time_stamp == None) or (time_stamp != self.last_timestamp)) and 
        obs and np.abs(nloc[0]) < 6000 and np.abs(nloc[1]) < 4000):    
      self.last_timestamp = time_stamp
      self.velocity = (self.velocity * self.smoothing + 
        (1-self.smoothing) * (nloc - self.loc)/time_elapsed)
      self.loc = ((self.loc + self.velocity*time_elapsed) * self.smoothing + 
        (1-self.smoothing) * (nloc))