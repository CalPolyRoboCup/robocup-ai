import sys
sys.path.insert(0, '../..')
from pygame_simulator.PySim_noise import *
from basic_skills.helper_functions import *

class Ball_Intercept_PYGym (PYsim):
  def __init__(self, max_bots):
    PYsim.__init__(self,max_bots)
    self.cat = []
    self.dog = []
    self.fish = []
    self.time = 0
    
    self.control_time = 300
    self.control_counter = 0
    
  def score(self):
    velocity_loss = np.linalg.norm(self.ball.velocity)
    nearness_loss = np.linalg.norm(self.ball.loc - self.blue_robots[0].loc)
    return velocity_loss + nearness_loss
    
  def new_scenario(self):
    '''
    put the ball in a new position and kick it at the blue robot with id 0
    '''
    random_offset = np.random.uniform(-1, 1, size = [2])*np.array([3000, 2500])
    self.ball_internal.loc = random_offset
    random_offset = np.random.uniform(-1, 1, size = [2])*np.array([500, 500])
    target_loc = self.blue_robots[0].loc + random_offset
    speed = np.random.uniform(100,800)
    self.ball_internal.velocity = (target_loc - self.ball.loc)
    self.ball_internal.velocity *= speed/np.linalg.norm(self.ball_internal.velocity)
    print(self.ball_internal.velocity, self.ball_internal.loc)
    self.control_counter = 0
    
  def step(self):
    '''
    kick balls at the blue robot with id 0 and wait for it to catch it
    '''
    self.time += 1
    if self.ball.controler != False:
      if self.control_counter == 0:
        self.control_counter = self.control_time
      else:
        self.control_counter -= 1
        if self.control_counter == 0:
          self.new_scenario()
    new_state = PYsim.step(self)
    return new_state, self.score()