import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from pygame_simulator.PySim import *
from basic_skills.helper_functions import *

class Ball_Intercept_PYGym (PYsim):
  def __init__(self, max_bots):
    PYsim.__init__(self,max_bots)
    self.cat = []
    self.dog = []
    self.fish = []
    self.time = 0
  def score(self):
    velocity_loss = np.linalg.norm(self.ball.velocity)
    nearness_loss = np.linalg.norm(self.ball.loc - self.blue_robots[0].loc)
    return velocity_loss + nearness_loss
  def new_scenario(self):
    #self.blue_robots[0].loc = np.random.uniform(-1, 1, size = [2])*np.array([2000, 1500])
    #self.blue_robots[0].rot = np.random.uniform(-math.pi, math.pi)
    random_offset = np.random.uniform(-1, 1, size = [2])*np.array([3000, 2500])
    self.ball.loc = random_offset
    random_offset = np.random.uniform(-1, 1, size = [2])*np.array([1000, 1000])
    target_loc = self.blue_robots[0].loc + random_offset
    speed = np.random.uniform(100,800)
    self.ball.velocity = (target_loc - self.ball.loc)
    self.ball.velocity *= speed/np.linalg.norm(self.ball.velocity)
  def step(self):
    #self.plot_update()
    self.time += 1
    if self.time % 300 == 0:
      self.new_scenario()
    new_state = PYsim.step(self)
    return new_state, self.score()