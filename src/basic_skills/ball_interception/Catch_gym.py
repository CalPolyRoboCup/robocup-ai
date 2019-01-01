import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '../../../src')
from GR_sim_networking.GR_Interact import *
from basic_skills.helper_functions import *

class Ball_Intercept_Gym (GRsim):
  def __init__(self, max_bots):
    GRsim.__init__(self,max_bots)
    self.cat = []
    self.dog = []
    self.fish = []
    self.time = 0
  def score(self):
    velocity_loss = np.linalg.norm(self.ball.velocity)
    nearness_loss = np.linalg.norm(self.ball.loc - self.blue_robots[0].loc)
    return velocity_loss + nearness_loss
  def plot_update(self):
    self.cat.append(self.blue_robots[0].action.action[4])
    hold_offset = self.blue_robots[0].loc - self.ball.loc
    target = normalize_angle(math.pi + math.atan2(hold_offset[1], hold_offset[0]))
    distance = min_angle(target - self.blue_robots[0].rot)
    self.dog.append(distance)
    self.fish.append(self.blue_robots[0].rot_vel)
    if (0 == self.time % 200):
      plt.figure(2)
      c, = plt.plot(self.cat, label = "action")
      d, = plt.plot(self.dog, label = "rot")
      f, = plt.plot(self.fish, label = "rot_vel")
      plt.legend([c,d,f])
      plt.show(block = False)
      plt.pause(1E-12)
      plt.clf()
      self.cat = []
      self.dog = []
      self.fish = []
  def reset(self):
    self.blue_robots[0].loc = np.random.uniform(-1, 1, size = [2])*np.array([2000, 1500])
    self.blue_robots[0].rot = np.random.uniform(-math.pi, math.pi)
    random_offset = np.random.uniform(-1, 1, size = [2])*np.array([3000, 2500])
    self.ball.loc = random_offset
    random_offset = np.random.uniform(-1, 1, size = [2])*np.array([1000, 1000])
    target_loc = self.blue_robots[0].loc + random_offset
    speed = np.random.uniform(1, 25)
    self.ball.velocity = (target_loc - self.ball.loc)
    self.ball.velocity *= speed/np.linalg.norm(self.ball.velocity)
    self.push_state()
  def step(self):
    #self.plot_update()
    self.time += 1
    if self.time % 100 == 0:
      self.reset()
    new_state = GRsim.step(self)
    return new_state, self.score()
