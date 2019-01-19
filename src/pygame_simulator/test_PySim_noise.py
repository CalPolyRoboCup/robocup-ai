import sys
sys.path.insert(0, '..')
from basic_skills.robot import *
from basic_skills.action import *
from PySim_noise import PYsim
import pygame

from pygame.locals import *
'''
test action
'''
class keyboard_control(action):
  def __init__(self):
    action.__init__(self)
    self.norm_vel = 0
    self.tang_vel = 0
    self.rot_vel = 0
    self.kick = 0
    self.speed = 65
    self.rot_speed = 4
  def keypress_update(self, keys):
    self.norm_vel = 0
    self.tang_vel = 0
    self.rot_vel = 0
    self.kick = 0
    if keys[K_SPACE]:
      self.kick = 1
    if keys[K_d]:
      self.norm_vel = -self.speed
    elif keys[K_a]:
      self.norm_vel = self.speed
    if keys[K_w]:
      self.tang_vel = self.speed
    elif keys[K_s]:
      self.tang_vel = -self.speed
    if keys[K_q]:
      self.rot_vel = self.rot_speed
    elif keys[K_e]:
      self.rot_vel = -self.rot_speed
  def run(self):
    action = [self.kick,0,self.norm_vel, self.tang_vel, self.rot_vel]
    self.action = action
    return action
    
    
#simple test code 
if __name__ == "__main__":
  pygame.init()
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  key_action = keyboard_control()
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  
  for b in range(len(game.yellow_robots)):
    game.add_action(key_action, b, is_blue = True)
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
      if event.type == KEYDOWN or event.type == KEYUP:
        keys = pygame.key.get_pressed()
        key_action.keypress_update(keys)
    new_time = clock.tick()
    #visualization test
    #game.step(key_points = [([0,1000],10), ([0,-1000],-10)])
    game.step()
    ttime = new_time