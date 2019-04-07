import sys
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from pygame_simulator.PySim import *

import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons

class action_sequence(action):
  #covers a pass from target_robot to target_loc
  def __init__(self, sequence = [], loop = False):
    action.__init__(self)
    self.sequence = sequence
    self.loop = loop
    self.ind = 0
    self.finished = False
  def add(self, robot, game):
    for s in self.sequence:
      s.add(robot, game)
    action.add(self, robot, game)
  def run(self):
    #if len(self.sequence) != 0:
    #  break
    if self.sequence[self.ind].done():
      self.ind += 1
      if self.ind == len(self.sequence) and self.loop:
        self.ind = 0
      elif self.ind == len(self.sequence):
        self.finished = True
        self.ind -= 1
    actions = self.sequence[self.ind].run()
    return actions
  def done(self):
    return self.finished
    

if __name__ == "__main__":
  game = PYsim(6)
  move_action1 = move_to([1000,1000], 0)
  move_action2 = move_to([-1000,-1000], 3.14)
  move_action3 = move_to([1000,-1000], 1.57)
  sequence_action_triangle = action_sequence([move_action1, move_action2, move_action3], loop = True)
  
  move_action1 = move_to([1000,1000], 0)
  move_action2 = move_to([1000,-1000], 3.14)
  move_action3 = move_to([-1000,-1000], 1.57)
  move_action4 = move_to([-1000,1000], -1.57)
  sequence_action_square = action_sequence([move_action1, move_action2, move_action3, move_action4], loop = True)
  
  move_action1 = move_to([2000,1000], 1.57)
  move_action2 = move_to([1300,0], 2.1)
  move_action3 = move_to([600,0], 3.4)
  move_action4 = move_to([-600,2000], 4.1)
  move_action5 = move_to([-1300,2000], 2.6)
  move_action6 = move_to([-2000,1000], 1.57)
  move_action7 = move_to([-1300,0], .86)
  move_action8 = move_to([-600,0], -.86)
  move_action9 = move_to([600,2000], -.86)
  move_action0 = move_to([1300,2000], .86)
  sequence_action_figure8 = action_sequence([move_action1, move_action2, move_action3, move_action4, move_action5, move_action6, move_action7, move_action8, move_action9, move_action0], loop = True)
  game.blue_robots[0].add_action(sequence_action_triangle)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
      if event.type == KEYDOWN or event.type == KEYUP:
        keys = pygame.key.get_pressed()
        #press r-key to reset
        if keys[K_r]:
          game.reset()
        if keys[K_1]:
          game.add_action(sequence_action_triangle, 0, True)
        if keys[K_2]:
          game.add_action(sequence_action_square, 0, True)
        if keys[K_3]:
          game.add_action(sequence_action_figure8, 0, True)
          
    new_time = clock.tick()
    game.step()
    ttime = new_time
  
