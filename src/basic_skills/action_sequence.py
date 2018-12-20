
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from pygame_simulator.PySim import *

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
  sequence_action = action_sequence([move_action1, move_action2, move_action3], loop = True)
  game.blue_robots[0].add_action(sequence_action)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    new_time = clock.tick()
    game.step()
    ttime = new_time