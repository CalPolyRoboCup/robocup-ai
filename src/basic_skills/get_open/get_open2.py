import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.helper_functions import *
from basic_skills.action import *
from basic_skills.move_to.move_to import *

from pygame_simulator.PySim import *
from basic_skills.ball_interception.Ball_Interception import *
from basic_skills.ball_interception.Catch_Pygym import *

class get_open(action):
  def __init__(self, points, weights, blockers):
    action.__init__(self)
    self.points = points
    self.weights = weights
    self.blockers = blockers
    self.move_to = np.array([0,0])
    
    self.stand_off_distance = 750

    self.pid = move_to()
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
  def run(self):
    imrpovements = np.array([0,0])
    improv_weight = 0
    pind = 0
    for p in self.points:
      ball_to_robot_speed = 2
      worst = -1
      first = True
      worst_local = np.array([0,0])
      pvec = np.array(p) - np.array(self.robot.loc)
      pangle = -math.atan2(pvec[1], pvec[0])
      pmag = np.linalg.norm(pvec)
      best_loc = np.array([0,0])
      #print()
      for b in self.blockers:
        local = convert_local(b - self.robot.loc, pangle)
        if local[0] < 0:
          dist = np.linalg.norm(local)
          dist += dist/ball_to_robot_speed
        elif local[0] > pmag:
          dist = np.linalg.norm(local - np.array([pmag, 0]))
          dist -= local[0]/ball_to_robot_speed
        else:
          dist = local[1]
          dist -= local[0]/ball_to_robot_speed
        if first or dist < worst:
          worst = dist
          worst_local = local
          worst_loc = b
          first = False
        #print(b, local, p, dist)
        
      if worst < 0:
        worst = 0
        
      importance = abs(7000/(100+abs(worst_local[1])))
      if worst_local[1] < 0:
        improvement = np.array([worst_local[0], importance])
      else:
        improvement = np.array([worst_local[0], -importance])
      if np.linalg.norm(improvement) < self.stand_off_distance:
        improvement = improvement * self.stand_off_distance / np.linalg.norm(improvement)
      improvement_field = convert_local(improvement, -pangle) + self.robot.loc
      #print(improvement_field, improvement, convert_local(improvement, -pangle), worst_loc, importance * self.weights[pind], pangle, p, self.robot.loc)
      imrpovements = imrpovements + improvement_field * importance * self.weights[pind]
      improv_weight += importance * self.weights[pind]
      pind += 1
    imrpovements = imrpovements/improv_weight
    move_to = imrpovements
    
    
    point_dir = self.points[0] - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions
    
    

if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  for i in game.yellow_robots[1:]:
    blockers = [b.loc for b in game.blue_robots]
    bsimple = [b.loc for b in game.blue_robots]
    for y in game.yellow_robots[1:]:
      if y.id != i.id:
        blockers.append(y.loc)
        #print(y.id, i.id)
    i.add_action(get_open([game.yellow_robots[0].loc, [-5100,0]], [3,1], blockers))
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    new_time = clock.tick()
    game.step()
    ttime = new_time