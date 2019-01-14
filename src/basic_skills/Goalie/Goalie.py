import numpy as np
import sys
sys.path.insert(0, '../..')
from basic_skills.action import *
from basic_skills.move_to.move_to import move_to
from pygame_simulator.PySim_noise import *

class goalie(action):
  # Covers a pass from target_ball to target_loc
  # for goalie
  # identical to cover with "lead_target_robot" False
  def __init__(self):
    action.__init__(self)
    self.goal = np.array([0,0])
    self.pid = move_to()
    self.orbit = 1500
  def add(self, robot, game):
    action.add(self, robot, game)
    move_to.add(self.pid, robot, game)
    if self.robot.is_blue:
      self.goal = np.array([-6500,0])
    else:
      self.goal = np.array([6500,0])
  def run(self):
    adjustment = 2500000/(100 + np.linalg.norm(self.game.ball.loc - self.goal))
    
    #recoil to buy time and absorb shock if the ball is free
    # if self.game.ball.controler == False:
      # adjustment = -adjustment
      
    push_out_distance = self.orbit + adjustment
    push_vec = self.game.ball.loc - self.goal
    
    #print(adjustment, -np.linalg.norm(push_vec))
    if push_out_distance > np.linalg.norm(push_vec):
      push_out_distance = np.linalg.norm(push_vec) - 40
    move_to = self.goal + push_vec * push_out_distance / np.linalg.norm(push_vec)
    point_dir = self.game.ball.loc - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    
    #bypass movement restrictions and set target location directly
    self.pid.target_loc = move_to
    self.pid.target_rot = target_rot
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions
    
    
if __name__ == "__main__":
  game = PYsim(6)
  goalie_action = goalie()
  game.add_action(goalie_action, 0, True)
  i = 0
  
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
        
      
      if event.type == MOUSEBUTTONDOWN:
        pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
        if pressed1:
          game.ball_internal.loc = game.convert_to_field_position(pygame.mouse.get_pos())
          game.ball_internal.velocity = np.array([0,0])
        if pressed3:
          game.ball_internal.velocity = (game.convert_to_field_position(pygame.mouse.get_pos()) - game.ball_internal.loc)
    new_time = clock.tick()
    game.step()
    ttime = new_time