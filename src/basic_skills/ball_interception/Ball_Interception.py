import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.orbit_ball.orbit_ball import *
from basic_skills.helper_functions import *
from basic_skills.ball_interception.Catch_Pygym import *

class intercept_ball(action):
  def __init__(self):
    action.__init__(self)
    self.pid = move_to()
    self.iterations = 3
    self.robot = False
    self.target_loc = False
    self.robot_actual_speed = 600
    self.robot_radius = 90
    self.ball_radius = 25
    self.orbit = orbit_ball()
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    
    self.orbit.add(robot, game)
    action.add(self, robot, game)
  def run(self):
    ball = self.game.ball
    ball_speed = np.linalg.norm(ball.velocity)
    robot = self.robot
    if ball_speed < 20:
      target_loc = ball.loc
    else:
      target_loc = drop_perpendicular(self.robot.loc, ball.loc, ball.velocity)
      
      ball_loc = ball.loc + ball.velocity
      off_by = np.linalg.norm(ball_loc - target_loc)
      old_distance = np.linalg.norm(ball.loc - target_loc)
      if off_by > old_distance:
        self.orbit.target_loc = ball.loc - ball.velocity
        return(self.orbit.run())
      #iteratively refine
      for i in range(self.iterations):
        travel_time = time_to_point(robot, target_loc, self.robot_actual_speed)
        ball_loc = ball.loc + ball.velocity * travel_time
        off_by = np.linalg.norm(ball_loc - target_loc)
        ball_vel_normalized = ball.velocity/ball_speed
        #if ball passed target_loc
        if (off_by + travel_time*ball_speed > old_distance):
          if self.robot_actual_speed - ball_speed > 0:
            shortening_rate = self.robot_actual_speed - ball_speed
            target_loc = target_loc + off_by/shortening_rate*ball_vel_normalized
            #print("running back", target_loc, ball_loc)
          else:
            #print("running down", ball_speed, ball_loc, ball.loc)
            target_loc = ball_loc
        else:
          #if ball hasn't reached target
          shortening_rate = self.robot_actual_speed + ball_speed
          target_loc = target_loc - off_by/shortening_rate*ball_vel_normalized*self.robot_actual_speed
          #print("running up", target_loc, off_by, shortening_rate)
    #move up to the ball not ontop of the ball
    hold_offset = self.robot.loc - ball.loc
    if np.linalg.norm(hold_offset) > .1:
      hold_offset = hold_offset / np.linalg.norm(hold_offset)
      #print(self.robot_radius + self.ball_radius - ball_speed/100, self.game.ball.controler)
      offset = self.robot_radius + self.ball_radius
      if not self.game.ball.controler:
        offset -= ball_speed/5
      hold_offset = hold_offset * offset
      target_loc = target_loc + hold_offset
    point_dir = robot.loc - ball.loc
    target_rot = -normalize_angle(math.pi + math.atan2(point_dir[1], point_dir[0]))
    #print(target_rot, math.atan2(point_dir[0], point_dir[1]), point_dir)
    self.pid.set_target(target_loc, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.target_loc = target_loc
    return actions


if __name__ == "__main__":
  max_bots_per_team = 6
  game = Ball_Intercept_PYGym(max_bots_per_team)
  intercept_action = intercept_ball()
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  game.add_action(intercept_action, 0, False)
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    new_time = clock.tick()
    game.step()
    ttime = new_time


