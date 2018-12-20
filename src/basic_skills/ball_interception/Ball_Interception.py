import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.helper_functions import *
from matplotlib.widgets import Slider, Button, RadioButtons

class intercept_ball(action):
  def __init__(self):
    action.__init__(self)
    self.pid = move_to()
    self.iterations = 3
    self.robot = False
    self.target_loc = False
    self.robot_actual_speed = 400
    self.robot_radius = 90
    self.ball_radius = 25
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
  def run(self):
    ball = self.game.ball
    ball_speed = np.linalg.norm(ball.velocity)
    robot = self.robot
    if ball_speed < 20:
      target_loc = ball.loc
    else:
      target_loc = drop_perpendicular(self.robot.loc, ball.loc, ball.velocity)

      #iteratively refine
      for i in range(self.iterations):
        travel_time = time_to_point(robot, target_loc, self.robot_actual_speed)
        ball_loc = ball.loc + ball.velocity * travel_time
        off_by = np.linalg.norm(ball_loc - target_loc)
        ball_vel_normalized = ball.velocity/ball_speed
        #if ball passed target_loc
        if (off_by + travel_time*ball_speed > np.linalg.norm(ball.loc - target_loc)):
          if self.robot_actual_speed - ball_speed > 0:
            shortening_rate = self.robot_actual_speed - ball_speed
            target_loc = target_loc + off_by/shortening_rate*ball_vel_normalized
            #print("running back", target_loc, off_by, shortening_rate)
          else:
            #print("running down", ball_speed, self.robot_actual_speed)
            old_target_loc = target_loc
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
      hold_offset = hold_offset * (self.robot_radius + self.ball_radius)
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
  intercept = intercept_ball()
  game = GRsim(1)
  game.blue_robots[0].add_action(intercept)
  loc = np.array([0,0])
  intercept.robot.loc = loc
  rot = 0
  intercept.robot.rot = rot
  intercept.target_loc = np.array([0,0])
  plt.figure(2)
  rlxax = plt.axes([0.25, 0.1, 0.65, 0.03])
  rlocx = Slider(rlxax, "robot locx", -2000, 2000, valinit=loc[0])

  rlyax = plt.axes([0.25, 0.15, 0.65, 0.03])
  rlocy = Slider(rlyax, "robot locy", -2000, 2000.0, valinit=loc[0])

  rvxax = plt.axes([0.25, 0.2, 0.65, 0.03])
  rvelx = Slider(rvxax, "robot velx", -2000, 2000, valinit=0)

  rvyax = plt.axes([0.25, .25, 0.65, 0.03])
  rvely = Slider(rvyax, "robot vely", -2000, 2000, valinit=0)

  rrax = plt.axes([0.25, 0.3, 0.65, 0.03])
  rrot = Slider(rrax, "robot rot", -4, 4, valinit=loc[0])

  blxax = plt.axes([0.25, 0.35, 0.65, 0.03])
  blocx = Slider(blxax, "ball locx", -2000, 2000.0, valinit=loc[0])

  blyax = plt.axes([0.25, 0.4, 0.65, 0.03])
  blocy = Slider(blyax, "ball locy", -2000, 2000.0, valinit=loc[0])

  bvxax = plt.axes([0.25, 0.45, 0.65, 0.03])
  bvelx = Slider(bvxax, "ball velx", -2000, 2000, valinit=0)

  bvyax = plt.axes([0.25, 0.5, 0.65, 0.03])
  bvely = Slider(bvyax, "ball vely", -2000, 2000, valinit=0)

  def update(event):
    plt.figure(1)
    intercept.robot.loc[0] = rlocx.val
    intercept.robot.loc[1] = rlocy.val
    intercept.robot.velocity[0] = rvelx.val
    intercept.robot.velocity[1] = rvely.val
    intercept.robot.rot = rrot.val
    intercept.game.ball.loc[0] = blocx.val
    intercept.game.ball.loc[1] = blocy.val
    intercept.game.ball.velocity[0] = bvelx.val
    intercept.game.ball.velocity[1] = bvely.val
    intercept.run()
    game.display([intercept.target_loc])
    event.canvas.draw()
    plt.clf()
  plt.gcf().canvas.mpl_connect('button_press_event', update)
  game.display([intercept.target_loc])
  plt.show()


