import sys
import math
#replace this with your path to robocup-ai
sys.path.insert(0,'../../../src')
from basic_skills.action import *
from basic_skills.helper_functions import *
from basic_skills.move_to.move_to import *
from basic_skills.helper_functions import *
from pygame_simulator.PySim import *
from basic_skills.ball_interception.Ball_Interception import *

class dribble_ball(action):
    def __init__(self, target_loc = False, target_rot = False):
        action.__init__(self)
        self.pid = move_to()
        self.intercept = intercept_ball()

        self.iterations = 3
        self.robot = False
        self.target_loc = False
        self.robot_actual_speed = 600
        self.robot_radius = 90
        self.ball_radius = 25
        self.ball_control_radius = 26

        self.target_loc = target_loc
        self.target_rot = False

        self.control_ball = False

        self.min_contact = .1 #125
        self.radius = 310

        self.mu = 1

        #PID
        self.I = 0
        self.D_constant = 0
        self.I_constant = 0
        self.P_constant = 0.85

        self.go_intercept = True

    def set_target(self, target_loc):
        if target_loc[0] < -5500:
            target_loc[0] = -5500
        if target_loc[0] > 5500:
            target_loc[0] = 5500
        if target_loc[1] < -4000:
            target_loc[1] = -4000
        if target_loc[1] > 4000:
            target_loc[1] = 4000
        self.target_loc = target_loc

    def add(self, robot, game):
        self.robot = robot
        self.pid.robot = robot
        self.intercept.add(robot, game)
        action.add(self, robot, game)

    def run(self):
        ball = self.game.ball
        robot = self.robot

        #ensures that the robot is carrier of ball 
        #TODO: subject to change when implementation of game is made

        # if robot is self.game.ball.controler:
        #     self.go_intercept = False
        # if robot is not self.game.ball.controler and self.go_intercept: #np.linalg.norm(robot.loc - ball.loc) > self.min_contact and 
        #     return(self.intercept.run())
        # else:
        #     threshold = 0.05
        #     delta = self.target_loc - robot.loc
        #     target_angle = math.atan2(delta[1], delta[0]) * -1
        #     if target_angle < 0:
        #         target_angle = 2*math.pi + target_angle
        #     delta_angle = target_angle - self.robot.rot
            
        #     increment = self.P_constant * math.atan(math.pow(np.linalg.norm(robot.velocity), 2) / (self.radius * 10 * self.mu))
        #     if delta_angle > threshold:
        #         #print("greater than threshold")
        #         new_theta = robot.rot + increment
        #         curve_x = self.robot.loc[0] + self.radius * math.sin(new_theta)
        #         curve_y = self.robot.loc[1] + self.radius * math.cos(new_theta)
        #         self.pid.set_target(np.array([curve_x, curve_y]), new_theta)
        #     elif delta_angle < -threshold:
        #         #print("less than threshold")
        #         new_theta = robot.rot + increment
        #         curve_x = self.robot.loc[0] - self.radius * math.sin(new_theta)
        #         curve_y = self.robot.loc[1] - self.radius * math.cos(new_theta)
        #         self.pid.set_target(np.array([curve_x, curve_y]), new_theta)
        #     else:
        #         self.pid.set_target(self.target_loc, robot.rot)

        threshold = 0.05
        delta = self.target_loc - robot.loc
        target_angle = math.atan2(delta[1], delta[0]) * -1
        if target_angle < 0:
            target_angle = 2*math.pi + target_angle
        delta_angle = target_angle - self.robot.rot
        
        increment = math.atan(math.pow(np.linalg.norm(robot.velocity), 2) / (self.radius * 10 * self.mu))
        if delta_angle > threshold:
            #print("greater than threshold")
            new_theta = robot.rot + increment
            curve_x = self.robot.loc[0] + self.radius * math.sin(new_theta)
            curve_y = self.robot.loc[1] + self.radius * math.cos(new_theta)
            self.pid.set_target(np.array([curve_x, curve_y]), new_theta)
        elif delta_angle < -threshold:
            #print("less than threshold")
            new_theta = robot.rot - increment
            curve_x = self.robot.loc[0] - self.radius * math.sin(new_theta)
            curve_y = self.robot.loc[1] - self.radius * math.cos(new_theta)
            self.pid.set_target(np.array([curve_x, curve_y]), new_theta)
        else:
            self.pid.set_target(self.target_loc, robot.rot) 

        # if delta_angle > threshold:
        #     angle = (robot.rot - self.P_constant + self.I *
        #              self.I_constant - self.D_constant * delta_angle) % (2*math.pi)
        #     print("pos target angle: {} current angle: {}".format(
        #         target_angle, angle))
        #     # curve_x = self.robot.loc[0] + self.radius * math.cos(angle)
        #     # curve_y = self.robot.loc[1] + self.radius * math.sin(angle)

        #     # self.pid.set_target(np.array([curve_x, curve_y]), angle)
        #     self.pid.set_target(robot.loc, angle)
        # elif delta_angle < -threshold:
        #     angle = (robot.rot + self.P_constant + self.I *
        #                  self.I_constant - self.D_constant * delta_angle) % (2*math.pi)
        #     print("pos target angle: {} current angle: {}".format(
        #         target_angle, angle))
        #     # curve_x = self.robot.loc[0] + self.radius * math.cos(angle)
        #     # curve_y = self.robot.loc[1] + self.radius * math.sin(angle)

        #     # self.pid.set_target(np.array([curve_x, curve_y]), angle)
        #     self.pid.set_target(robot.loc, angle)
        # else:
        #     self.I = 0
        #     self.pid.set_target(self.target_loc, robot.rot)

        actions = self.pid.run()
        self.actions = actions
        return actions

    def reset(self):
        self.go_intercept = True

    def get_target(self):
        return self.target_loc

if __name__ == "__main__": 
    max_bots_per_team = 6
    game = PYsim(max_bots_per_team)
    clock = pygame.time.Clock()
    clock.tick(60)
    ttime = clock.tick()
    move_action = move_to()
    dribble_action = dribble_ball()
    intercept_action = intercept_ball()
    move_action.set_target(np.array([0,0]), 0)

    game.add_action(dribble_action, 0, True)
    game.add_action(move_action, 1, True)
    j = 0
    while 1:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN or event.type == KEYUP:
                keys = pygame.key.get_pressed()
                key_action.keypress_update(keys)
        new_time = clock.tick()
        if j % 750 == 0:
            dribble_action.reset()
            print("Change")
            random_location = np.random.uniform(-1, 1, size = [2])*np.array([3000, 2500])
            dribble_action.set_target(random_location)
            move_action.set_target(dribble_action.get_target(), 0)
        game.step()
        j += 1
        ttime = new_time
