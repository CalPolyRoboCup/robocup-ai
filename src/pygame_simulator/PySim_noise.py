import os
import sys
import time
import numpy as np
import math
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
from basic_skills.source.robot import robot
from basic_skills.source.action import action
from basic_skills.source.helper_functions import mag, convert_local, min_angle, drop_perpendicular, scale_to, mag_with, angle_of
from basic_skills.source.ball import ball

'''
GRsimGame environment documentation

inputs:
kickspeedx 0
kickspeedz 1
veltangent 2
velnormal    3
velangular 4

output:
ball                     x, y, observed
allie_robots        x, y, orientation
enemy_robots        x, y, orientation
'''
        
import pygame

default_formation = [[-1000,0],[-2000, 500], [-2000, -500], [-3000, 1000], [-3000, -1000], [-4500, 0]]
        
class PYsim:

    # constant multiplication for x,y velocity actions to match GRsim
    action_scaling_constant = 15.3846
    
    def __init__(self, max_bots_per_team, starting_formation = default_formation):
        pygame.font.init() # you have to call this at the start, 
                                     # if you want to use this module.
        self.font = pygame.font.SysFont('Comic Sans MS', 20)
    
    
        # robot and ball radii
        self.robot_radius = 90
        self.ball_radius = 25
    
        self.max_bots_per_team = max_bots_per_team
        self.blue_robots = [robot(True, i, self) for i in range(self.max_bots_per_team)]
        self.yellow_robots = [robot(False, i, self) for i in range(self.max_bots_per_team)]
        
        self.blue_robots_internal = [robot(True, i, self) for i in range(self.max_bots_per_team)]
        self.yellow_robots_internal = [robot(False, i, self) for i in range(self.max_bots_per_team)]

        pygame.display.set_caption('Pysim')

        self.clock = pygame.time.Clock()
        self.last_tick = pygame.time.get_ticks()
        self.screen_res = np.array([1040, 740])
        self.field_upper_left = np.array([-6000, -4500])
        self.field_dims = np.array([12000,9000])
        self.goal_height = 1500

        self.screen = pygame.display.set_mode(self.screen_res, pygame.HWSURFACE, 32)
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "../resources/Field.png")
        self.field_image = pygame.image.load(filename).convert_alpha()
        field_scale = self.field_image.get_rect().size * self.screen_res / np.array([1040, 740])
        field_scale = (int(field_scale[0]), int(field_scale[1]))
        self.field_image = pygame.transform.scale(self.field_image, field_scale)
        
        filename = os.path.join(dirname, "../resources/BlueBot.png")
        self.blue_robot_image = pygame.image.load(filename).convert_alpha()
        robot_scale = 2*self.robot_radius*self.screen_res/self.field_dims
        robot_scale = (int(robot_scale[0]), int(robot_scale[1]))
        self.blue_robot_image = pygame.transform.scale(self.blue_robot_image, robot_scale)
        
        filename = os.path.join(dirname, "../resources/YellowBot.png")
        self.yellow_robot_image = pygame.image.load(filename).convert_alpha()
        self.yellow_robot_image = pygame.transform.scale(self.yellow_robot_image, robot_scale)
        
        self.time_step = 1/60
        self.starting_formation = starting_formation
        
        self.ball = ball()
        self.ball_internal = ball()
        
        '''
        physics parameters
        '''

        # kicker refactory period
        self.kick_cd = 180
        
        # hit box for kick
        self.kick_length = 40
        self.kick_width = 70
        
        # kick force
        self.kick_delta_V = 2500
        
        # hit box for spinner
        self.spin_length = 40
        self.spin_width = 100
        
        # spin force when touching
        self.spin_lin_accel = 15
        
        # stored spin force after contact broken (leaving it equal to spin_lin_accel works well)
        self.spin_rot_accel = 15
        
        # how much acceleration can be stored in spin
        self.max_ball_spin = 4
        
        # how quickly the ball spins up (0 to 1 inclusive) larger is slower
        self.spin_accel_rate = .85
        
        # how fast the current robot velocity changes to the target velocity (0 to 1 inclusive) larger is slower
        self.accel_rate = .99

        self.rot_accel_rate = 0.97
        
        # how fast the robot can move
        self.max_speed = 800
        
        # how fast the robot can turn (radians)
        self.max_angular_speed = 2
        
        # mass of the ball affects collision push out speed
        self.ball_mass = 1
        
        # 1 perfectly elastic (bounces)
        # 0 perfectly inelastic (sticks)
        self.elasticity_factor = .1
        
        # how quickly the ball looses spin
        self.spin_degen = .75
        
        # how quickly the ball slows down when not controlled (larger slows down less)
        self.ball_friction_factor = .999
        
        #the anglage of the robot that is flat (not curved)
        self.robot_front_angle = 10/180*math.pi

        # noise parameterization
        self.translation_noise = 0
        self.rotation_noise = 0
        self.vanish_prob = 0
        
        self.throw_in_max_speed = 500

        self.blue_goal_loc = np.array([-6500,0])
        self.yellow_goal_loc = np.array([6500,0])

        self.reset()
        
    '''
    brief: place all robots in starting fomation and the ball in the center
    '''
    def reset(self, toss_in = True):
        for i in range(self.max_bots_per_team):
            self.blue_robots_internal[i].loc = np.array(self.starting_formation[i])
            self.blue_robots_internal[i].rot = 0
            self.yellow_robots_internal[i].loc = - np.array(self.starting_formation[i])
            self.yellow_robots_internal[i].rot = -np.pi
            self.blue_robots_internal[i].velocity = np.array([0,0])
            self.yellow_robots_internal[i].velocity = np.array([0,0])
        self.ball_internal.loc = np.array([0,0])
        if toss_in:
            self.ball_internal.velocity = np.random.uniform(-self.throw_in_max_speed, self.throw_in_max_speed,[2])
        else:
            self.ball_internal.velocity = np.array([0,0])
        
    '''
    brief - convert a screen pos to a field location
    param - loc: a screen position
    return - a field location
    '''
    def convert_to_field_position(self, loc):
        return self.field_upper_left + loc * self.field_dims / self.screen_res
        
    '''
    brief - convert a screen pos to a field location
    param - loc: a screen position
    return - a field location
    '''
    def convert_to_screen_position(self, loc, dims = [0,0]):
        return (loc - self.field_upper_left)*self.screen_res/self.field_dims - np.array(dims)/2
        
    '''
    brief: render the game
    params: key_points - points to plot for debugging. A list of values. Values can be a [x,y] point or a tuple of ([x,y], size).
                                                if size is negative the color is orange otherwise it is blue. Either way the point has radius |size|
    '''
    def draw(self, key_points = []):
        self.render_game()
        self.render_key_points(key_points)
        pygame.display.update()
        
    '''
    render the game
    '''
    def render_game(self):
        self.screen.blit(self.field_image,(0,0))
        for br in self.blue_robots_internal:
            rot_image = pygame.transform.rotate(self.blue_robot_image , math.degrees(br.rot))
            position = self.convert_to_screen_position(br.loc, rot_image.get_rect().size)
            num = self.font.render(str(br.id), False, (0,0,0))
            self.screen.blit(num, position)
            self.screen.blit(rot_image, position)
        for yr in self.yellow_robots_internal:
            rot_image = pygame.transform.rotate(self.yellow_robot_image , math.degrees(yr.rot))
            position = self.convert_to_screen_position(yr.loc, rot_image.get_rect().size)
            num = self.font.render(str(yr.id), False, (0,0,0))
            self.screen.blit(num, position)
            self.screen.blit(rot_image, position)
        position = self.convert_to_screen_position(self.ball_internal.loc)
        pygame.draw.circle(self.screen, (255,55,0), (int(position[0]), int(position[1])), int(self.ball_radius*self.screen_res[0]/self.field_dims[0]))
        
    '''
    render the key points
    '''
    def render_key_points(self, key_points):
        for kp in key_points:
            if type(kp) is tuple:
                plot_point = self.convert_to_screen_position(np.array(kp[0]))
                scale = int(kp[1])
                if scale < 0:
                    pygame.draw.circle(self.screen, (155,155,0), (int(plot_point[0]), int(plot_point[1])), -scale)
                else:
                    pygame.draw.circle(self.screen, (0,155,155), (int(plot_point[0]), int(plot_point[1])), scale)
            else:
                kp = self.convert_to_screen_position(np.array(kp))
                pygame.draw.circle(self.screen, (155,155,0), (int(kp[0]), int(kp[1])), int(self.ball_radius*self.screen_res[0]/self.field_dims[0])*4)

    '''
    spinner stuff
    '''
    def update_bot_spinner(self, robot, delta_time):
        # if the ball is in a box in front of the robot apply spin
        ball_robot_vector = self.ball_internal.loc - robot.loc
        robot_local_ball_loc = convert_local(ball_robot_vector, -robot.rot)
        if (robot_local_ball_loc[0] > 0 and robot_local_ball_loc[0] < self.robot_radius + self.spin_length + self.ball_radius
            and abs(robot_local_ball_loc[1]) < self.spin_width/2):
            
            self.ball_internal.controler = robot
            pull_vel = robot.loc + convert_local([self.robot_radius + self.ball_radius, 0], robot.rot) - self.ball_internal.loc
            pull_vel = pull_vel / mag(pull_vel) * self.max_ball_spin
            self.ball_internal.spin = self.ball_internal.spin * self.spin_accel_rate + pull_vel * (1-self.spin_accel_rate)
            self.ball_internal.velocity = self.ball_internal.velocity * self.spin_accel_rate + pull_vel * (1-self.spin_accel_rate)

    '''
    brief: updates a robot's kicker
    params: robot - robot to update 
                    action - the action taken by the robot
                    kick - True if this bot should kick the ball
                    chip - Unused. Would indicate whether to chip the ball up and over
    '''
    def update_bot_kicker(self, robot, kick, chip):
        
        # if kicker off cool down and the ball is in the kicking box in front of the robot
        #     kick the ball
        ball_robot_vector = self.ball_internal.loc - robot.loc
        robot_local_ball_loc = convert_local(ball_robot_vector, -robot.rot)
        if robot.kick_cooldown > 0:
            robot.kick_cooldown -= 1
        if kick and robot.kick_cooldown == 0:
            robot.kick_cooldown = self.kick_cd
            if (robot_local_ball_loc[0] > 0 and robot_local_ball_loc[0] < self.robot_radius + self.kick_length + self.ball_radius
                    and abs(robot_local_ball_loc[1]) < self.kick_width/2):
                #print("kick worked")
                self.ball_internal.velocity = self.ball_internal.velocity + self.kick_delta_V * np.array([np.cos(-robot.rot), np.sin(-robot.rot)])
        
    '''
    brief: updates a robot's movement
    params: robot - robot to update 
                    action - the action taken by the robot
                    norm_vel - the target velocity in the normal direction (Forward Backwards)
                    tang_vel - the target velocity in the tangential direction (Side to Side)
                    rot_vel - the target rotational velocity
                    delta_time - time since last update
    '''
    def update_bot_movement(self, robot, norm_vel, tang_vel, rot_vel, delta_time):
        
        robot.velocity = robot.velocity*self.accel_rate + convert_local(np.array([norm_vel, tang_vel])*PYsim.action_scaling_constant, robot.rot) * (1-self.accel_rate)
        robot.rot_vel = robot.rot_vel*self.rot_accel_rate + rot_vel * (1-self.rot_accel_rate)
        
        #limit max speed
        if mag(robot.velocity) > self.max_speed:
            robot.velocity = robot.velocity * self.max_speed/mag(robot.velocity)
        if robot.rot_vel > self.max_angular_speed:
            robot.rot_vel = self.max_angular_speed
        if robot.rot_vel < -self.max_angular_speed:
            robot.rot_vel = -self.max_angular_speed
        
        robot.loc = robot.loc + robot.velocity * delta_time
        robot.rot = min_angle(robot.rot + robot.rot_vel * delta_time)
        
    '''
    brief - updates robot physics for a delta_time time step
    params - robot: the robot to update
                    delta_time: the time between this step and the previous update
    '''
    def update_bot(self, robot, delta_time):
        self.update_bot_spinner(robot, delta_time)
        action = robot.run_action()
        if action == None:
            kick, chip, norm_vel, tang_vel, rot_vel = (0,0,0,0,0)
        else:
            kick, chip, norm_vel, tang_vel, rot_vel = action
        self.update_bot_kicker(robot, kick, chip)
        self.update_bot_movement(robot, norm_vel, tang_vel, rot_vel, delta_time)
        
    '''
    brief: handle collisions with balls
    params: robot - the robot to do collision for
                    ball_held - whether the ball has already been pushed out of a robot
    return: ball_held - an updated version of ball_held
    '''
    def do_ball_collision(self, robot, ball_held, delta_time):
        ball_vector = self.ball_internal.loc - robot.loc
        ball_distance = mag(ball_vector)
        
        #if the ball overlaps with the robot
        if ball_distance < self.robot_radius + self.ball_radius:
            push_out_vector = ball_vector * ((self.robot_radius + self.ball_radius) - ball_distance)/ball_distance
            bounce_velocity = drop_perpendicular(self.ball_internal.velocity, np.array([0,0]), ball_vector) * self.elasticity_factor
            
            '''
            if no other robot has touched the ball move only the ball.
            if another robot has touched the ball only move the robot.
            '''
            if ball_held:
                robot.loc = robot.loc - push_out_vector
                self.ball_internal.velocity = self.ball_internal.velocity + push_out_vector / delta_time / self.ball_mass + bounce_velocity
            else:
                ball_held = True
                if abs(angle_of(push_out_vector) - angle_of(robot.facing)) < self.robot_front_angle:
                    push_out_vector = scale_to(robot.facing, mag_with(push_out_vector, robot.facing))
                self.ball_internal.velocity = self.ball_internal.velocity + push_out_vector / delta_time / self.ball_mass + bounce_velocity
                self.ball_internal.loc = self.ball_internal.loc + push_out_vector
        return ball_held
        
    '''
    brief: handle collisions with other robots
    params: r - robot to run collision on
                    o - other robot to run collision on
                    delta_time - time step
    '''
    def do_robot_collision(self, r, o, delta_time):
        distance = mag(r.loc - o.loc)
        if distance < 2 * self.robot_radius:
            push_out_vector = (r.loc - o.loc) * (distance - 2*self.robot_radius)/(distance * 2)
            r.loc = r.loc - push_out_vector
            r.velocity = r.velocity - push_out_vector / delta_time
            o.loc = o.loc + push_out_vector
            o.velocity = o.velocity + push_out_vector / delta_time
        
    '''
    brief: do all collisions
    params: delta_time - time since last update
    '''
    def do_collision(self, delta_time):
        robots = []
        robots.extend(self.blue_robots_internal)
        robots.extend(self.yellow_robots_internal)
        ball_held = False
        
        ind = 0
        for r in robots:
            ind += 1
            for o in robots[ind:]:
                self.do_robot_collision(r, o, delta_time)
            ball_held = self.do_ball_collision(r, ball_held, delta_time)
           
    '''
    brief: checks for out of bounds and goals
    returns - reward: the reward for the blue team in this state
                        done: True if a goal is made or a foul is committed
    ''' 
    def get_reward(self):
        reward = 0
        
        if self.ball_internal.controler != False:
            self.ball_internal.last_controler = self.ball_internal.controler
        if self.ball_internal.last_controler != False:
            if (self.ball_internal.loc[0] > (self.field_dims + self.field_upper_left)[0] or self.ball_internal.loc[1] > (self.field_dims + self.field_upper_left)[1] or
                self.ball_internal.loc[0] < self.field_upper_left[0] or self.ball_internal.loc[1] < self.field_upper_left[1]):
                if (abs(self.ball_internal.loc[1]) < self.goal_height/2):
                    if self.ball_internal.last_controler.is_blue:
                        # goal for yellow
                        print("goal for blue")
                        reward = -50
                    else:
                        # goal for blue
                        print("goal for yellow")
                        reward = 50
                elif self.ball_internal.last_controler.is_blue:
                    # out of bounds on blue
                    print("out on blue")
                    reward = -5
                else:
                    # out of bounds on yellow
                    print("out on yellow")
                    reward = 5
                return reward, True
        return reward + self.ball_internal.loc[0]/1000, False
        
    '''
    brief: if the ball is free its spin will adjust its velocity slightly
    params: delta_time - time since last update
    '''
    def ball_update(self, delta_time):
        if not self.ball_internal.controler:
            self.ball_internal.velocity = self.ball_internal.velocity + self.ball_internal.spin
            self.ball_internal.spin = self.ball_internal.spin * self.spin_degen
        self.ball_internal.loc = self.ball_internal.loc + self.ball_internal.velocity * delta_time
        self.ball_internal.velocity = self.ball_internal.velocity * self.ball_friction_factor
        
    '''
    brief: Advance the state of the game
    params: delta_time - time since last tick. Defaults to 1/60
                    visualize - Whether the game should be drawn
                    key_points - points to display on field. Either in the form [(x,y)] or [((x,y), size]. 
                                                If size is negative the points are drawn in a different color. Ignored if visualize == False
    returns: state - (Bstate, Ystate) Bstate and Ystate are the state vectors of game for blue and yellow teams respectively
                                        . Useful for ML down the road
                      blue_reward - reward for the blue team. Invert for Yellow team.
                      transition - Whether a state change has occurred. True when goal scored, or foul committed.
                      
    '''
    def step(self, delta_time = .01666666, visualize = True, key_points = []):
        state = self.get_state()
        self.ball_internal.controler = False
        
        for blue_robot in self.blue_robots_internal:
            self.update_bot(blue_robot, delta_time)
        for yellow_robot in self.yellow_robots_internal:
            self.update_bot(yellow_robot, delta_time)
        
        self.do_collision(delta_time)
        
        self.ball_update(delta_time)
        
        self.draw(key_points)
        blue_reward, transition = self.get_reward()
        if transition:
            self.reset()
        return state, blue_reward, transition
        
    '''
    brief: Needed to match GRsim API
    '''
    def push_state(self):
        pass
        
    '''
    brief: use this method to add actions to robots so that internal and external objects are kept up to date
    params: action - action to add
                    index - the index of the bot to add action to
                    is_blue - whether the bot to add action to is on the blue team
    '''
    def add_action(self, action, index, is_blue):
        if is_blue:
            self.blue_robots_internal[index].add_action(action)
            self.blue_robots[index].add_action(action)
        else:
            self.yellow_robots_internal[index].add_action(action)
            self.yellow_robots[index].add_action(action)
          
    '''
    brief: inject noise into ball_internal, and robot_internal states and update external objects
    returns: state_blue - state vector for blue team
                      state_yellow - state vector for yellow team
    '''  
    def get_state(self):
        
        state_blue = []
        state_yellow = []
        
        # copy states between yellow and blue state vectors
        # ball info [0:4] is the same between both
        # blue robot info is swapped with yellow robot info between the two states
        # we assign the list slices to each other to copy pointers so that only yellow state must be filled 
        # This allows for data reuse.
        state_blue[0:4] = state_yellow[0:4]
        state_blue[4:10+6*self.max_bots_per_team] = state_yellow[10+6*self.max_bots_per_team:]
        state_blue[10+6*self.max_bots_per_team:10+12*self.max_bots_per_team] = state_yellow[4:10+6*self.max_bots_per_team]
        
        # call updates on balls and robots and inject noise to simulate hardware
        self.ball.update(self.ball_internal.loc + np.random.normal(size = [2]) * self.translation_noise, np.random.randint(2))
        i = 0
        for BRobot in self.blue_robots:
            BRobot.update(self.blue_robots_internal[i].loc + np.random.normal(size = [2]) * self.translation_noise, 
                self.blue_robots_internal[i].rot + np.random.normal() * self.rotation_noise,
                np.random.choice([0,1], p = [self.vanish_prob, 1 - self.vanish_prob]))
            i += 1
        
        i = 0
        for YRobot in self.yellow_robots:
            YRobot.update(self.yellow_robots_internal[i].loc + np.random.normal(size = [2]) * self.translation_noise, 
                self.yellow_robots_internal[i].rot + np.random.normal() * self.rotation_noise, 
                np.random.choice([0,1], p = [self.vanish_prob, 1 - self.vanish_prob]))
            i += 1
        self.ball.controler = self.ball_internal.controler
        
        # fill the yellow state vector with info. This data is copied to blue state vector as explained above
        state_yellow.append(self.ball.loc[0])
        state_yellow.append(self.ball.loc[1])
        state_yellow.append(self.ball.velocity[0])
        state_yellow.append(self.ball.velocity[1])
        for YRobot in self.yellow_robots:
            state_yellow.append(YRobot.loc[0])
            state_yellow.append(YRobot.loc[1])
            state_yellow.append(YRobot.velocity[0])
            state_yellow.append(YRobot.velocity[1])
            state_yellow.append(YRobot.rot_vel)
            state_yellow.append(YRobot.rot)
        for BRobot in self.blue_robots:
            state_yellow.append(BRobot.loc[0])
            state_yellow.append(BRobot.loc[1])
            state_yellow.append(BRobot.velocity[0])
            state_yellow.append(BRobot.velocity[1])
            state_yellow.append(BRobot.rot_vel)
            state_yellow.append(BRobot.rot)
        
        return state_blue, state_yellow
        
    '''
    brief: runs game test environment. NOTE: this is not final
    params: blue_strategy - a strategy to choose actions for blue robots. Must have an update method
                    yellow_strategy - a strategy to choose actions for yellow robots. Must have an update method
                    framerate - the frame rate to run the game at. If None the game is not visualized and is run as fast as possible.
    '''
    def run(self, blue_strategy = None, yellow_strategy = None, framerate = 60, key_points = []):
        if framerate != None:
            clock = pygame.time.Clock()
            clock.tick(framerate)
            
        while(1):
            if blue_strategy != None:
                blue_strategy.update()
            if yellow_strategy != None:
                yellow_strategy.update()
                
                
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                        
                if event.type ==  pygame.KEYDOWN or event.type ==  pygame.KEYUP:
                    keys = pygame.key.get_pressed()
                    #press r-key to reset
                    if keys[pygame.K_r]:
                        game.reset()
            if framerate != None:
                new_time = clock.tick()
                game.step(new_time - ttime, key_points = key_points)
                ttime = new_time
            else:
                game.step(visualize = False)
                