import os
import sys
import time
dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname+'/..')
from basic_skills.source.robot import robot
from basic_skills.source.action import action
from PySim_noise import PYsim
import pygame


#test action
class keyboard_control(action):
    '''
    sets a robot to be controlled by WAD keys (tangential and normal velocities)
    QE keys control rotational velocity
    SPACE bar kicks
    '''
    def __init__(self):
        action.__init__(self)
        self.norm_vel = 0
        self.tang_vel = 0
        self.rot_vel = 0
        self.kick = 0
        self.speed = 65
        self.rot_speed = 4
    def keypress_update(self, keys):
        '''
        brief: Takes updates on key states and stores values for use in run action
        params: keys - KEYPRESS dictionary as from         pygame.key.get_pressed()
        '''
        self.norm_vel = 0
        self.tang_vel = 0
        self.rot_vel = 0
        self.kick = 0
        if keys[pygame.K_SPACE]:
            self.kick = 1
        if keys[pygame.K_d]:
            self.norm_vel = -self.speed
        elif keys[pygame.K_a]:
            self.norm_vel = self.speed
        if keys[pygame.K_w]:
            self.tang_vel = self.speed
        elif keys[pygame.K_s]:
            self.tang_vel = -self.speed
        if keys[pygame.K_q]:
            self.rot_vel = self.rot_speed
        elif keys[pygame.K_e]:
            self.rot_vel = -self.rot_speed
    def run(self):
        '''
        brief: arranges values in vectorized format
        '''
        action = [self.kick,0,self.norm_vel, self.tang_vel, self.rot_vel]
        self.action = action
        return action
        
        
#simple test code 
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
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
            keys = pygame.key.get_pressed()
            key_action.keypress_update(keys)
    new_time = clock.tick()
    #visualization test
    #game.step(key_points = [([0,1000],10), ([0,-1000],-10)])
    game.step()
    ttime = new_time