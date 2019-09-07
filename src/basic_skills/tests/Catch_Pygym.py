import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/../..')

from pygame_simulator.PySim_noise import *
from basic_skills.source.helper_functions import *

class Ball_Intercept_PYGym (PYsim):
    def __init__(self, max_bots):
        PYsim.__init__(self,max_bots)
        self.cat = []
        self.dog = []
        self.fish = []
        self.time = 0
        
        self.timeout = 0
        self.timeout_value = 2500
        
        self.control_time = 300
        self.control_counter = 0
        
    def score(self):
        velocity_loss = mag(self.ball.velocity)
        nearness_loss = mag(self.ball.loc - self.blue_robots[0].loc)
        return velocity_loss + nearness_loss
        
    def new_scenario(self):
        '''
        put the ball in a new position and kick it at the blue robot with id 0
        '''
        self.timeout = self.timeout_value
        random_offset = np.random.uniform(-1, 1, size = [2])*np.array([3000, 2500])
        self.ball_internal.loc = random_offset
        random_offset = np.random.uniform(-1, 1, size = [2])*np.array([500, 500])
        target_loc = self.blue_robots[0].loc + random_offset
        speed = np.random.uniform(100,800)
        self.ball_internal.velocity = (target_loc - self.ball.loc)
        self.ball_internal.velocity *= speed/mag(self.ball_internal.velocity)
        print(self.ball_internal.velocity, self.ball_internal.loc)
        self.control_counter = 0
        
    def step(self, key_points = []):
        '''
        kick balls at the blue robot with id 0 and wait for it to catch it
        '''
        self.time += 1
        if self.ball.controler != False:
            if self.control_counter == 0:
                self.control_counter = self.control_time
            else:
                self.control_counter -= 1
                if self.control_counter == 0:
                    self.new_scenario()
                    
        self.timeout -= 1
        if (self.timeout <= 0):
            self.new_scenario()
        
        new_state = PYsim.step(self, key_points = key_points)
        return new_state, self.score()