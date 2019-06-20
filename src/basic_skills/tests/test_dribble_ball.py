import sys
import os
import time
import numpy as np
import pygame
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/../..')
from basic_skills.source.DribbleBall import DribbleBall
from pygame_simulator.PySim_noise import PYsim
from basic_skills.source.helper_functions import angle_to

game = PYsim(6)
dribble_action = DribbleBall(np.array([0,3000]))
game.add_action(dribble_action, 0, True)
i = 0

target_loc = np.array([0,3000])
look_loc = np.array([0,3000])
tick_time = 0

clock = pygame.time.Clock()
clock.tick(60)
ttime = clock.tick()
while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.MOUSEBUTTONDOWN:
            pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
            
            # left mouse button
            if pressed1:
                mouse_pos = game.convert_to_field_position(pygame.mouse.get_pos())
                target_loc = mouse_pos
    
    tick_time += 1
    
    # choose new locations to dribble to
    game.blue_robots[0].action.target_pos = target_loc
        
            
    # run the game and show target location
    new_time = clock.tick()
    if tick_time != 1:
        kp = [target_loc, dribble_action.target_loc]
    else:
        kp = [target_loc]
        
    game.step(key_points = kp)
    ttime = new_time