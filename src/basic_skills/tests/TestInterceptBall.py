import sys
import os
import pygame
import numpy as np
dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname+'/../..')
from basic_skills.source.InterceptBall import InterceptBall
from pygame_simulator.PySim_noise import PYsim

if __name__ == "__main__":
    game = PYsim(6)
    catch_action = InterceptBall()
    game.add_action(catch_action, 1, True)
    i = 0
    
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
                
                # place ball with left mouse
                if pressed1:
                    game.ball_internal.loc = game.convert_to_field_position(pygame.mouse.get_pos())
                    game.ball_internal.velocity = np.array([0,0])
                    
                # throw ball with right mouse
                if pressed3:
                    game.ball_internal.velocity = (game.convert_to_field_position(pygame.mouse.get_pos()) - game.ball_internal.loc)
        new_time = clock.tick()
        game.step(key_points=[catch_action.target_loc])
        ttime = new_time