import sys
import os
import numpy as np
import pygame
dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname+'/../..')

from pygame_simulator.PySim_noise import PYsim
from basic_skills.source.cover import cover
from basic_skills.source.MoveTo import MoveTo

if __name__ == "__main__":

    game = PYsim(6)

    cover_action = cover(np.array([0,3000]), game.yellow_robots[0])
    CR_action = cover(game.blue_robots[0], game.yellow_robots[0])
    move_action = MoveTo()

    game.add_action(cover_action, 0, True)
    game.add_action(move_action, 0, False)
    game.add_action(CR_action, 1, False)

    move_action.set_target([3000,0], 0)
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
                    mouse_pos = pygame.mouse.get_pos()
                    mouse_pos = game.convert_to_field_position(mouse_pos)
                    move_action.set_target(mouse_pos, 0)
                    
        new_time = clock.tick()
        game.step()
        ttime = new_time