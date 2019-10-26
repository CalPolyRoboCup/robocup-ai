import numpy as np
import math
import time
import sys
import os
import pygame

dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname)
from strategy_helpers import team
from evaluate_robots import evaluate_robots

sys.path.insert(0, dirname+'/..')
from pygame_simulator.PySim_noise import PYsim
from basic_skills.source.helper_functions import angle_to
from basic_skills.source.worst_intercept import worst_intercept

if __name__ == "__main__":
    game = PYsim(6)

    blue_team = team(game, is_blue=True)
    yellow_team = team(game, is_blue=False)
    yellow_team.ball_controler = blue_team.enemies[0]
    best_reciever_index = None

    clock = pygame.time.Clock()
    clock.tick(60)
    ttime = clock.tick()

    select_id = 0
    select_blue = True
    while 1:
        _, _, done = game.step(key_points=yellow_team.prints)

        values, best_reciever_index = evaluate_robots(yellow_team, best_reciever_index)

        goal_shot_safety, _ = worst_intercept(yellow_team.allies[0].loc, yellow_team.enemy_goal, yellow_team.enemies)
        #print("goal shot", goal_shot_safety, _)

        '''
        select teams with Y key (yellow)
                          B key (blue)
        select robots with number keys (0-5)
        left click to place selected robot
        '''
        for event in pygame.event.get():
            if event.type ==pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
                keys = pygame.key.get_pressed()
                # press r-key to reset
                if keys[pygame.K_r]:
                    game.reset()
                    yellow_strategy.reset()
                if keys[pygame.K_1]:
                    select_id = 1
                if keys[pygame.K_2]:
                    select_id = 2
                if keys[pygame.K_3]:
                    select_id = 3
                if keys[pygame.K_4]:
                    select_id = 4
                if keys[pygame.K_5]:
                    select_id = 5
                if keys[pygame.K_0]:
                    select_id = 0
                if keys[pygame.K_b]:
                    select_blue = True
                if keys[pygame.K_y]:
                    select_blue = False
            if event.type == pygame.MOUSEBUTTONDOWN or event.type == pygame.MOUSEMOTION:
                pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()

                mouse_pos = pygame.mouse.get_pos()
                mouse_pos = game.convert_to_field_position(mouse_pos)

                # left mouse button
                if pressed1:
                    if select_blue:
                        game.blue_robots_internal[select_id].loc = mouse_pos
                    else:
                        game.yellow_robots_internal[select_id].loc = mouse_pos

                if pressed3:
                    if select_blue:
                        mouse_angle = angle_to(mouse_pos, game.blue_robots_internal[select_id].loc)
                        game.blue_robots_internal[select_id].rot = mouse_angle
                    else:
                        mouse_angle = angle_to(mouse_pos, game.yellow_robots_internal[select_id].loc)
                        game.yellow_robots_internal[select_id].rot = mouse_angle

                if pressed2:
                    game.ball_internal.loc = mouse_pos
                    game.ball_internal.velocity = np.array([0,0])
