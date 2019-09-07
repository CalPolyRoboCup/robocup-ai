import numpy as np
import math
import time
import sys
import os
import pygame

dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from neutral_strategy import neutral_strategy
from offensive_strategy import offensive_strategy
from defensive_strategy import defensive_strategy
from passing_strategy import passing_strategy
from strategy_numbers import *
from state_machine import state_machine
from strategy_helpers import team

sys.path.insert(0, dirname+'/..')
from pygame_simulator.PySim_noise import PYsim
from basic_skills.source.Goalie import Goalie



class strategy:
    def __init__(self, game, is_blue):
        self.team = team(game, is_blue)
        self.neutral = neutral_strategy(NEUTRAL_STRATEGY_STATE_NUMBER, self.team)
        self.defense = defensive_strategy(DEFENSIVE_STRATEGY_STATE_NUMBER, self.team)
        self.offense = offensive_strategy(OFFENSIVE_STRATEGY_STATE_NUMBER, self.team)
        self.passing = passing_strategy(PASSING_STRATEGY_STATE_NUMBER, self.team)
        self.state_machine = state_machine([self.neutral, self.defense, self.offense, self.passing])

        self.reset()

    def update(self):
        self.team.update()
        self.state_machine.run()

    def reset(self):
        self.team.game.add_action(Goalie(), self.team.goalie.id, self.team.is_blue)
        self.state_machine.reset()

if __name__ == "__main__":
    game = PYsim(6)

    blue_strategy = strategy(game, is_blue=True)
    yellow_strategy = strategy(game, is_blue=False)
    blue_strategy.reset()
    yellow_strategy.reset()

    clock = pygame.time.Clock()
    clock.tick(60)
    ttime = clock.tick()

    select_id = 0
    select_blue = True
    while 1:
        _, _, done = game.step()#key_points=yellow_strategy.team.prints)

        # reset strategy when game ends
        if done:
            blue_strategy.reset()
            yellow_strategy.reset()

        blue_strategy.update()
        yellow_strategy.update()

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
                # left mouse button
                if pressed1:
                    mouse_pos = pygame.mouse.get_pos()
                    mouse_pos = game.convert_to_field_position(mouse_pos)
                    if select_blue:
                        game.blue_robots_internal[select_id].loc = mouse_pos
                    else:
                        game.yellow_robots_internal[select_id].loc = mouse_pos

