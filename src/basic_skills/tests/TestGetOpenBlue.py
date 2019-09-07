import sys
import os
import pygame
import numpy as np
dirname = os.path.dirname(__file__)

sys.path.insert(0, dirname+'/../..')
from basic_skills.source.GetOpen import Striker, Fielder, GetOpenForKick
from pygame_simulator.PySim_noise import PYsim
from basic_skills.source.robot import robot

if __name__ == "__main__":
    max_bots_per_team = 6
    game = PYsim(max_bots_per_team)
    clock = pygame.time.Clock()
    clock.tick(60)
    ttime = clock.tick()
    
    '''
    Create ball controller
    assumes blue_robot 1 has the highest pass values and blue_robot 0 has the ball or can easily get the ball
    '''
    game.add_action(GetOpenForKick([game.blue_robots[1], game.blue_robots[3]], game.yellow_robots, game.blue_robots), 0, True)
    
    '''
    Create the Strikers
    assumes blue_robot 0 has the ball and [-5000,0] is the goal
    '''
    
    blocker_enemies = [robot(True, -1, None), robot(True, -1, None)]
    enemy_goal = game.yellow_goal_loc
    blocker_enemies[0].loc = enemy_goal + np.array([0, game.goal_height/2 + 500])
    blocker_enemies[1].loc = enemy_goal + np.array([0, -game.goal_height/2 - 500])
    for i in game.blue_robots[1:3]:
        game.add_action(Striker(game.blue_robots[0], game.yellow_goal_loc, game.yellow_robots + blocker_enemies, game.blue_robots), i.id, True)
            
    '''
    Create the Fielders
    assumes blue_robot 0 has the ball and blue_robot i.id - 2 is a Striker
    '''
    for i in game.blue_robots[3:5]:
        game.add_action(Fielder(game.blue_robots[0], game.blue_robots[i.id - 2], game.yellow_robots + blocker_enemies, game.blue_robots), i.id, True)
        
        
    while 1:
            
            
        '''
        handle resets and allow robots to be placed for debugging
        '''
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
                keys = pygame.key.get_pressed()
                
                # press r-key to reset
                if keys[pygame.K_r]:
                    game.reset()

        pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
        # left mouse button
        if pressed1:
            game.yellow_robots_internal[1].loc = game.convert_to_field_position(pygame.mouse.get_pos())
            
        # right mouse button
        if pressed3:
            game.yellow_robots_internal[0].loc = game.convert_to_field_position(pygame.mouse.get_pos())

        new_time = clock.tick()
        if game.blue_robots[0].action.target_pos is not None:
            game.step(key_points=game.blue_robots[1].action.prints)
        else:
            game.step()
        ttime = new_time