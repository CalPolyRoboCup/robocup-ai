import sys
sys.path.insert(0, '../..')
from basic_skills.dribble_ball.dribble_ball import dribble_ball
from pygame_simulator.PySim_noise import *

game = PYsim(6)
dribble_action = dribble_ball(game, np.array([0,3000]))
game.add_action(dribble_action, 0, True)
i = 0

target_loc = np.array([0,3000])
time = 0

clock = pygame.time.Clock()
clock.tick(60)
ttime = clock.tick()
while 1:
  for event in pygame.event.get():
    if event.type == QUIT:
      pygame.quit()
      sys.exit()
  
  time += 1
  
  # choose new locations to dribble to
  if time % 700 == 0:
    target_loc = np.random.uniform(-1, 1, size = [2]) * np.array([4000, 2000])
  game.blue_Robots[0]._action.target_loc = target_loc
    
      
  # run the game and show target location
  new_time = clock.tick()
  if time != 1:
    kp = [target_loc, (game.blue_Robots[0]._action.moving_to, -3)]
  else:
    kp = [target_loc]
  game.step(key_points = kp)
  ttime = new_time