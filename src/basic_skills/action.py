class action:
  def __init__(self):
    #the robot this action is bound to
    self.robot = False
    #the result of the last call to run
    self.action = [0,0,0,0,0]
    self.game = False
  def run(self, game):
    self.action = [0,0,0,0,0]
    return self.action
  def add(self, robot, game):
    #run when action added to robot
    #use for one time calculations
    self.robot = robot
    self.game = game
  def end(self):
    #clean up
    pass
  def done(self):
    #return true is action is complete otherwise false
    return False
