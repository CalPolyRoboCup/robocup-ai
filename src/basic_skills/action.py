from abc import *

class CommandStatus:
  """
  An enumeration used to indicate the status of a command
  """
  RUNNING = 0
  COMPLETED = 1
  FAILED = 2
  INTERRUPTED = 3

class Action (object):
  def __init__(self, game):
    # the robot this action is bound to
    self._robot = None
    
    # the game this action is in
    self.game = game
    
    # the result of the last call to run
    self.output = [0,0,0,0,0]
    
    # 0 don't kick 1 kick
    self.kick = 0
    
    # whether to chip the ball
    # we can't do this. 
    self.chip = 0
    
    # Forwards backwards movement
    self.norm_vel = 0
    
    # Side to side movement
    self.tang_vel = 0
    
    # rotational movement
    self.rot_vel = 0
    
    self.blue_bots = game.blue_Robots
    self.yellow_bots = game.yellow_Robots
    #self.field = game.field
    self.ball = game.ball
    self.master = game
    
  def run(self, delta_time):
    # generate outputs in a vectored form
    self.output = [self.kick, self.chip, self.norm_vel, self.tang_vel, self.rot_vel]
    return self.output
    
  def add(self, robot):
    # run when action added to robot
    # use for one time calculations
    if not robot.is_virtual_robot:
      self._robot = robot
      self.assigned()

    return True

  def get_robot(self):
    """
    Returns the robot assigned with this command
    :return: The robot assigned with this command
    """
    return self._robot

  def is_finished(self):
    """
    Returns true if the command has finished execution
    :return: True if the command has finished execution, otherwise False
    """
    return self.get_status() != CommandStatus.RUNNING

  @abstractmethod
  def assigned(self):
    """
    Called when the command is first assigned to a robot
    """
    pass

  @abstractmethod
  def start(self):
    """
    Called when the command starts execution
    """
    pass

  @abstractmethod
  def update(self, delta_time):
    """
    Called continuously throughout the command's execution
    :param delta_time: The time passed since the last update
    """
    pass

  @abstractmethod
  def get_status(self):
    """
    Used to determine if this command has finished executing
    :return: True if the command has finished executing, otherwise False
    """
    return CommandStatus.RUNNING

  @abstractmethod
  def end(self, command_status):
    """
    Called when this command quits execution
    :param command_status: The status of the command when its execution was ended
    """
    pass
    
  def set_robot(self, robot):
    """
    Sets the parent robot to the given robot, if possible
    :param robot: The robot to assign this command to
    :return: True if the command could be assigned to the given robot, otherwise False
    """
    if self._robot is not None:
      return False

    self._robot = robot
    self.assigned()
    return True


