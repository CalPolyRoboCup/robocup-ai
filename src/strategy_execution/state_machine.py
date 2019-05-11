
class state:
  def __init__(self, id):
    self.id = id
    self.transition = True
  def setup(self):
    pass
  def update():
    return self.id
    
    
class state_machine:
  def __init__(self, states):
    self.states = states
    self.current_state = self.states[0]
    self.current_state.setup()
  def run(self):
    new_state = self.current_state.update()
    if (new_state != self.current_state.id):
      self.states[new_state].setup()
    self.current_state = self.states[new_state]
  def reset():
    self.current_state = self.states[0]
    self.current_state.setup()