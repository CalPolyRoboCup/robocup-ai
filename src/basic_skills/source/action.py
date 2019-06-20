class action:
    def __init__(self):
        #the robot this action is bound to
        self.robot = None
        
        #the game this action is in
        self.game = None
        
        #the result of the last call to run
        self.output = [0,0,0,0,0]
        
        #0 don't kick 1 kick
        self.kick = 0
        
        #whether to chip the ball
        #we can't do this. 
        self.chip = 0
        
        #Forwards backwards movement
        self.norm_vel = 0
        
        #Side to side movement
        self.tang_vel = 0
        
        #rotational movement
        self.rot_vel = 0

        #don't remove actions automatically when they finish
        #it makes debuging hard
        self.terminate_when_done = False
        
    def run(self):
        #generate outputs in a vectored form
        self.output = [self.kick, self.chip, self.norm_vel, self.tang_vel, self.rot_vel]
        return self.output
        
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
