#!/usr/bin/env python

class Robot():
    def __init__(self, pos, vel, angl, id):
        self.pos = pos
        self.vel = vel
        self.angl = angl
        self.id = id
        self.strat = None

    def get_pos(self):
        return self.pos

    def get_vel(self):
        return self.vel
    
    def get_angl(self):
        return self.angl

    def get_id(self):
        return self.id

    def get_strat(self):
        return self.strat

    def update_id(self, newID):
        self.id = newID

    def update_vals(self, newPos, newVel, newAngl):
        if newPos != None:
            self.pos = newPos
        if newVel != None:
            self.vel = newVel
        if newAngl != None:
            self.angl = newAngl

    def update_strat(self, strat):
        self.strat = strat