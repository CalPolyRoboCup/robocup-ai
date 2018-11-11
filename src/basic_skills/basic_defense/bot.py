import random
import math

class bot:
    def __init__(self, hasBall, team, coordinate = [0, 0], angle = 0):
        teamDict = {0: "Ally", 1: "Enemy"}
        self.hasBall = hasBall
        self.team = teamDict.get(team)
        self.x = coordinate[0]
        self.y = coordinate[1]
        self.angle = angle

    # def __init__(self, hasBall, team):
    #     teamDict = {0: "Ally", 1: "Enemy"}
    #     self.hasBall = hasBall
    #     self.team = teamDict.get(team)
    #     self.x = random.randint(150, 650)
    #     self.y =  random.randint(150, 650)
    #     self.angle = random.randint(0, 360)
    
    def __lt__(self, other):
        return self.x < other.x and self.y < other.y

    def __str__(self):
        return "Bot is on {} team at ({}, {}) with angle {}. Has ball?: {} ".format(self.team, self.x, self.y, self.angle, self.hasBall)

    def set_angle(self, angle):
        self.angle = angle

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_angle(self, angle):
        self.angle = angle

    def set_team(self, team):
        self.team = team

    def set_hasBall(self, hasBall):
        self.hasBall = hasBall

    def get_pos(self):
        return (self.x, self.y, self.angle)

    def get_coor(self):
        return [self.x, self.y]

    def get_points(self):
        return self.x, self.y

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_team(self):
        return self.team

    def get_angle(self):
        return math.radians(self.angle)

    def get_hasBall(self):
        return self.hasBall