import random
import pygame
import matplotlib.pyplot as plt
import math
import numpy as np

class bot():
    '''DEPRECATED'''
    def __init__(self, team = 0, x = 0, y = 0, angle = 0):
        self.team = team
        self.x = x
        self.y = y
        self.angle = angle

    def getTeam(self):
        return self.team

    def getPosition(self):
        return (self.x, self.y, self.angle)

class defendBot:
    def __init__(self, enemyBot, bot, angle = 15):
        self.eBot = enemyBot
        self.bot = bot
        self.points = self.defendsivePoints(self.eBot.x, self.eBot.y, 5)

    def getPoints(self):
        return self.points

    def defendsivePoints(self, x, y, angle = 15):
        points = []
        rAngle = math.radians(self.eBot.angle)
        outAngle = math.radians(angle)
        for i in range(6):
            shootDistance = (i + 1) * 100
            x1 = int(shootDistance * math.cos(rAngle) + (shootDistance * math.tan(outAngle) * (math.sin(rAngle))))
            x2 = int(shootDistance * math.cos(rAngle) - (shootDistance * math.tan(outAngle) * (math.sin(rAngle))))
            y1 = int(shootDistance * math.sin(rAngle) - (shootDistance * math.tan(outAngle) * (math.cos(rAngle))))
            y2 = int(shootDistance * math.sin(rAngle) + (shootDistance * math.tan(outAngle) * (math.cos(rAngle))))
            points = points + [[x1 + x, y1 + y], [x2 + x, y2 + y]]
        return points

    def visualize(self):
        x = int(600*math.cos(math.radians(self.eBot.angle))) + self.eBot.x
        y = int(600*math.sin(math.radians(self.eBot.angle))) + self.eBot.y
        v = createVisualization(self.points, [[self.eBot.x, self.eBot.y], [x, y]])

class createVisualization():
    def __init__(self, points, shootLinePoint):
        pygame.init()
        size = [750, 750]
        screen = pygame.display.set_mode(size)
        done = False
        clock = pygame.time.Clock()

        while not done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            screen.fill((255, 255, 255))
            
            pygame.draw.lines(screen, (0, 0, 255), True, shootLinePoint, 2)
            pygame.draw.lines(screen, (0, 255, 0), True, [shootLinePoint[0], points[len(points) - 1]], 2)
            pygame.draw.lines(screen, (255, 0, 0), True, [shootLinePoint[0], points[len(points) - 2]], 2)
            pygame.draw.circle(screen, (255, 0, 255), shootLinePoint[0], 10)
            pygame.display.flip()


def mainCall():
    eBot = bot(1, random.randint(0, 700), random.randint(0, 700), random.randint(0, 360))
    aBot = bot(0, 10, 10, 40)
    d = defendBot(eBot, bot, 5)
    print(d.getPoints())
    d.visualize()

mainCall()
