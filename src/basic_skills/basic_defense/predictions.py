import math

class botPredictions:
    def __init__(self, bot):
        self.bot = bot
        self.x, self.y = self.bot.get_points()
        self.max_distance = 0
        self.angle = self.bot.get_angle()
        
    def makePredictions(self, pred_angle):
        points = []
        pred_angle = math.radians(pred_angle)
        for i in range(6):
            shootDistance = (i + 1) * 100
            x1 = int(shootDistance * math.cos(self.angle) + (shootDistance * math.tan(pred_angle) * (math.sin(self.angle))))
            x2 = int(shootDistance * math.cos(self.angle) - (shootDistance * math.tan(pred_angle) * (math.sin(self.angle))))
            y1 = int(shootDistance * math.sin(self.angle) - (shootDistance * math.tan(pred_angle) * (math.cos(self.angle))))
            y2 = int(shootDistance * math.sin(self.angle) + (shootDistance * math.tan(pred_angle) * (math.cos(self.angle))))
            points = points + [[x1 + self.x, y1 + self.y], [x2 + self.x, y2 + self.y]]

            if shootDistance > self.max_distance:
                self.max_distance = shootDistance
        return points, points[len(points) - 1], points[len(points) - 2]

    def shootingLine(self):
        x = int(self.max_distance * math.cos(self.angle)) + self.x
        y = int(self.max_distance * math.sin(self.angle)) + self.y
        return [self.bot.get_coor(), [x, y]]

class coordinatePredictions:
    def __init__(self, coor = [0, 0], angle = 0):
        self.x, self.y = coor[0], coor[1]
        self.max_distance = 0
        self.angle = angle
        
    def makePredictions(self, pred_angle):
        points = []
        pred_angle = math.radians(pred_angle)
        for i in range(6):
            shootDistance = (i + 1) * 100
            x1 = int(shootDistance * math.cos(self.angle) + (shootDistance * math.tan(pred_angle) * (math.sin(self.angle))))
            x2 = int(shootDistance * math.cos(self.angle) - (shootDistance * math.tan(pred_angle) * (math.sin(self.angle))))
            y1 = int(shootDistance * math.sin(self.angle) - (shootDistance * math.tan(pred_angle) * (math.cos(self.angle))))
            y2 = int(shootDistance * math.sin(self.angle) + (shootDistance * math.tan(pred_angle) * (math.cos(self.angle))))
            points = points + [[x1 + self.x, y1 + self.y], [x2 + self.x, y2 + self.y]]

            if shootDistance > self.max_distance:
                self.max_distance = shootDistance
        return points, points[len(points) - 1], points[len(points) - 2]

    def shootingLine(self):
        x = int(self.max_distance * math.cos(self.angle)) + self.x
        y = int(self.max_distance * math.sin(self.angle)) + self.y
        return [self.bot.get_coor(), [x, y]]

    def getViability(self, bots, goalCoor):
        totalValue = checkForEnemy(bots)
        totalValue += checkForEnemy(goalCoor)
        return totalValue

    def checkForEnemy(self, bots):
        prediction, edge1, edge2 = self.makePredictions(25)

        for enemyBot in bots:
            for i in range(self.size[0] - self.x):
                x1 = int(shootDistance * math.cos(self.angle) + (shootDistance * math.tan(pred_angle) * (math.sin(self.angle))))
                x2 = int(shootDistance * math.cos(self.angle) - (shootDistance * math.tan(pred_angle) * (math.sin(self.angle))))
                if x1 > x2:
                    
                if enemyBot.get_x():
                    pass

    def checkForGoal(self, coor = [(0, 0), (0, 80), (200, 0), (200, 80)]):
        pass
