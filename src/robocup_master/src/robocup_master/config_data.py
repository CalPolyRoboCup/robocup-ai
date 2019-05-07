from PyQt4.QtCore import Qt
'''
Metadata items for gui
'''
class Config:
    def __init__(self):
        self.robots_yellow = []
        self.robots_blue = []
        self.ball = None

    def __str__(self):
        val = 'yellow:\n'
        for r in self.robots_yellow:
            val += str(r)
            val += '\n'
        
        val += 'blue:\n'
        for r in self.robots_blue:
            val += str(r)
            val += '\n'
        
        val += str(self.ball)
        return val
    
    def __eq__(self, other):
        if self.ball != other.ball:
            return False

        for i in range(len(self.robots_yellow)):
            try:
                if self.robots_yellow[i] != other.robots_yellow[i]:
                    print 'here 1'
                    return False
            except IndexError:
                    print 'here 2'
                    return False
         
        for i in range(len(self.robots_blue)):
            try:
                if self.robots_blue[i] != other.robots_blue[i]:
                    print 'here 3'
                    return False
            except IndexError:
                    print 'here 4'
                    return False
        print 'here 5'
        return True

    def __ne__(self, other):
        return not self.__eq__(other)

class Path:
    def __init__(self):
        self.poses = []
    
    def __str__(self):
        val = ''
        for p in self.poses:
            val += str(p)
            val += '\n'
        return val
    
    def __eq__(self, other):
        if len(self.poses) != len(other.poses):
            return False

        for i in range(len(self.poses)):
            if self.poses[i] != other.poses[i]:
                return False
        return True

    def __ne__(self, other):
        return not self.__eq__(other)

class Robot:
    def __init__(self, x=0.0, y=0.0, theta=0.0, selected = False, color = Qt.yellow):
        self.pose = Pose(x, y, theta)
        self.selected = selected
        self.color = color

    def __str__(self):
        return 'Robot: \nLocation: {0}, {1}, {2}\nColor: {3}\nSelected: {4}'.format(self.pose.x, self.pose.y, self.pose.theta, self.color, self.selected)
    def __eq__(self, other):
        if type(self) == type(other):
            return (self.pose == other.pose) and (self.color == other.color)
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

class Ball:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __str__(self):
        return 'Ball: {0}, {1}'.format(self.x, self.y)

    def __eq__(self, other):
        if type(self) == type(other):
            return (self.x == other.x) and (self.y == other.y)
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return 'Pose: {0}, {1}, {2}'.format(self.x, self.y, self.theta)

    def __eq__(self, other):
        if type(self) == type(other):
            return (self.x == other.x) and (self.y == other.y) and (self.theta == other.theta)
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)
