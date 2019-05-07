from setup_widget import SetupWidget
from config_data import Robot, Ball
from PyQt4.QtGui import QPainter, QPen, QColor
from PyQt4.QtCore import Qt
from config_data import Config

class PlaceWidget(SetupWidget):
    def __init__(self):
        super(PlaceWidget, self).__init__()
        self.config = Config()
        self.checked_btn = -1
        self.robot_color = 0
        self.max_robots = 10

    def paintEvent(self, e):
        painter = QPainter(self)
        self.draw_bkgnd()
        self.draw_robots()
        self.draw_ball()
        painter.drawPixmap(self.rect(), self.field)

    def draw_robots(self):
        painter = QPainter(self.field)
        painter.translate(self.field.width()/2, self.field.height()/2)

        for r in self.config.robots_yellow:
            painter.setPen(QPen(Qt.yellow, 1))
            painter.setBrush(Qt.yellow)
            painter.drawEllipse(r.pose.x, r.pose.y, 15, 15)

        for r in self.config.robots_blue:
            painter.setPen(QPen(Qt.blue, 1))
            painter.setBrush(Qt.blue)
            painter.drawEllipse(r.pose.x, r.pose.y, 15, 15)

    def draw_ball(self):
        painter = QPainter(self.field)
        painter.translate(self.field.width()/2, self.field.height()/2)
        ball = self.config.ball

        if ball is not None:
            painter.setPen(QPen(QColor(255,165,0), 1))
            painter.setBrush(QColor(255,165,0))
            painter.drawEllipse(ball.x, ball.y, 4, 4)

    def mousePressEvent(self, e):
        if self.checked_btn == -2:
            self.robot_color = Qt.yellow
            self.config.robots_yellow = self.update_robots(e, self.config.robots_yellow)                           
            self.draw_robots()

        elif self.checked_btn == -3:
            self.robot_color = Qt.blue
            self.config.robots_blue = self.update_robots(e, self.config.robots_blue)
            self.draw_robots()

        elif self.checked_btn == -4:
            if self.config.ball is None:
                self.config.ball = Ball()

            self.config.ball.x = e.x() - (self.width()/2)
            self.config.ball.y = e.y() - (self.height()/2)
                
            self.draw_ball()

        self.update()

    def update_robots(self, e, robots):
        tmp_x = e.x() - (self.width()/2)
        tmp_y = e.y() - (self.height()/2)
        add_flg = True
        for r in robots:
            dist_x = abs(tmp_x - r.pose.x)
            dist_y = abs(tmp_y - r.pose.y)
            if (dist_x < 10) and (dist_y < 10):
                if r.selected:
                    r.selected = False
                else:
                    r.selected = True
                    add_flg = False
            else:
                r.selected = False

        if add_flg and (len(robots) < self.max_robots):
            new_robot = Robot()
            new_robot.pose.x = tmp_x
            new_robot.pose.y = tmp_y
            new_robot.selected = False
            new_robot.color = self.robot_color
            robots.append(new_robot)
        return robots

    def mouseMoveEvent(self, e):
        robots = []
        if self.robot_color == Qt.yellow:
                robots = self.config.robots_yellow
        elif self.robot_color == Qt.blue:
            robots = self.config.robots_blue

        for r in robots:
            if r.selected:
                r.pose.x = e.x() - (self.width() / 2)
                r.pose.y = e.y() - (self.height() / 2)
                self.draw_robots()
                self.update()
                break
