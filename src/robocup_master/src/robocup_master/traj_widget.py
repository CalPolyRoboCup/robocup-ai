from setup_widget import SetupWidget
from PyQt4.QtGui import QPainter, QPen
from PyQt4.QtCore import Qt, QPointF
from config_data import Path, Pose

class TrajWidget(SetupWidget):
    def __init__(self):
        super(TrajWidget, self).__init__()
        self.path = Path()

    def paintEvent(self, e):
        painter = QPainter(self)
        self.draw_bkgnd()
        self.draw_pts()
        painter.drawPixmap(self.rect(), self.field)

    def draw_pts(self):
        painter = QPainter(self.field)
        painter.setPen(QPen(Qt.red, 4))
        painter.translate(self.field.width()/2, self.field.height()/2)
        for p in self.path.poses: 
            pt = QPointF(p.x, p.y)
            painter.drawPoint(pt)

    def mousePressEvent(self, e):
        new_pose = Pose()
        new_pose.x = e.x() - (self.width()/2)
        new_pose.y = e.y() - (self.height()/2)
        self.path.poses.append(new_pose)
        self.draw_pts()
        self.update()
