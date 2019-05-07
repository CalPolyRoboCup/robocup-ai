from PyQt4.QtGui import QWidget, QPixmap, QGroupBox, QVBoxLayout, QPainter, QPen
from PyQt4.QtCore import Qt

class SetupWidget(QWidget):
    def __init__(self):
        super(SetupWidget, self).__init__()
        self.setFixedSize(900, 600)
        self.field = QPixmap(self.size())

    def init_view(self):
        field_group = QGroupBox('Field')
        field_layout = QVBoxLayout()
        field_layout.addWidget(self)
        field_layout.addStretch(1)
        field_group.setLayout(field_layout)
        return field_group

    def draw_bkgnd(self):
        # initialize grass
        self.field.fill(Qt.green)
        
        # setup painter
        painter = QPainter(self.field)
        painter.setPen(QPen(Qt.white, 3))
    
        # draw field lines
        painter.drawLine(450, 0, 450, 600)
        painter.drawRect(0, 0, 900, 600)
        painter.drawRect(0, 200, 100, 200)
        painter.drawRect(800, 200, 100, 200)
        painter.drawEllipse(400, 250, 100, 100)
 

