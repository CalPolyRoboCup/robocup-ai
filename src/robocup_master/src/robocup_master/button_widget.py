from PyQt4.QtGui import QButtonGroup, QRadioButton, QGroupBox, QVBoxLayout

class ButtonWidget(QButtonGroup):
    def __init__(self):
        super(ButtonWidget, self).__init__()
        
        # buttons for adding robots and ball
        self.yellow_btn = QRadioButton('Yellow Team')
        self.blue_btn = QRadioButton('Blue Team')
        self.ball_btn = QRadioButton('Ball')
        
        self.addButton(self.yellow_btn)
        self.addButton(self.blue_btn)
        self.addButton(self.ball_btn)
        self.setExclusive(True)

    def init_view(self):

        # initializing groups and layouts
        button_group = QGroupBox('Edit')
        button_layout = QVBoxLayout()
        robot_group = QGroupBox('Robots')
        robot_layout = QVBoxLayout()
        ball_group = QGroupBox('Ball')
        ball_layout = QVBoxLayout()
        
        # robot buttons to group
        robot_layout.addWidget(self.yellow_btn)
        robot_layout.addWidget(self.blue_btn)
        robot_layout.addStretch(1)
        robot_group.setLayout(robot_layout)

        # ball button to group
        ball_layout.addWidget(self.ball_btn)
        ball_layout.addStretch(1)
        ball_group.setLayout(ball_layout)

        # layout for button groups
        button_layout.addWidget(robot_group)
        button_layout.addWidget(ball_group)
        button_layout.addStretch(1)
        button_group.setLayout(button_layout)
        return button_group

