from place_widget import PlaceWidget
from button_widget import ButtonWidget
from PyQt4.QtGui import QWidget, QGroupBox, QHBoxLayout

class ConfigWidget(QWidget):
    def __init__(self):
        super(ConfigWidget, self).__init__()
        self.place_area = PlaceWidget()
        self.init_btns()

    def init_btns(self):
        self.place_btn = ButtonWidget()
        for b in self.place_btn.buttons():
            self.place_btn.buttonClicked.connect(lambda: self.bind_btn_field())

    def bind_btn_field(self):
        self.place_area.checked_btn = self.place_btn.checkedId()

    def init_view(self):
 
        # set up group and layout for widget       
        config_space_group = QGroupBox()
        config_space_layout = QHBoxLayout()

        # initalize other widgets
        self.place_area = PlaceWidget()
        self.init_btns()
        
        # set up group and layout for other widgets
        button_group = self.place_btn.init_view()
        field_group = self.place_area.init_view()
        
        # add widgets to layout
        config_space_layout.addWidget(field_group)
        config_space_layout.addWidget(button_group)
        config_space_group.setLayout(config_space_layout)
        return config_space_group
