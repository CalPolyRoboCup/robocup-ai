import sys
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from config_data import Config, Path
from config_handler import read_trajectory, write_trajectory, read_placement, write_placement
from config_widget import ConfigWidget
from traj_widget import TrajWidget

file_filters = {'new': 'Placement file (*.place);;Trajectory file (*.traj)', 'place': 'Placement file (*.place)', 'traj': 'Trajectory file (*.traj)'}

PLACE = 0
TRAJ = 1

class App(QMainWindow):
    
    def __init__(self):
        super(App, self).__init__()
        
        # gui titles and dimensions
        self.title = 'Configuration GUI'
        self.left = 10
        self.top = 10
        self.width = 1200
        self.height = 800

        # action list
        self.actions = {}

        # widgets
        self.config = ConfigWidget()
        self.traj = TrajWidget()
        self.state = 0

        # initialize layout
        self.init_dims()
        self.init_actions()
        self.init_menu_bar()
        self.init_config_space()

        # saved metadata
        self.current_file = None
        self.is_saved = False # saved for first time, i.e new
        self.last_config = self.config.place_area.config
        self.last_path = self.traj.path
    
    # set the gui dimensions and title        
    def init_dims(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
    
    # create all actions used in gui
    def init_actions(self):
        exit_action = QAction('Exit', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.setStatusTip('Exit App')
        exit_action.triggered.connect(self.close)
        self.actions['exit'] = exit_action

        save = QAction('Save', self)
        save.setShortcut('Ctrl+S')
        save.triggered.connect(lambda: self.save_file())
        self.actions['save'] = save

        new = QAction('New', self)
        new.setShortcut('Ctrl+N')
        self.actions['new'] = new
        
        new_place = QAction('Placement', self)
        new_place.triggered.connect(lambda: self.create_placement())
        self.actions['new_place'] = new_place

        place = QAction('Placement', self)
        place.triggered.connect(lambda: self.load_placement())
        self.actions['place'] = place

        new_traj = QAction('Trajectory', self)
        new_traj.triggered.connect(lambda: self.create_trajectory())
        self.actions['new_traj'] = new_traj

        traj = QAction('Trajectory', self)
        traj.triggered.connect(lambda: self.load_trajectory())
        self.actions['traj'] = traj

    # create the menu bar
    def init_menu_bar(self):
        self.statusBar()
        menu = self.menuBar()
        file_menu = menu.addMenu('File')
        new_menu = file_menu.addMenu('New')
        new_menu.addAction(self.actions['new_place'])
        new_menu.addAction(self.actions['new_traj'])
        load_menu = file_menu.addMenu('Load')
        load_menu.addAction(self.actions['place'])
        load_menu.addAction(self.actions['traj'])
        file_menu.addAction(self.actions['save'])
        file_menu.addAction(self.actions['exit'])
    
    # initalize the configuration space, aka where everything happens
    def init_config_space(self):
        config_space_group = self.config.init_view()
        self.state = PLACE
        self.setCentralWidget(config_space_group)

    def init_traj_space(self):
        field_group = self.traj.init_view()
        self.state = TRAJ
        self.setCentralWidget(field_group)

    # handle file dialog
    def get_file(self, ext):
        dialog = QFileDialog()
        dialog.setFileMode(QFileDialog.AnyFile)
        dialog.setFilter(ext)
        
        files = QStringList()
        
        if dialog.exec_():
            files = dialog.selectedFiles()
            return files[0]
        return None

    def create_trajectory(self):
        f = self.get_file(file_filters['traj'])
        if f is not None:
            self.current_file = f
            self.is_saved = False
            self.init_traj_space()
    
    def load_trajectory(self):
        print 'trying to load trajectories'
        f = self.get_file(file_filters['traj'])
        if f is not None:
            self.init_traj_space()
            self.traj.path = read_trajectory(f)
            self.current_file = f
            self.last_path = self.traj.path
            self.traj.update()
            self.is_saved = True

    def save_trajectory(self):
        print 'saving trajectory'
        write_trajectory(self.current_file, self.traj.path)
        self.is_saved = True
        self.last_path = self.traj.path

    def create_placement(self):
        f = self.get_file(file_filters['place'])
        if f is not None:
            self.current_file = f
            self.is_saved = False
            self.init_config_space()
            
    def load_placement(self):
        f = self.get_file(file_filters['place'])
        if f is not None:
            self.init_config_space()
            self.config.place_area.config = read_placement(f)
            self.current_file = f
            self.last_config = self.config.place_area.config
            self.config.place_area.update()
            self.is_saved = True

    def save_placement(self):
        print 'saving placement'
        write_placement(self.current_file, self.config.place_area.config)
        self.is_saved = True
        self.last_config = self.config.place_area.config

    def save_file(self):
        print 'saving'
        if self.state == PLACE:
            self.save_placement()
        elif self.state == TRAJ:
            self.save_trajectory()

    def closeEvent(self, e):
        if self.is_saved:
            if self.state == PLACE:
                print self.last_config.ball
                print self.config.place_area.config.ball
                if self.last_config == self.config.place_area.config:
                    e.accept()
                else:
                    print 'hi'
                    e.ignore()
            elif self.state == TRAJ:
                print self.last_path == self.traj.path
                if self.last_path == self.traj.path:
                    print 'closing application'
                    e.accept()
                else:
                    e.ignore()
        else:
            e.ignore()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = App()
    win.show()
    sys.exit(app.exec_())
