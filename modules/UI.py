# UI Code
# Almost everything is derived from MainWindow and its functions

from PyQt6 import QtCore, QtGui, QtWidgets
import numpy as np
import copy
import os
import matplotlib as mpl
mpl.use('QtAgg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.ticker import ScalarFormatter
import matplotlib.pyplot as plt
import yaml

import modules.towers as TW
import modules.bases as BA
import modules.weather as WT
import modules.coordinates as CO
import modules.uav as UAVS
import modules.solver as SO
import modules.simulation as SIM
import modules.yaml as YAML

def set_bit(v: int, index:int, x:int) -> int:
  """
  Set the index:th bit of v to 1 if x is truthy, else to 0, and return the new value.
  """
  mask = 1 << index-1 # Compute mask, an integer with just bit 'index' set.
  v &= ~mask          # Clear the bit indicated by the mask (if x is False)
  if x:
    v |= mask         # If x was True, set the bit indicated by the mask.

  return v            # Return the result, we're done

def is_set(v:int, n:int) -> bool:
    return v & 1 << n-1 != 0

global version
version = 0.2

class MainWindow(QtWidgets.QMainWindow):
    # Main Window initialization. It contains all permanent variables within the program
    def __init__(self, *args, **kwargs):
        """
        Main Window for the UI. It contains variables as parameters that store pretty much everything needed for the UI to run and 
        to solve and plot the problem. 
        """
        super(MainWindow, self).__init__(*args, **kwargs)

        self.current_directory  = os.getcwd()

        # Some permanent parameters and data -------------

        self.status_flag =            0b00000
        #                               ||||\-- Input data? 
        #                               |||\--- Weather?
        #                               ||\---- Problem Solved?
        #                               |\----- Sim Trajectories?
        #                               \------ Waypoints?
        #

        self.towers = TW.Towers()
        self.bases = BA.Bases()
        self.uavs = UAVS.UAV_Team()
        self.location_LatLon = [37.411669, -6.0016236] # [Latitude, Longitud] GRVC Lab as default
        self.problem_graph = None

        # Window settings --------------------------------

        self.setWindowTitle("Smart Flies Planner Alpha v"+ str(version))
        self.resize(1260, 900)
        self.icon = os.path.join( self.current_directory, 'ico.svg' )
        self.setWindowIcon(QtGui.QIcon(self.icon))

        # Menu Bar ---------------------------------------

        menubar = self.menuBar()

        self.solverType = 'abstract'

        menu0 = menubar.addMenu('File')
        db_action01 = menu0.addAction("Load Mission Initialization")
        db_action01.setStatusTip("Load a YAML file for the problem definition")
        db_action01.triggered.connect(lambda: load_data_from_YAML(self))

        db_action02 = menu0.addAction("Load Bases KML")
        db_action02.setStatusTip("Not implemented yet")
        db_action03 = menu0.addAction("Load Towers KML")
        db_action03.setStatusTip("Not implemented yet")

        menu = menubar.addMenu('Theme')
        db_action11 = menu.addAction("Light Theme")
        db_action12 = menu.addAction("Dark Theme")
        db_action11.setStatusTip("Apply Light theme")
        db_action11.triggered.connect(self.setLightTheme)
        db_action12.setStatusTip("Apply Dark theme")
        db_action12.triggered.connect(self.setDarkTheme)

        menu2 = menubar.addMenu('Solver')
        db_action21 = menu2.addAction("Regular Solver")
        db_action21.setStatusTip("Use the Regular Solver for the planner")
        db_action21.triggered.connect(lambda: self.changeSolverType('regular'))

        db_action22 = menu2.addAction("Abstract Solver")
        db_action22.setStatusTip("Use the Abstract Solver for the planner")
        db_action22.triggered.connect(lambda: self.changeSolverType('abstract'))

        db_action23 = menu2.addAction("GRASP Solver")
        db_action23.setStatusTip("Work in Progress")

        menu3 = menubar.addMenu("Weather")
        db_action31 = menu3.addAction("Edit manually")
        db_action31.triggered.connect(lambda: get_weatherWindow(self))

        # Status Bar -----------------------------------------------

        self.statusbar = QtWidgets.QStatusBar(parent=self)
        self.setStatusBar(self.statusbar)

        # Central Widget = Matplotlib Graphics and Toolbar ---------

        self.verticalLayout = QtWidgets.QVBoxLayout()
        
        self.sc = MplCanvas(self, width=5, height=5, dpi=100)
        self.mpltoolbar = NavigationToolbar(self.sc, self)

        self.verticalLayout.addWidget(self.mpltoolbar)
        self.verticalLayout.addWidget(self.sc)

        self.widget = QtWidgets.QWidget()
        self.widget.setLayout(self.verticalLayout)

        self.setCentralWidget(self.widget)

        # Right Dockable --------------------------------------------

        self.dockRight = QtWidgets.QDockWidget(parent=self)
        self.dockRight.setWindowTitle("Controls")
        self.dockRight.setMinimumWidth(360)
        #self.dockRight.setMinimumHeight(800)
        self.dockRightcontents = QtWidgets.QWidget()
        self.gridLayout_dockRight = QtWidgets.QVBoxLayout(self.dockRightcontents)

            # Map, Towers and UAVs Input Data Group 

        self.wloaded = AnotherWindow("Input Data loaded") # Secondary window to show that it loaded

        self.MTUgroup = QtWidgets.QGroupBox(parent=self.dockRightcontents)
        self.MTUgroup.setTitle("Map, Towers and UAVs")
        self.MTUgroup.setMinimumHeight(120)
        self.MTUgroup.setMaximumHeight(120)
        self.gridLayout_dockRight.addWidget(self.MTUgroup, 1)

        self.MTUloadButton = QtWidgets.QPushButton(parent=self.MTUgroup)
        self.MTUloadButton.setGeometry(QtCore.QRect(240, 20, 75, 24))
        self.MTUloadButton.setText("Load")
        self.MTUloadButton.clicked.connect(lambda: loadInputdata(self))

        self.baseFileBox = QtWidgets.QLineEdit(parent=self.MTUgroup)
        self.baseFileBox.setGeometry(QtCore.QRect(20, 20, 170, 22))
        self.baseFileBox.setText("./files/bases.kml")
        self.baseFileDialog = QtWidgets.QPushButton(parent=self.MTUgroup)
        self.baseFileDialog.setGeometry(QtCore.QRect(190, 20, 30, 22))
        self.baseFileDialog.setText("...")
        self.baseFileDialog.clicked.connect(lambda: fileDialog(
                                                            self,
                                                            self.baseFileBox,
                                                            'Open Bases KML file', 
                                                            'KML File (*.kml)'))

        self.towerFileBox = QtWidgets.QLineEdit(parent=self.MTUgroup)
        self.towerFileBox.setGeometry(QtCore.QRect(20, 50, 170, 22))
        self.towerFileBox.setText("./files/towers.kml")
        self.towerFileDialog = QtWidgets.QPushButton(parent=self.MTUgroup)
        self.towerFileDialog.setGeometry(QtCore.QRect(190, 50, 30, 22))
        self.towerFileDialog.setText("...")
        self.towerFileDialog.clicked.connect(lambda: fileDialog(
                                                            self,
                                                            self.towerFileBox,
                                                            'Open Towers KML file', 
                                                            'KML File (*.kml)'))

        self.uavFileBox = QtWidgets.QLineEdit(parent=self.MTUgroup)
        self.uavFileBox.setGeometry(QtCore.QRect(20, 80, 170, 22))
        self.uavFileBox.setText("./files/UAV_team.xml")
        self.uavFileDialog = QtWidgets.QPushButton(parent=self.MTUgroup)
        self.uavFileDialog.setGeometry(QtCore.QRect(190, 80, 30, 22))
        self.uavFileDialog.setText("...")
        self.uavFileDialog.clicked.connect(lambda: fileDialog(
                                                            self,
                                                            self.uavFileBox,
                                                            'Open UAV Team XML file', 
                                                            'XML File (*.xml)'))

        self.onlineCheckBox = QtWidgets.QCheckBox(parent=self.MTUgroup)
        self.onlineCheckBox.setGeometry(QtCore.QRect(230, 55, 100, 20))
        self.onlineCheckBox.setChecked(True)
        self.onlineCheckBox.setText("Online Height")
        #self.onlineCheckBox.toggled.connect(lambda: toogleTest(self))

        self.satellitalCheckBox = QtWidgets.QCheckBox(parent=self.MTUgroup)
        self.satellitalCheckBox.setGeometry(QtCore.QRect(230, 80, 100, 20))
        self.satellitalCheckBox.setChecked(False)
        self.satellitalCheckBox.setText("Satellital Img.")
        #self.onlineCheckBox.toggled.connect(lambda: toogleTest(self))

            # Weather Data Group

        self.weather = WT.Weather()

        self.ww = WeatherWindow(self)

        self.weatherGroup = QtWidgets.QGroupBox(parent=self.dockRightcontents)
        self.weatherGroup.setTitle("Weather")
        self.weatherGroup.setMinimumHeight(220)
        self.weatherGroup.setMaximumHeight(220)
        self.gridLayout_dockRight.addWidget(self.weatherGroup, 2)

        onlyDoublewin = QtGui.QDoubleValidator()
        onlyDoublewin.setRange(0, 1000)
        onlyDoubleAngle = QtGui.QDoubleValidator()
        onlyDoubleAngle.setRange(0, 360)
        onlyDoubleTemperature = QtGui.QDoubleValidator()
        onlyDoubleTemperature.setRange(-100, 100)
        onlyDoubleHumidity = QtGui.QDoubleValidator()
        onlyDoubleHumidity.setRange(0, 100)
        onlyDoublePressure = QtGui.QDoubleValidator()
        onlyDoublePressure.setRange(100000, 120000)

        self.windSpeedLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.windSpeedLabel.setText("Wind Speed (m/s)")
        self.windSpeedLabel.setGeometry(QtCore.QRect(50, 20, 120, 20))
        self.windSpeedInput = QtWidgets.QLineEdit(parent=self.weatherGroup)
        self.windSpeedInput.setText("0.0")
        self.windSpeedInput.setGeometry(QtCore.QRect(170, 20, 120, 20))
        self.windSpeedInput.setValidator(onlyDoublewin)

        self.windDirLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.windDirLabel.setText("Wind Direction (°)")
        self.windDirLabel.setGeometry(QtCore.QRect(50, 50, 120, 20))
        self.windDirInput = QtWidgets.QLineEdit(parent=self.weatherGroup)
        self.windDirInput.setText("0.0")
        self.windDirInput.setGeometry(QtCore.QRect(170, 50, 120, 20))
        self.windDirInput.setValidator(onlyDoubleAngle)

        self.temperatureLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.temperatureLabel.setText("Temperature (°C)")
        self.temperatureLabel.setGeometry(QtCore.QRect(50, 80, 120, 20))
        self.temperatureInput = QtWidgets.QLineEdit(parent=self.weatherGroup)
        self.temperatureInput.setText("20.0")
        self.temperatureInput.setGeometry(QtCore.QRect(170, 80, 120, 20))
        self.temperatureInput.setValidator(onlyDoubleTemperature)

        self.humidityLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.humidityLabel.setText("Humidity (%)")
        self.humidityLabel.setGeometry(QtCore.QRect(50, 110, 120, 20))
        self.humidityInput = QtWidgets.QLineEdit(parent=self.weatherGroup)
        self.humidityInput.setText("30.0")
        self.humidityInput.setGeometry(QtCore.QRect(170, 110, 120, 20))
        self.humidityInput.setValidator(onlyDoubleHumidity)

        self.pressureLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.pressureLabel.setText("Pressure (Pa)")
        self.pressureLabel.setGeometry(QtCore.QRect(50, 140, 120, 20))
        self.pressureInput = QtWidgets.QLineEdit(parent=self.weatherGroup)
        self.pressureInput.setText("101325.0")
        self.pressureInput.setGeometry(QtCore.QRect(170, 140, 120, 20))
        self.pressureInput.setValidator(onlyDoublePressure)

        self.onlineWeatherUpdate = QtWidgets.QPushButton(parent=self.weatherGroup)
        self.onlineWeatherUpdate.setText("Update Online")
        self.onlineWeatherUpdate.setGeometry(QtCore.QRect(110, 180, 120, 30))
        self.onlineWeatherUpdate.clicked.connect(lambda: onlineWeatherUpdate(self))

            # Wind vector matplotlib quiver reference
        self.windVectorPlot = None

        self.windSpeedInput.editingFinished.connect(lambda: manualWeatherUpdate(self))
        self.windDirInput.editingFinished.connect(lambda: manualWeatherUpdate(self))
        self.temperatureInput.editingFinished.connect(lambda: manualWeatherUpdate(self))
        self.humidityInput.editingFinished.connect(lambda: manualWeatherUpdate(self))
        self.pressureInput.editingFinished.connect(lambda: manualWeatherUpdate(self))

        self.dockRight.setWidget(self.dockRightcontents)
        self.addDockWidget(QtCore.Qt.DockWidgetArea(2), self.dockRight)

            # Planner Group ----------------------------------

        self.plannerGroup = QtWidgets.QGroupBox(parent=self.dockRightcontents)
        self.plannerGroup.setTitle("Planner")
        self.plannerGroup.setMinimumHeight(100)
        self.plannerGroup.setMaximumHeight(100)
        self.gridLayout_dockRight.addWidget(self.plannerGroup, 3)

        self.loadingPlanner = QtWidgets.QProgressBar(parent=self.plannerGroup)
        self.loadingPlanner.setGeometry(QtCore.QRect(35, 30, 300, 15))

        self.planButton = QtWidgets.QPushButton(parent=self.plannerGroup)
        self.planButton.setText("Search Optimal Routes")
        self.planButton.setGeometry(QtCore.QRect(70, 60, 200, 30))
        self.planButton.clicked.connect(lambda: exec_Planner(self))

            # Simulation -------------------------------------

        self.simGroup = QtWidgets.QGroupBox(parent=self.dockRightcontents)
        self.simGroup.setTitle("Simulation")
        self.simGroup.setMinimumHeight(100)
        self.simGroup.setMaximumHeight(100)
        self.gridLayout_dockRight.addWidget(self.simGroup, 4)

        onlyDoubleSim = QtGui.QDoubleValidator()
        onlyDoubleSim.setRange(0.1, 2.0)
        self.simSpeedLabel = QtWidgets.QLabel(parent=self.simGroup)
        self.simSpeedLabel.setText("Simulation Speed")
        self.simSpeedLabel.setGeometry(QtCore.QRect(50, 20, 120, 20))
        self.simSpeedInput = QtWidgets.QLineEdit(parent=self.simGroup)
        self.simSpeedInput.setText("1.0")
        self.simSpeedInput.setGeometry(QtCore.QRect(170, 20, 120, 20))
        self.simSpeedInput.setValidator(onlyDoubleSim)

        self.simButton = QtWidgets.QPushButton(parent=self.simGroup)
        self.simButton.setText("Simulate")
        self.simButton.setGeometry(QtCore.QRect(30, 60, 120, 30))
        self.simButton.clicked.connect(lambda: compute_sim_trajectories(self))

        self.pausesimButton = QtWidgets.QPushButton(parent=self.simGroup)
        self.pausesimButton.setText("⏵|⏸")
        self.pausesimButton.setStyleSheet('font-size: 18px;')
        self.pausesimButton.setGeometry(QtCore.QRect(180, 60, 50, 30))
        self.pausesimButton.clicked.connect(lambda: startstopSim(self))

        self.resetsimButton = QtWidgets.QPushButton(parent=self.simGroup)
        self.resetsimButton.setText("↺")
        self.resetsimButton.setStyleSheet('font-size: 18px;')
        self.resetsimButton.setGeometry(QtCore.QRect(250, 60, 50, 30))
        self.resetsimButton.clicked.connect(lambda: resetSim(self))

            # Save Mission Group
        
        self.saveGroup = QtWidgets.QGroupBox(parent=self.dockRightcontents)
        self.saveGroup.setTitle("Save Mission File")
        self.saveGroup.setMinimumHeight(100)
        self.saveGroup.setMaximumHeight(100)
        self.gridLayout_dockRight.addWidget(self.saveGroup, 5)

        self.fileLineLocation = QtWidgets.QLineEdit(parent=self.saveGroup)
        self.fileLineLocation.setText("./mission.yaml")
        self.fileLineLocation.setGeometry(QtCore.QRect(40, 30, 250, 20))

        self.saveButton = QtWidgets.QPushButton(parent=self.saveGroup)
        self.saveButton.setText("Save")
        self.saveButton.setGeometry(QtCore.QRect(70, 60, 200, 30))
        self.saveButton.clicked.connect(lambda: YAML.save_Mission(self.fileLineLocation.text(), self.uavs, self.towers.get_UTM_Zone()))

        # Left Dockable --------------------------------------

        self.dockLeft = QtWidgets.QDockWidget(parent=self)
        self.dockLeft.setWindowTitle("UAV Models")
        self.dockLeft.setMinimumWidth(275)
        self.dockLeftcontents = QtWidgets.QWidget()
        self.gridLayout_dockLeft = QtWidgets.QGridLayout(self.dockLeftcontents)

            # UAV Models Tree View

        self.treeView = QtWidgets.QTreeView(parent=self.dockLeftcontents)
        self.treeView.setHeaderHidden(True)
        self.gridLayout_dockLeft.addWidget(self.treeView, 0, 0, 1, 1)
        self.dockLeft.setWidget(self.dockLeftcontents)
        self.addDockWidget(QtCore.Qt.DockWidgetArea(1), self.dockLeft)

        # Up Dockable --------------------------------------
        
        self.dockUp = QtWidgets.QDockWidget(parent=self)
        self.dockUp.setWindowTitle("Mission Settings")
        self.dockUp.setMinimumHeight(150)
        self.dockUp.setMaximumHeight(200)
        self.dockUp.setMinimumWidth(1024)
        self.dockUpcontents = QtWidgets.QWidget()
        self.gridLayout_dockUp = QtWidgets.QGridLayout(self.dockUpcontents)

            # Mission Settings model for TableView
        
        self.avoid_loop = False
        self.tableModel = QtGui.QStandardItemModel() #missionSettingsModel() #
        self.tableModel.setHorizontalHeaderLabels(['Base',
                                          'Nav. speed [m/s]',
                                          'Insp. speed [m/s]',
                                          'Landing Mode',
                                          "Inspection Height [m]",
                                          "Insp. horizontal offset [m]",
                                          "Cam. angle [º]",
                                          "Tower distance [m]"])

            # Info Column View

        self.tableView = QtWidgets.QTableView(parent=self.dockUpcontents)

        self.combo_bases = []
        self.combo_modes = []

        self.tableView.setModel(self.tableModel)
        self.tableView.horizontalHeader().setStyleSheet("::section{Background-color:rgb(230,230,230)}")
        self.tableView.verticalHeader().setStyleSheet("::section{Background-color:rgb(230,230,230)}")

        self.gridLayout_dockUp.addWidget(self.tableView, 0, 0, 1, 1)
        self.dockUp.setWidget(self.dockUpcontents)
        self.addDockWidget(QtCore.Qt.DockWidgetArea(4), self.dockUp)

        # Set Light Theme by default
        self.setLightTheme()

        self.show()
        self.tableView.resizeColumnsToContents()


        self.sim_iter = 0
        self.simQ = False
        self.continueQ = []
        self.timer = QtCore.QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(lambda: updateSim(self))
        self.timer.start()

        return None

    def setLightTheme(self):
        """
        Sets the default theme. No arguments requiered
        """

        print("Using Light Theme")

        self.setStyleSheet("""
                            background-color:rgb(245, 245, 245);
                            color:rgb(25, 25, 25);
                           """)
        
        self.ww.setStyleSheet("""
                            background-color:rgb(245, 245, 245);
                            color:rgb(25, 25, 25);
                           """)

        self.MTUloadButton.setStyleSheet('background-color:rgb(245, 245, 245);')
        self.simButton.setStyleSheet('background-color:rgb(245, 245, 245);')
        self.planButton.setStyleSheet('background-color:rgb(245, 245, 245);')
        self.saveButton.setStyleSheet('background-color:rgb(245, 245, 245);')
        self.onlineWeatherUpdate.setStyleSheet('background-color:rgb(245, 245, 245);')

        self.widget.setStyleSheet("background-color:rgb(245, 245, 245);")
        self.baseFileBox.setStyleSheet("background-color:white;")
        self.towerFileBox.setStyleSheet("background-color:white;")
        self.uavFileBox.setStyleSheet("background-color:white;")
        self.windSpeedInput.setStyleSheet("background-color:white;")
        self.windDirInput.setStyleSheet("background-color:white;")
        self.temperatureInput.setStyleSheet("background-color:white;")
        self.humidityInput.setStyleSheet("background-color:white;")
        self.pressureInput.setStyleSheet("background-color:white;")
        self.simSpeedInput.setStyleSheet("background-color:white;")
        self.fileLineLocation.setStyleSheet("background-color:white;")
        self.treeView.setStyleSheet("background-color:white;") 
        self.tableView.setStyleSheet("background-color:white;")

        #self.sc.axes.set_facecolor('white')
        #self.sc.draw()

        return None

    def setDarkTheme(self):
        """
        Sets the dark theme. No arguments requiered
        """

        print("Using Dark Theme")

        self.setStyleSheet("""
                           background-color:rgb(5, 5, 5);
                           color:rgb(255, 255, 255);
                           selection-background-color:rgb(60, 60, 60);
                           selection-color:rgb(255,255,255);
                           alternate-background-color:rgb(255,255,255)
                           """)
        
        self.ww.setStyleSheet("""
                           background-color:rgb(5, 5, 5);
                           color:rgb(255, 255, 255);
                           selection-background-color:rgb(60, 60, 60);
                           selection-color:rgb(255,255,255);
                           alternate-background-color:rgb(255,255,255)
                           """)
        
        self.MTUloadButton.setStyleSheet('background-color:rgb(60, 60, 60);')
        self.simButton.setStyleSheet('background-color:rgb(60, 60, 60);')
        self.planButton.setStyleSheet('background-color:rgb(60, 60, 60);')
        self.saveButton.setStyleSheet('background-color:rgb(60, 60, 60);')
        self.onlineWeatherUpdate.setStyleSheet('background-color:rgb(60, 60, 60);')
        
        #self.widget.setStyleSheet("background-color:rgb(5, 5, 5);")
        self.baseFileBox.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.towerFileBox.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.uavFileBox.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.windSpeedInput.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.windDirInput.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.temperatureInput.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.humidityInput.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.pressureInput.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.simSpeedInput.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.fileLineLocation.setStyleSheet("background-color:rgb(60, 60, 60);")
        self.treeView.setStyleSheet("""
                           background-color:rgb(60, 60, 60);
                           color:rgb(255, 255, 255);
                           selection-background-color:rgb(60, 60, 60);
                           selection-color:rgb(255,255,255);
                           alternate-background-color:rgb(255,255,255)
                           """) 
        self.tableView.setStyleSheet("""
                           background-color:rgb(60, 60, 60);
                           color:rgb(240, 240, 240);
                           selection-background-color:rgb(60, 60, 60);
                           selection-color:rgb(30, 30, 30);
                           alternate-background-color:rgb(30,30,30)
                           """)
        
        
        #self.sc.axes.set_facecolor('black')
        #self.sc.draw()

        return None

    def changeSolverType(self, type1: str):
            """
            Auxiliary method to change the type of solver from the UI using the menubar
            """
            self.solverType = type1
            print('Solver changed to: ', type1)

            return None


# -------------------------------------------------------------------------------------------
#
#                  Auxiliary UI functions and classes. Not needed for Non-UI working
#
# -------------------------------------------------------------------------------------------
            
# ------------------------------ Generic secondary UI Elements ------------------------------
            
class AnotherWindow(QtWidgets.QWidget):
    def __init__(self, text: str):
        """
        Secondary Window class. Used for error messages or general info pop ups
        """
        super().__init__()
        self.setWindowTitle(" ")
        layout = QtWidgets.QVBoxLayout()
        self.label = QtWidgets.QLabel(text)
        self.label.setMinimumSize(150,30)
        layout.addWidget(self.label)
        self.setLayout(layout)

        return None

class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent = None, width=5, height=4, dpi=100):
        """
        Matplotlib canvas used for PyQT6. It stores every graphics from the central widget
        """
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        #self.fig.tight_layout()
        self.axes.set_xlabel("E coordinate")
        self.axes.set_ylabel("N coordinate")

        # Super allows you to call functions or method from the class where the current one is derived
        super(MplCanvas, self).__init__(self.fig)

        return None

# Editable and non editable UI items for TreeView and TableView
class noneditableItem(QtGui.QStandardItem):
    def __init__(self, txt = ' ', font_size = 12, set_bold = False, color = QtGui.QColor(0,0,0)):
        """
        Non-editable item generally used within TableView or TreeView
        """
        super().__init__()

        fnt = QtGui.QFont('Arial',font_size)
        fnt.setBold(set_bold)

        self.setEditable(False)
        self.setForeground(color)
        self.setFont(fnt)
        self.setText(txt)

        return None

class editableItem(QtGui.QStandardItem):
    def __init__(self, txt = ' ', font_size = 12, set_bold = False, color = QtGui.QColor(0,0,0)):
        """
        Editable item generally used within TableView or TreeView
        """
        super().__init__()

        fnt = QtGui.QFont('Arial',font_size)
        fnt.setBold(set_bold)

        self.setEditable(True)
        self.setForeground(color)
        self.setFont(fnt)
        self.setText(txt)

        return None

# ------------------------------ File Loading -----------------------------------------------
                       
def fileDialog(ui: MainWindow, fileLine: QtWidgets.QLineEdit, dialog: str, filetype:str):
    """
    Opens a file loading dialog for a specific filetye with some text for user context.
    """

    # Using Qt preexisting FileDialog
    filename = QtWidgets.QFileDialog.getOpenFileName(
        ui,
        dialog,
        '',
        filetype, 
    )

    # Check if result is not empty
    if filename[0] != '':
        fileLine.setText(filename[0])

    return None

def load_data_from_YAML(ui: MainWindow):
    """
    Auxiliary UI function to load everything needed to define a GTSP-MUAV problem (Towers, Bases & UAV Team) except from weather.
    It uses the menubar item to do so.
    """

    filename = QtWidgets.QFileDialog.getOpenFileName(
        ui,
        "Open Mission Initialization YAML",
        '',
        "Mission Initialization (*.yaml)", 
    )

    ui.bases, ui.towers, ui.uavs, _ = YAML.load_data_from_YAML(filename[0])

    # Update with new data
    updatePlot(ui.sc, ui.bases, ui.towers, ui.satellitalCheckBox.isChecked())

    # Set general location as the first base. This could be better done
    base1 = ui.bases.get_Base("B1")
    ui.location_LatLon = CO.utm2latlon(base1.get_Coordinates(), base1.get_UTM_Zone()) # [Latitude, Longitud]


    # UAVs Model Info Update to treeview
    updateUAVsModelData(ui.treeView)

    # Update TableView with models

    UAVs_IDs = []

    for uav in ui.uavs:
        UAVs_IDs.append(uav.get_ID())

    ui.tableModel.setVerticalHeaderLabels(UAVs_IDs)


    ui.combo_modes = []
    ui.combo_bases = []

        
    keys = list(uav.missionSettings.keys())

    bases_list = [uav.missionSettings["Base"] for uav in ui.uavs]
    landing_list = ["None", "Auto"]

    for i in range(len(UAVs_IDs)):

        b_index = bases_list.index(ui.uavs.select_UAV_by_Order(i).missionSettings["Base"])
        lm_index = landing_list.index(ui.uavs.select_UAV_by_Order(i).missionSettings["Landing Mode"])

        for k in range(ui.tableModel.columnCount()):
            ui.tableModel.setData(ui.tableModel.index(i, k), ui.uavs.select_UAV_by_Order(i).missionSettings[keys[k]],
                                    QtCore.Qt.ItemDataRole.EditRole)

        ui.combo_bases.append(QtWidgets.QComboBox())
        ui.combo_bases[i].addItems(bases_list)
        ui.combo_bases[i].setCurrentIndex(b_index)

        ui.combo_modes.append(QtWidgets.QComboBox())
        ui.combo_modes[i].addItems(landing_list)
        ui.combo_modes[i].setCurrentIndex(lm_index)
    
        ui.tableView.setIndexWidget(ui.tableModel.index(i, 0), ui.combo_bases[i])
        ui.tableView.setIndexWidget(ui.tableModel.index(i, 3), ui.combo_modes[i])
        
        ui.combo_bases[i].currentTextChanged.connect(lambda: getBaseMissionSettings(ui))
        ui.combo_modes[i].currentTextChanged.connect(lambda: getLandingModeMissionSettings(ui))

    # data changed or itemchanged do not detect combo changes
    ui.tableModel.itemChanged.connect(lambda item: updateMissionSettings(ui, item))
    ui.tableView.resizeColumnsToContents()
        
    # Set Flag Status
    ui.status_flag = set_bit(ui.status_flag, 1, 1)
    print(bin(ui.status_flag))

    print("Bases, Towers and UAV files loaded and drawn")

    return None

def loadInputdata(ui: MainWindow):
    """
    Auxiliary UI function to load everything needed to define a GTSP-MUAV problem (Towers, Bases & UAV Team) except from weather.
    It uses the file dialogs from the UI to locate the right files and show their information to the user
    """
    try:

        # Load each file using the corresponding method to the UI-stored instance
        ui.towers.load_File(ui.towerFileBox.text(), ui.onlineCheckBox.isChecked())
        ui.bases.load_File(ui.baseFileBox.text(), ui.onlineCheckBox.isChecked())
        ui.uavs.load_File(ui.uavFileBox.text())
        
        # Update with new data
        updatePlot(ui.sc, ui.bases, ui.towers, ui.satellitalCheckBox.isChecked())

        # Set general location as the first base. This could be better done
        base1 = ui.bases.get_Base("B1")
        ui.location_LatLon = CO.utm2latlon(base1.get_Coordinates(), base1.get_UTM_Zone()) # [Latitude, Longitud]


        # UAVs Model Info Update to treeview
        updateUAVsData(ui.treeView, ui.uavs)

        # Update TableView with models

        UAVs_IDs = []

        for uav in ui.uavs:
            UAVs_IDs.append(uav.get_ID())

        ui.tableModel.setVerticalHeaderLabels(UAVs_IDs)

        #default_values = ['', 15, 5, '', 5, 0, 90, 5]

        ui.combo_modes = []
        ui.combo_bases = []

        
        keys = list(uav.missionSettings.keys())

        for i in range(len(UAVs_IDs)):

            for k in range(ui.tableModel.columnCount()):
                ui.tableModel.setData(ui.tableModel.index(i, k), ui.uavs.select_UAV_by_Order(i).missionSettings[keys[k]],
                                    QtCore.Qt.ItemDataRole.EditRole)

            ui.combo_bases.append(QtWidgets.QComboBox())
            ui.combo_bases[i].addItems([base.get_Name() for base in ui.bases])

            ui.combo_modes.append(QtWidgets.QComboBox())
            ui.combo_modes[i].addItems(["None", "Auto"])
        
            ui.tableView.setIndexWidget(ui.tableModel.index(i, 0), ui.combo_bases[i])
            ui.tableView.setIndexWidget(ui.tableModel.index(i, 3), ui.combo_modes[i])
        
            ui.combo_bases[i].currentTextChanged.connect(lambda: getBaseMissionSettings(ui))
            ui.combo_modes[i].currentTextChanged.connect(lambda: getLandingModeMissionSettings(ui))

        # data changed or itemchanged do not detect combo changes
        ui.tableModel.itemChanged.connect(lambda item: updateMissionSettings(ui, item))
        ui.tableView.resizeColumnsToContents()
        
        # Set Flag Status
        ui.status_flag = set_bit(ui.status_flag, 1, True)
        print(bin(ui.status_flag))

        print("Bases, Towers and UAV files loaded and drawn")
        #ui.wloaded.show()
        
    except:
        
        # In case something fails, show error
        print("loadInputdata ERROR")

    return None

# ------------------------------ Simulation specific ----------------------------------------

def compute_sim_trajectories(ui: MainWindow):
    """
    Compute the trajectiories for UI simulator animation. It uses the physiscal waypoints of each UAV to do so.
    """

    # Add some condition to not compute the trajectories if they already exits

    if not(is_set(ui.status_flag, 3)):
        print('No Solution found yet.')
        return None
    
    if not(is_set(ui.status_flag, 5)):
        print('Waypoints need to be computed')
        ui.uavs.compute_Team_Waypoints("PaV", ui.towers, ui.bases)

    
    ui.continueQ = []

    sim_speed = float(ui.simSpeedInput.text())
    dt = 1*sim_speed # this might need adjustment

    for uav in ui.uavs:

        firstQ = True
        k = 0

        points = uav.waypoints.get_Points_List()[1:-1]
        actions = uav.waypoints.get_Actions_List()[1:-1]
        modes = uav.waypoints.get_Modes_List()[1:-1]

        print(points)

        angles = []
        [angles.append(action['yaw']) for action in actions]

        for k in range(len(points)-1):

            if 'Inspection' == modes[k]:  # TO BE FINISHED
                uav_speed = uav.missionSettings['Insp. speed']
            else: uav_speed = uav.missionSettings['Nav. speed']

            if firstQ:
                sim_trajectory = SIM.get_Path_Points(points[k][0:2], points[k+1][0:2], uav_speed, dt)
                sim_angles =  np.asarray([angles[k]] * len(sim_trajectory))
                firstQ = False
            else:
                temp = SIM.get_Path_Points(points[k][0:2], points[k+1][0:2], uav_speed, dt)
                sim_trajectory = np.concatenate((sim_trajectory, temp))
                sim_angles =  np.concatenate((sim_angles, np.asarray([angles[k]] * len(temp))))
            k += 1

        uav.sim_trajectory = copy.deepcopy(sim_trajectory)
        uav.sim_angles = copy.deepcopy(sim_angles)
        ui.continueQ.append(False)

        
        """
        for path in uav.routeUTM:

            if uav.routeModes[k] == 'Inspection':
                uav_speed = uav.missionSettings['Insp. speed']
            else: uav_speed = uav.missionSettings['Nav. speed']

            if firstQ:
                sim_trajectory = SIM.get_path_points(path[0][0:2], path[1][0:2], uav_speed, dt)
                firstQ = False
            else:
                sim_trajectory = np.concatenate((sim_trajectory, SIM.get_path_points(path[0][0:2], path[1][0:2], uav_speed, dt)))
            k += 1

        uav.sim_trajectory = copy.deepcopy(sim_trajectory)
        ui.continueQ.append(False)
        """

    ui.sim_iter = 0
    ui.status_flag = set_bit(ui.status_flag, 4, 1)
    print(bin(ui.status_flag))

    print('Trajectories simulated')
    return None

def startstopSim(ui: MainWindow):
    """
    Each this function is called, it triggers a stop or a start of the simulation animation.
    """

    if not is_set(ui.status_flag, 4):
        return None
    
    ui.simQ = not ui.simQ

    if all(ui.continueQ):
            ui.sim_iter = 0

    return None
            
def resetSim(ui: MainWindow):
    """
    Restarts the animation to the first frame and starts playing it again.
    """
    ui.sim_iter = 0
    ui.simQ = True

    return None

def updateSim(ui: MainWindow):
    """
    Each time the function is called, it updates the animation to next frame
    """

    if ui.simQ:

        colorlist = ['red', 'blue', 'green', 'cyan', 'brown']

        k = 0
        for uav in ui.uavs:
            if ui.sim_iter < len(uav.sim_trajectory):

                point = uav.sim_trajectory[ui.sim_iter, :]
                angle = uav.sim_angles[ui.sim_iter]

                if uav.sim_plot == None:
                    uav.sim_plot, = ui.sc.axes.plot(point[0], point[1], color = colorlist[k], marker = SIM.uav_Marker(angle), markersize = 25)

                else:
                    uav.sim_plot.set_xdata(point[0])
                    uav.sim_plot.set_ydata(point[1])
                    uav.sim_plot.set_marker(SIM.uav_Marker(angle))

                ui.continueQ[k] = False

            else:
                ui.continueQ[k] = True

            k += 1

        if all(ui.continueQ):
            ui.simQ = False
            return None

        ui.sc.draw()
        ui.sim_iter += 1

# ------------------------------ Weather specific ----------------------------------------

class MplWeatherCanvas(FigureCanvasQTAgg):
    def __init__(self, parent = None, width=2, height=2, dpi=300):
        """
        Matplotlib canvas used for PyQT6. It stores every graphics for weather visualization
        """
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.fig.patch.set_facecolor([245/255, 245/255, 245/255])
        self.axes = self.fig.add_subplot(121, projection = 'polar')
        self.axes2 = self.fig.add_subplot(122)
        self.axes2.axis('off')

        self.axes.set_title("Wind Speed [km/h]")
        self.axes.set_theta_zero_location("N")
        self.axes.set_theta_direction(-1)
        self.axes.set_rmin(0.0)

        self.arrow = self.axes.arrow(45/180.*np.pi, 0.0, 0, 1, width = 0.05,
                 edgecolor = 'black', facecolor = 'green', length_includes_head=True)
        
        self.cb = self.axes2.figure.colorbar(mpl.cm.ScalarMappable(norm=mpl.colors.Normalize(-20, 50), cmap='jet'),
                               ax = self.axes2, orientation='vertical', label='Temperature [°]', fraction=0.046, aspect = 5)

        self.fig.add_axes(self.cb.ax)
        
        #self.cb2 = self.fig.colorbar(mpl.cm.ScalarMappable(norm=mpl.colors.Normalize(0.9, 1.1), cmap='rainbow'),
        #     ax=self.axes2, orientation='vertical', label='Pressure [atm]', fraction=0.046, aspect = 5)

        # Super allows you to call functions or method from the class where the current one is derived
        super(MplWeatherCanvas, self).__init__(self.fig)

    def plotWeather(self, weather: WT.Weather):
        """
        Plots the input weather to the associated canvas
        """

        angle = np.arccos(weather.wind_vector[1])
        if weather.wind_vector[0] < 0.0: angle + np.pi
        length = np.linalg.norm(weather.wind_vector)  * 3.6 # [km / h]

        self.arrow.set_data(x = angle * 180.0 / np.pi, y = 0.0, dx = 0.0, dy = length)
        self.cb.ax.axhline(weather.temperature, c = "k")
        #self.cb2.ax.axhline(weather.pressure / 101325, c = "k")
        self.axes.set_rmin(0.0)
        self.axes.set_rmax(1.1 * length)

class WeatherWindow(QtWidgets.QWidget):
    def __init__(self, ui: MainWindow):
        """
        Secondary Window class used for weather manual edition and visualization
        """
        super().__init__()
        self.setWindowTitle("Weather editor")
        layout = QtWidgets.QHBoxLayout()

        self.weatherGroup = QtWidgets.QGroupBox()
        self.weatherGroup.setTitle("Weather")
        self.weatherGroup.setMinimumHeight(220)
        self.weatherGroup.setMaximumHeight(220)
        self.weatherGroup.setMinimumWidth(350)
        self.weatherGroup.setMaximumWidth(350)

        onlyDoublewin = QtGui.QDoubleValidator()
        onlyDoublewin.setRange(0, 1000)
        onlyDoubleAngle = QtGui.QDoubleValidator()
        onlyDoubleAngle.setRange(0, 360)
        onlyDoubleTemperature = QtGui.QDoubleValidator()
        onlyDoubleTemperature.setRange(-100, 100)
        onlyDoubleHumidity = QtGui.QDoubleValidator()
        onlyDoubleHumidity.setRange(0, 100)
        onlyDoublePressure = QtGui.QDoubleValidator()
        onlyDoublePressure.setRange(100000, 120000)

        self.windSpeedLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.windSpeedLabel.setText("Wind Speed [m/s]")
        self.windSpeedLabel.setGeometry(QtCore.QRect(50, 20, 120, 20))
        self.windSpeedInput = QtWidgets.QLineEdit(parent=self.weatherGroup)
        self.windSpeedInput.setText("0.0")
        self.windSpeedInput.setGeometry(QtCore.QRect(170, 20, 120, 20))
        self.windSpeedInput.setValidator(onlyDoublewin)

        self.windDirLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.windDirLabel.setText("Wind Direction [°]")
        self.windDirLabel.setGeometry(QtCore.QRect(50, 50, 120, 20))
        self.windDirInput = QtWidgets.QLineEdit(parent=self.weatherGroup)
        self.windDirInput.setText("0.0")
        self.windDirInput.setGeometry(QtCore.QRect(170, 50, 120, 20))
        self.windDirInput.setValidator(onlyDoubleAngle)

        self.temperatureLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.temperatureLabel.setText("Temperature [°C]")
        self.temperatureLabel.setGeometry(QtCore.QRect(50, 80, 120, 20))
        self.temperatureInput = QtWidgets.QLineEdit(self.weatherGroup)
        self.temperatureInput.setText("20.0")
        self.temperatureInput.setGeometry(QtCore.QRect(170, 80, 120, 20))
        self.temperatureInput.setValidator(onlyDoubleTemperature)

        self.humidityLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.humidityLabel.setText("Humidity [%]")
        self.humidityLabel.setGeometry(QtCore.QRect(50, 110, 120, 20))
        self.humidityInput = QtWidgets.QLineEdit(parent=self.weatherGroup)
        self.humidityInput.setText("30.0")
        self.humidityInput.setGeometry(QtCore.QRect(170, 110, 120, 20))
        self.humidityInput.setValidator(onlyDoubleHumidity)

        self.pressureLabel = QtWidgets.QLabel(parent=self.weatherGroup)
        self.pressureLabel.setText("Pressure [Pa]")
        self.pressureLabel.setGeometry(QtCore.QRect(50, 140, 120, 20))
        self.pressureInput = QtWidgets.QLineEdit(parent=self.weatherGroup)
        self.pressureInput.setText("101325.0")
        self.pressureInput.setGeometry(QtCore.QRect(170, 140, 120, 20))
        self.pressureInput.setValidator(onlyDoublePressure)

        
        saveWeatherUpdate = QtWidgets.QPushButton(parent=self.weatherGroup)
        saveWeatherUpdate.setText("Save")
        saveWeatherUpdate.setGeometry(QtCore.QRect(110, 180, 120, 30))
        saveWeatherUpdate.clicked.connect(lambda: self.windowWeatherUpdate(ui))    

        layout.addWidget(self.weatherGroup)


        # Matplotlib canvas 
        
        self.sc = MplWeatherCanvas(self, width=2, height=2, dpi=100)
        self.sc.setMinimumHeight(220)
        self.sc.setMaximumHeight(220)
        self.sc.setMinimumWidth(350)
        self.sc.setMaximumWidth(350)
        layout.addWidget(self.sc)

        self.setLayout(layout)

        self.setMinimumHeight(27)
        self.setMaximumHeight(270)
        self.setMinimumWidth(700)
        self.setMaximumWidth(700)

    def windowWeatherUpdate(self, ui: MainWindow):
        """
        Updates the weather tags and the Main Windows UI from the Weather Editor Secondary Window.
        """

        # Update internal weather instance
        alpha = float(self.windDirInput.text())
        ui.weather.wind_vector = float(self.windSpeedInput.text())*np.array([np.sin(alpha/180*np.pi), np.cos(alpha/180*np.pi), 0]) # [m/s]
        ui.weather.temperature = float(self.temperatureInput.text());                 # [Celsius]
        ui.weather.humidity = float(self.humidityInput.text());                       # [%]
        ui.weather.pressure = float(self.pressureInput.text())*100;                   # [Pa]

        # Update MainWindow Weather Status.

        x = 0.8*(ui.sc.axes.get_xticks()[-1]  - ui.sc.axes.get_xticks()[0]) + ui.sc.axes.get_xticks()[0]
        y = 0.2*(ui.sc.axes.get_yticks()[-1]  -ui.sc.axes.get_yticks()[0]) + ui.sc.axes.get_yticks()[0]

        if ui.windVectorPlot == None: 
            ui.windVectorPlot = ui.sc.axes.quiver(x, y, ui.weather.wind_vector[0], ui.weather.wind_vector[1])
            ui.sc.axes.text(x - 40, y - 35, 'Wind vector')
        else: 
            ui.windVectorPlot.set_UVC(ui.weather.wind_vector[0], ui.weather.wind_vector[1])
  
        ui.sc.draw()

        # Set Flag Status
        ui.status_flag = set_bit(ui.status_flag, 2, 1)
        print(bin(ui.status_flag))

        # this WILL BE CHAMGED AS IT IS REDUDANT
        ui.windDirInput.setText(self.windDirInput.text())
        ui.windSpeedInput.setText(self.windSpeedInput.text())
        ui.temperatureInput.setText(self.temperatureInput.text())
        ui.humidityInput.setText(self.humidityInput.text())
        ui.pressureInput.setText(self.pressureInput.text())

        self.close()

def get_weatherWindow(ui: MainWindow):
    """
    Spawns the Weather Editor Window.
    """
    ui.ww.sc.plotWeather(ui.weather)
    ui.ww.show()   

def manualWeatherUpdate(ui: MainWindow):
    """
    Gets manually edited weather conditions from the UI and updates the internal weather variables
    """

    alpha = float(ui.windDirInput.text())

    ui.weather.set_Wind_Vector(float(ui.windSpeedInput.text())*np.array([np.sin(alpha/180*np.pi), np.cos(alpha/180*np.pi), 0])) # [m/s]
    ui.weather.set_Temperature(float(ui.temperatureInput.text()))                # [Celsius]
    ui.weather.set_Humidity(float(ui.humidityInput.text()))                     # [%]
    ui.weather.set_Pressure(float(ui.pressureInput.text())*100)                   # [Pa]

    x = 0.8*(ui.sc.axes.get_xticks()[-1]  - ui.sc.axes.get_xticks()[0]) + ui.sc.axes.get_xticks()[0]
    y = 0.2*(ui.sc.axes.get_yticks()[-1]  -ui.sc.axes.get_yticks()[0]) + ui.sc.axes.get_yticks()[0]

    if ui.windVectorPlot == None: 
        ui.windVectorPlot = ui.sc.axes.quiver(x, y, ui.weather.get_Wind_Vector()[0], ui.weather.get_Wind_Vector()[1])
        ui.sc.axes.text(x - 40, y - 35, 'Wind vector')
    else: 
        ui.windVectorPlot.set_UVC(ui.weather.get_Wind_Vector()[0], ui.weather.get_Wind_Vector()[1])
  
    ui.sc.draw()

    # Set Flag Status
    ui.status_flag = set_bit(ui.status_flag, 2, 1)
    print(bin(ui.status_flag))

def onlineWeatherUpdate(ui: MainWindow):
    """
    Get online weather conditions from a web API and updates the internal weather variables
    """
    # Location must be given as [Latitude, Longitud]

    q = ui.weather.update_Online(ui.location_LatLon)

    if q:

        speed = np.linalg.norm(ui.weather.get_Wind_Vector())
        alpha = np.rad2deg(np.arccos(ui.weather.get_Wind_Vector()[1] / speed))
        if ui.weather.get_Wind_Vector()[0] < 0: alpha = 360-alpha
        
        ui.windSpeedInput.setText(str(speed))
        ui.windDirInput.setText(str(alpha))
        ui.temperatureInput.setText(str(ui.weather.get_Temperature()))
        ui.humidityInput.setText(str(ui.weather.get_Humidity()))
        ui.pressureInput.setText(str(ui.weather.get_Pressure()))

        x = 0.8*(ui.sc.axes.get_xticks()[-1]  - ui.sc.axes.get_xticks()[0]) + ui.sc.axes.get_xticks()[0]
        y = 0.2*(ui.sc.axes.get_yticks()[-1]  -ui.sc.axes.get_yticks()[0]) + ui.sc.axes.get_yticks()[0]

        if ui.windVectorPlot == None: 
            ui.windVectorPlot = ui.sc.axes.quiver(x, y, ui.weather.get_Wind_Vector()[0], ui.weather.get_Wind_Vector()[1])
            ui.sc.axes.text(x - 40, y - 35, 'Wind vector')
        else: 
            ui.windVectorPlot.set_UVC(ui.weather.get_Wind_Vector()[0], ui.weather.get_Wind_Vector()[1])
        
        ui.sc.draw()

        # Set Flag Status
        ui.status_flag = set_bit(ui.status_flag, 2, 1)
        print(bin(ui.status_flag))

# ------------------------------ UAV specific --------------------------------------------

def updateUAVsData(treeView: QtWidgets.QTreeView, uavs:UAVS.UAV_Team):
    """
    Prints the UAV Team data to the TreeView on the UI
    """

    # Creates a new model for the tree view
    treeModel = QtGui.QStandardItemModel()

    # root node to which we append all the UAVs as columns
    rootNode = treeModel.invisibleRootItem()

    model = []
    
    k = 0
    for uav in uavs:
        # Append UAV to the list
        model.append(noneditableItem(uav.get_Name()+' ID: '+uav.get_ID(), 16, set_bold = True))

        # Add as column of the newly added UAV each of the parameters using items

        bat = noneditableItem('Battery', 12, set_bold = False)
        bat.appendColumn([
            noneditableItem('Type: '+uav.get_Battery().type, 12, set_bold = False),
            noneditableItem(f'Capacity: {uav.get_Battery().capacity}', 12, set_bold = False),
            noneditableItem(f'Number of cells: {uav.get_Battery().cells}', 12, set_bold = False),
            noneditableItem(f'Volts per cell: {uav.get_Battery().volts_per_cell}', 12, set_bold = False),
        ])

        data = uav.get_UAV_Parameters()

        model[k].appendColumn([noneditableItem(f'Mass [kg]: {data["mass"]}', 12, set_bold = False),
                               noneditableItem(f'Number of rotors: {data["number_of_Rotors"]}', 12, set_bold = False),
                               noneditableItem(f'Blades per rotor: {data["rotor_Blades"]}', 12, set_bold = False),
                               noneditableItem(f'Rotor radius [m]: {data["rotor_Radius"]}', 12, set_bold = False),
                               noneditableItem(f'Blade chord [m]: {data["blade_Chord"]}', 12, set_bold = False),
                               noneditableItem(f'Lift Coefficient: {data["lift_Coefficient"]}', 12, set_bold = False),
                               noneditableItem(f'Drag Coefficient: {data["drag_Coefficient"]}', 12, set_bold = False),
                               noneditableItem(f'kappa: {data["kappa"]}', 12, set_bold = False),
                               noneditableItem(f'eta: {data["eta"]}', 12, set_bold = False),
                               noneditableItem(f'K_mu: {data["K_mu"]}', 12, set_bold = False),
                               noneditableItem(f'Effective flat area [m²]: {data["flat_Area"]}', 12, set_bold = False),
                               noneditableItem(f'CameraQ: {data["camera"]}', 12, set_bold = False),
                               bat
                               ])
        
        k += 1
    
    # Append list to root and set model
    rootNode.appendColumn(model)
    treeView.setModel(treeModel)

def updateUAVsModelData(treeView: QtWidgets.QTreeView):
    """
    Prints the available UAV Models data to the TreeView on the UI
    """

    # Creates a new model for the tree view
    treeModel = QtGui.QStandardItemModel()

    # root node to which we append all the UAVs as columns
    rootNode = treeModel.invisibleRootItem()

    # Loads models database
    f = open("./files/devices.yaml", "r")
    database = yaml.load(f, Loader = yaml.Loader)
    f.close()

    

    model = []
    
    k = 0
    for model_name in database:
        # Append UAV to the list
        model.append(noneditableItem(model_name, 16, set_bold = True))

        model_data = database[model_name]

        # Add as column of the newly added UAV each of the parameters using items

        bat = noneditableItem('Battery', 12, set_bold = False)
        bat.appendColumn([
            noneditableItem('Type: '+"Cellular", 12, set_bold = False),
            noneditableItem(f'Capacity: {model_data["battery"]["capacity"]}', 12, set_bold = False),
            noneditableItem(f'Number of cells: {model_data["battery"]["number_of_cells"]}', 12, set_bold = False),
            noneditableItem(f'Volts per cell: {model_data["battery"]["voltage_per_cell"]}', 12, set_bold = False),
        ])

        model[k].appendColumn([noneditableItem(f'Mass [kg]: {model_data["mass"]}', 12, set_bold = False),
                               noneditableItem(f'Number of rotors: {model_data["number_of_rotors"]}', 12, set_bold = False),
                               noneditableItem(f'Blades per rotor: {model_data["rotor_blades"]}', 12, set_bold = False),
                               noneditableItem(f'Rotor radius [m]: {model_data["rotor_radius"]}', 12, set_bold = False),
                               noneditableItem(f'Blade chord [m]: {model_data["blade_chord"]}', 12, set_bold = False),
                               noneditableItem(f'Lift Coefficient: {model_data["lift_coefficient"]}', 12, set_bold = False),
                               noneditableItem(f'Drag Coefficient: {model_data["draft_coefficient"]}', 12, set_bold = False),
                               noneditableItem(f'kappa: {model_data["induced_power_factor"]}', 12, set_bold = False),
                               noneditableItem(f'eta: {model_data["energy_efficiency"]}', 12, set_bold = False),
                               noneditableItem(f'K_mu: {model_data["P0_numerical_constant"]}', 12, set_bold = False),
                               noneditableItem(f'Effective flat area [m²]: {model_data["equivalent_flat_plate_area"]}', 12, set_bold = False),
                               noneditableItem(f'CameraQ: {True}', 12, set_bold = False),
                               bat
                               ])
        
        k += 1
    
    # Append list to root and set model
    rootNode.appendColumn(model)
    treeView.setModel(treeModel)

def getMissionSettings(ui: MainWindow):
    """
    It updates the UI varibles that stores the settings from the UI.
    """

    for row in range(ui.tableModel.rowCount()):

        uav = ui.uavs.select_UAV_by_Order(row)

        uav.missionSettings = {}

        uav.missionSettings['Base'] = str(ui.combo_bases[row].currentText())
        uav.missionSettings['Nav. speed'] = float(ui.tableModel.data(ui.tableModel.index(row, 1)))
        uav.missionSettings['Insp. speed'] = float(ui.tableModel.data(ui.tableModel.index(row, 2)))
        uav.missionSettings['Landing Mode'] = str(ui.combo_modes[row].currentText())
        uav.missionSettings['Insp. height'] = float(ui.tableModel.data(ui.tableModel.index(row, 4)))
        uav.missionSettings['Insp. horizontal offset'] = float(ui.tableModel.data(ui.tableModel.index(row, 5)))
        uav.missionSettings['Cam. angle'] = float(ui.tableModel.data(ui.tableModel.index(row, 6)))
        uav.missionSettings['Tower distance'] = float(ui.tableModel.data(ui.tableModel.index(row, 7)))


    ui.tableView.resizeColumnsToContents()

    for uav in ui.uavs:
        print(uav.missionSettings)

    return None

def getBaseMissionSettings(ui: MainWindow):
    """
    Instead of getting all settings from the UI, it just extracts the base. It is done separately because a UI Item Combo
    is used instead of regular text items.
    """

    for row in range(ui.tableModel.rowCount()):
        ui.uavs.select_UAV_by_Order(row).missionSettings['Base'] = str(ui.combo_bases[row].currentText())


    ui.tableView.resizeColumnsToContents()

    for uav in ui.uavs:
        print(uav.missionSettings)

    return None

def getLandingModeMissionSettings(ui: MainWindow):
    """
    Instead of getting all settings from the UI, it just extracts the landing mode. It is done separately because a UI Item Combo
    is used instead of regular text items.
    """

    for row in range(ui.tableModel.rowCount()):
        ui.uavs.select_UAV_by_Order(row).missionSettings['Landing Mode'] = str(ui.combo_modes[row].currentText())


    ui.tableView.resizeColumnsToContents()

    for uav in ui.uavs:
        print(uav.missionSettings)

    return None

def updateMissionSettings(ui: MainWindow, item: QtWidgets.QTableWidgetItem):

    
    ui.uavs.compute_Team_Waypoints("PaV", ui.towers, ui.bases)

    print(item.column())

    row = item.row()

    # scuffed type of thing
    if ui.avoid_loop == True:
        ui.avoid_loop = False
        return None
    
    uav = ui.uavs.select_UAV_by_Order(row)

    match item.column():     
            
        case 4 | 5:


            uav.missionSettings['Insp. height'] = float(ui.tableModel.data(ui.tableModel.index(row, 4)))
            uav.missionSettings['Insp. horizontal offset'] = float(ui.tableModel.data(ui.tableModel.index(row, 5)))

            uav.missionSettings['Tower distance'] = float(np.sqrt(
                uav.missionSettings['Insp. height']**2 + uav.missionSettings['Insp. horizontal offset']**2))
            temp = uav.missionSettings['Insp. horizontal offset']
            if temp == 0:
                uav.missionSettings['Cam. angle'] = 90
            else:
                uav.missionSettings['Cam. angle'] = float(np.rad2deg(np.arctan(
                    uav.missionSettings['Insp. height'] / temp)))
                

            ui.avoid_loop = True
            ui.tableModel.setData(ui.tableModel.index(row, 6),
                                    uav.missionSettings['Cam. angle'],
                                    QtCore.Qt.ItemDataRole.EditRole)

            ui.avoid_loop = True
            ui.tableModel.setData(ui.tableModel.index(row, 7),
                                    uav.missionSettings['Tower distance'],
                                    QtCore.Qt.ItemDataRole.EditRole)
                

        case 6 | 7:

            uav.missionSettings['Cam. angle'] = float(ui.tableModel.data(ui.tableModel.index(row, 6)))
            uav.missionSettings['Tower distance'] = float(ui.tableModel.data(ui.tableModel.index(row, 7)))

            uav.missionSettings['Insp. height'] = float(uav.missionSettings['Tower distance'] * 
                                                                      np.sin(np.deg2rad(uav.missionSettings['Cam. angle'])))
            uav.missionSettings['Insp. horizontal offset'] = float(uav.missionSettings['Tower distance'] * 
                                                                      np.cos(np.deg2rad(uav.missionSettings['Cam. angle'])))
            
            ui.avoid_loop = True
            ui.tableModel.setData(ui.tableModel.index(row, 4),
                                    uav.missionSettings['Insp. height'],
                                    QtCore.Qt.ItemDataRole.EditRole)

            ui.avoid_loop = True
            ui.tableModel.setData(ui.tableModel.index(row, 5),
                                    uav.missionSettings['Insp. horizontal offset'],
                                    QtCore.Qt.ItemDataRole.EditRole)

        case _: 
            uav.missionSettings['Nav. speed'] = float(ui.tableModel.data(ui.tableModel.index(row, 1)))
            uav.missionSettings['Insp. speed'] = float(ui.tableModel.data(ui.tableModel.index(row, 2)))


    ui.tableView.resizeColumnsToContents()

    for uav in ui.uavs:
        print(uav.missionSettings)

    ui.status_flag = set_bit(ui.status_flag, 5, 1)
    print("Mission Settings changed. Waypoints might need to be recomputed")

# ------------------------------ Plotting specific ----------------------------------------

def updatePlot(canvas: MplCanvas, bases: BA.Bases, towers: TW.Towers, satellitalImgQ: bool):
    """
    Updates a given Matplotlib canvas with the data from bases and towers
    """

    # If no towers are in the list, do nothing
    if not(towers.get_Graph().nodes):
        return
    
    # Clear and redraw is slow but useful
    canvas.axes.cla()

    towers.plot(canvas.axes)
    bases.plot(canvas.axes)

    # Satellital image background

    if satellitalImgQ == True:
        zoom = 17

        E1, E2, N1, N2 = CO.getBoundingBox(np.concatenate((towers.get_ArrayCoordinates(), bases.get_Coordinates())))

        p1 = CO.utm2latlon(np.array([E1, N2]), towers.get_UTM_Zone())
        p2 = CO.utm2latlon(np.array([E2, N1]), towers.get_UTM_Zone())

        a, latlon1, latlon2 = CO.get_Image_Cluster(p1[0], p1[1], p1[0]-p2[0], p2[1]-p1[1], zoom)

        CO.plot_Satellital_Map(canvas.axes, np.asarray(a), latlon1, latlon2, zoom)
    
    # Very inefficient but networkz overrides axes everytime anyways
    canvas.axes.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
 
    canvas.axes.set_xlabel("E coordinate")
    canvas.axes.set_ylabel("N coordinate")

    canvas.axes.yaxis.set_major_formatter(ScalarFormatter(useOffset = canvas.axes.get_yticks()[0]))
    canvas.axes.xaxis.set_major_formatter(ScalarFormatter(useOffset = canvas.axes.get_xticks()[0]))
    plt.axis('on') # turns on axis
    
    #canvas.axes.set_facecolor('black')

    canvas.axes.legend(['Towers','Line Segments','Bases'])

    canvas.draw()

    return None

# ------------------------------ Planner / Solver specific --------------------------------


def exec_Planner(ui: MainWindow):
    """
    Runs the planner solver and updates the UAV Team with their optimal paths.
    """

    if not(is_set(ui.status_flag, 1)) or not(is_set(ui.status_flag, 2)):
        print('Either Input Data or Weather not loaded')
        return None
    
    getMissionSettings(ui)

    problem = SO.Problem(ui.towers, ui.bases, ui.uavs, ui.weather)
    problem.link_Progress_Bar(ui.loadingPlanner)
    ui.loadingPlanner.setValue(10)
    problem.solve(ui.solverType)
    ui.loadingPlanner.setValue(100)
    ui.loadingPlanner.reset()

    mode = "PaV"

    ui.uavs.compute_Team_Waypoints(mode, ui.towers, ui.bases)
    ui.status_flag = set_bit(ui.status_flag, 5, 1)
    print(bin(ui.status_flag))


    ui.problem_graph = problem.get_Graph()

    ui.tableView.resizeColumnsToContents()


    problem.get_UAV_Team().plot_Routes(ui.sc.axes)
    ui.sc.draw()
    
    ui.status_flag = set_bit(ui.status_flag, 3, 1)
    print(bin(ui.status_flag))

    return None
