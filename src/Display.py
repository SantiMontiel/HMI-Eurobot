#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, json
from PyQt5.QtCore	 import Qt
from PyQt5.QtGui	 import QIcon
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QRadioButton, QWidget, QTabWidget, QHBoxLayout, QVBoxLayout, QLabel, QSlider, QLCDNumber, QComboBox, QCheckBox, QTableWidget, QTableWidgetItem, QLineEdit

WIDTH = 1024
HEIGHT = 600

# Creating the main window 
class Application(QMainWindow): 
	def __init__(self): 
		super().__init__()
		self.setWindowTitle('UAH Robotics Team - Eurobot 2021: Sail the World!') 
		self.setGeometry(0, 0, WIDTH, HEIGHT) 

		self.tab_manager = TabManager(self) 
		self.setCentralWidget(self.tab_manager) 

		self.show() 

# Creating tab widget
class TabManager(QWidget): 
	def __init__(self, parent): 
		super(QWidget, self).__init__(parent) 
		self.layout = QVBoxLayout(self) 

		# Initialize tab screen 
		self.tabM = QTabWidget()
		self.tabM.resize(WIDTH, HEIGHT) 

		# Add tabs, choose icon and name
		self.tabM.addTab(HomeTab(), QIcon("img/boat.png"),("Inicio")) 
		self.tabM.addTab(ParamTab(), QIcon("img/compass.png"), ("Parámetros"))
		self.tabM.addTab(MapTab(), QIcon("img/mountain.png"), ("Mapa")) 
		self.tabM.addTab(CameraTab(), QIcon("img/camera.png"), ("Cámara")) 
		self.tabM.addTab(PointsTab(), QIcon("img/crab.png"), ("Puntuación")) 

		# Add tabs to widget 
		self.layout.addWidget(self.tabM) 
		self.setLayout(self.layout)

class HomeTab(QWidget):
	def __init__(self):
		super().__init__()

		# START: Define layout
		self.layout = QVBoxLayout()

		# END: Set layout
		self.setLayout(self.layout)

		
class ParamTab(QWidget):
	def __init__(self):
		super().__init__()

		# START: Define layout
		self.hLayout = QHBoxLayout()
		self.lLayout = QVBoxLayout()
		self.lbLayout = QHBoxLayout()
		self.rLayout = QVBoxLayout()
		self.rtLayout = QHBoxLayout()


		# --- TAB CONTENT
		# ----- LEFT LAYOUT: Generación de fichero

		# Title
		self.genLabel = QLabel("Generador de ficheros de configuración")
		self.genLabel.setAlignment(Qt.AlignCenter)
		self.genLabel.setStyleSheet("background-color: #8673a1; font-weight: bold; color: white; border-radius: 10px;")
		self.lLayout.addWidget(self.genLabel)

		# Robot name: Parejitas o Posavasos
		self.robotName = QLabel("Robot:")
		self.robotName.setStyleSheet("font-weight: bold")		
		self.lLayout.addWidget(self.robotName)

		self.robotBox1 = QRadioButton("Posavasos")
		self.robotBox1.setChecked(True)
		self.lLayout.addWidget(self.robotBox1)
		self.robotBox2 = QRadioButton("Parejitas")
		self.lLayout.addWidget(self.robotBox2)

		# Pose inicial: X e Y
		self.poseLabel = QLabel("Posición inicial:")
		self.poseLabel.setStyleSheet("font-weight: bold")
		self.lLayout.addWidget(self.poseLabel)

		self.labelPoseX = QLabel("Pose en X: 0")
		self.lLayout.addWidget(self.labelPoseX)

		self.sliderX = QSlider(Qt.Horizontal)							# Create horizontal slider
		self.sliderX.setMinimum(0)										# Minimum value
		self.sliderX.setMaximum(300)									# Max value
		self.sliderX.setValue(0)										# Initial value
		self.sliderX.setTickPosition(QSlider.TicksBelow)				# Posición de los ticks
		self.sliderX.setTickInterval(1)									# Intervalo entre los ticks
		self.sliderX.valueChanged.connect(self.updatePose)				#
		self.lLayout.addWidget(self.sliderX)							#

		self.labelPoseY = QLabel("Pose en Y: 0")
		self.lLayout.addWidget(self.labelPoseY)

		self.sliderY = QSlider(Qt.Horizontal)
		self.sliderY.setMinimum(0)
		self.sliderY.setMaximum(300)
		self.sliderY.setValue(0)
		self.sliderY.setTickPosition(QSlider.TicksBelow)
		self.sliderY.setTickInterval(1)
		self.sliderY.valueChanged.connect(self.updatePose)
		self.lLayout.addWidget(self.sliderY)

		# Lado: azul o amarillo
		self.sideLabel = QLabel("Lado inicial:")
		self.sideLabel.setStyleSheet("font-weight: bold")
		self.lLayout.addWidget(self.sideLabel)

		self.sideCombo = QComboBox()
		self.sideCombo.addItem("Azul")
		self.sideCombo.addItem("Amarillo")
		self.lLayout.addWidget(self.sideCombo)

		# Modo: demo o competitivo
		self.modeLabel = QLabel("Modo de funcionamiento:")
		self.modeLabel.setStyleSheet("font-weight: bold")
		self.lLayout.addWidget(self.modeLabel)

		self.modeCombo = QComboBox()
		self.modeCombo.addItem("Demo")
		self.modeCombo.addItem("Competitivo")
		self.lLayout.addWidget(self.modeCombo)

		# Rutinas: por definir
		self.routinesLabel = QLabel("Rutina a ejecutar:")
		self.routinesLabel.setStyleSheet("font-weight: bold")
		self.lLayout.addWidget(self.routinesLabel)

		self.routinesCombo = QComboBox()
		self.routinesCombo.addItem("Rutinas por definir")
		self.lLayout.addWidget(self.routinesCombo)

		# Botones de generar fichero y clear (Left-bottom layout)
		self.clearBtn = QPushButton("Reestablecer valores predeterminados")
		self.clearBtn.setStyleSheet("background-color: #ff3f3f")
		self.clearBtn.clicked.connect(self.clearField)
		self.lbLayout.addWidget(self.clearBtn)
		self.generateBtn = QPushButton("Generar fichero de configuración")
		self.generateBtn.setStyleSheet("background-color: lightgreen")
		self.generateBtn.clicked.connect(self.generateBtnClicked)
		self.lbLayout.addWidget(self.generateBtn)

		# ----- RIGHT LAYOUT: Generador de rutinas + precarga de rutinas
		self.exLabel = QLabel("Generador de rutinas (en progreso)")
		self.exLabel.setAlignment(Qt.AlignCenter)
		self.exLabel.setStyleSheet("background-color: #7f0000; font-weight: bold; color: white; border-radius: 10px;")
		self.rLayout.addWidget(self.exLabel)

		# -- RIGHT TOP LAYOUT (QHBoxLayout: QComboBox (rutina) + QLineEdit (atributo))
		self.chooseRoutCombo = QComboBox()
		self.chooseRoutCombo.addItem("avanzar")
		self.chooseRoutCombo.addItem("coger vasos")
		self.chooseRoutCombo.addItem("aparcar")
		self.rtLayout.addWidget(self.chooseRoutCombo)

		self.chooseAttrLine = QLineEdit()
		self.rtLayout.addWidget(self.chooseAttrLine)

		# Botón para agregar rutina
		self.addRoutineBtn = QPushButton("Agregar rutina")
		self.addRoutineBtn.clicked.connect(self.addRoutineBtnClicked)
		self.rtLayout.addWidget(self.addRoutineBtn)

		# Tabla de comandos para generar rutina
		self.routTable = QTableWidget()
		self.routTable.setFixedHeight(400)
		self.index = 0
		self.routTable.setRowCount(4)
		self.routTable.setColumnCount(2)
		self.routTable.setHorizontalHeaderLabels(('Rutina', 'Atributo'))
		self.routTable.setColumnWidth(0, 334)
		self.routTable.setColumnWidth(1, 120)
		self.rLayout.addWidget(self.routTable)

		# END: Set layout
		self.lLayout.addLayout(self.lbLayout)
		self.rLayout.addLayout(self.rtLayout)
		self.hLayout.addLayout(self.lLayout, 5)
		self.hLayout.addLayout(self.rLayout, 5)
		self.setLayout(self.hLayout)

	# FUNCTIONS
	def updatePose(self, event):
		self.labelPoseX.setText("Pose en X: " + str(self.sliderX.value()))
		self.labelPoseY.setText("Pose en Y: " + str(self.sliderY.value()))

	def generateBtnClicked(self):
		self.generateConfigFile()

	def clearBtnClicked(self):
		self.clearField()

	def addRoutineBtnClicked(self):
		self.addRoutine()

	def generateConfigFile(self):

		# 1. DATA COLLECTION
		# 1.1. Robot name
		if self.robotBox1.isChecked():
			robot_name = "Posavasos"
		elif self.robotBox2.isChecked():
			robot_name = "Parejitas"
		print(robot_name)

		# 1.2. Pose
		pose = [self.sliderX.value(), self.sliderY.value()]
		print(pose)

		# 1.3. Lado
		side = str(self.sideCombo.currentText())
		print(side)

		# 1.4. Modo
		mode = str(self.modeCombo.currentText())
		print(mode)

		# 1.5. Rutinas
		routines = str(self.routinesCombo.currentText())
		print(routines)
		
		# 2. GENERATE JSON FILE
		json_dict = {	"robot_name": str(robot_name),
						"pose": str(pose),
						"side": str(side),
						"mode": str(mode),
						"routines": str(routines)}

		with open("config/config.json", "w") as json_file:
			json.dump(json_dict, json_file)

	def clearField(self):
		self.robotBox1.setChecked(True)
		self.sliderX.setValue(0)
		self.sliderY.setValue(0)
		self.sideCombo.setCurrentIndex(0)
		self.modeCombo.setCurrentIndex(0)
		self.routinesCombo.setCurrentIndex(0)

	def addRoutine(self):
		self.routTable.setItem(self.index, 0, QTableWidgetItem(str(self.chooseRoutCombo.currentText())))
		self.routTable.setItem(self.index, 1, QTableWidgetItem(str(self.chooseAttrLine.text())))
		self.index += 1
		print("Hola")

class MapTab(QWidget):
	def __init__(self):
		super().__init__()

		# START: Define layout
		self.layout = QVBoxLayout()

		# Etiqueta wip
		self.wip = QLabel("Work in progress!")		
		self.layout.addWidget(self.wip)

		# END: Set layout
		self.setLayout(self.layout)


class CameraTab(QWidget):
	def __init__(self):
		super().__init__()

		# START: Define layout
		self.layout = QVBoxLayout()

		# Etiqueta wip
		self.wip = QLabel("Work in progress!")		
		self.layout.addWidget(self.wip)

		# END: Set layout
		self.setLayout(self.layout) 


class PointsTab(QWidget):
	def __init__(self):
		super().__init__()

		# START: Define layout
		self.layout = QVBoxLayout()

		# Etiqueta wip
		self.wip = QLabel("Work in progress!")		
		self.layout.addWidget(self.wip)

		# END: Set layout
		self.setLayout(self.layout)


if __name__ == '__main__': 

	app = QApplication(sys.argv) 
	root = Application() 
	sys.exit(app.exec_()) 
