#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys 
from PyQt5.QtCore    import Qt
from PyQt5.QtGui	 import QIcon
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QRadioButton, QWidget, QTabWidget, QHBoxLayout, QVBoxLayout, QLabel, QSlider, QLCDNumber, QComboBox, QCheckBox

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

		# Botón de generar fichero (Left-bottom layout)
		self.clearBtn = QPushButton("Reestablecer valores predeterminados")
		self.clearBtn.setStyleSheet("background-color: #ff3f3f")
		self.lbLayout.addWidget(self.clearBtn)
		self.generateBtn = QPushButton("Generar fichero de configuración")
		self.generateBtn.setStyleSheet("background-color: lightgreen")
		self.lbLayout.addWidget(self.generateBtn)

		# ----- RIGHT LAYOUT: Empty
		self.exLabel = QLabel("Generador de rutinas (en progreso)")
		self.exLabel.setAlignment(Qt.AlignCenter)
		self.exLabel.setStyleSheet("background-color: #7f0000; font-weight: bold; color: white; border-radius: 10px;")
		self.rLayout.addWidget(self.exLabel)

		# END: Set layout
		self.lLayout.addLayout(self.lbLayout)
		self.hLayout.addLayout(self.lLayout, 5)
		self.hLayout.addLayout(self.rLayout, 5)
		self.setLayout(self.hLayout)

	# FUNCTIONS
	def updatePose(self, event):
		self.labelPoseX.setText("Pose en X: " + str(self.sliderX.value()))
		self.labelPoseY.setText("Pose en Y: " + str(self.sliderY.value()))

	# TODO: def clearField(self):

	# TODO: def generateFile(self):



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
