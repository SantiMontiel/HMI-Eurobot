#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys 
from PyQt5.QtGui	 import QIcon
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QAction, QTabWidget, QVBoxLayout, QLabel

# Creating the main window 
class Application(QMainWindow): 
	def __init__(self): 
		super().__init__()
		self.setWindowTitle('UAH Robotics Team - Eurobot 2021: Sail the World!') 
		self.setGeometry(0, 0, 1024, 600) 

		self.tab_manager = TabManager(self) 
		self.setCentralWidget(self.tab_manager) 

		self.show() 

# Creating tab widgets 
class TabManager(QWidget): 
	def __init__(self, parent): 
		super(QWidget, self).__init__(parent) 
		self.layout = QVBoxLayout(self) 

		# Initialize tab screen 
		self.tabM = QTabWidget() 
		self.tab1 = QWidget() 
		self.tab2 = QWidget() 
		self.tab3 = QWidget()
		self.tab4 = QWidget()
		self.tab5 = QWidget() 
		self.tabM.resize(1024, 600) 

		# Add tabs 
		self.tabM.addTab(self.tab1, QIcon("img/boat.png"),("Inicio")) 
		self.tabM.addTab(self.tab2, QIcon("img/compass.png"), ("Parámetros"))
		self.tabM.addTab(self.tab3, QIcon("img/mountain.png"), ("Mapa")) 
		self.tabM.addTab(self.tab4, QIcon("img/camera.png"), ("Cámara")) 
		self.tabM.addTab(self.tab5, QIcon("img/crab.png"), ("Puntuación")) 

		# Create first tab 
		self.tab1.layout = QVBoxLayout(self) 
		self.l = QLabel() 
		self.l.setText("This is the first tab") 
		self.tab1.layout.addWidget(self.l) 
		self.tab1.setLayout(self.tab1.layout) 

		# Add tabs to widget 
		self.layout.addWidget(self.tabM) 
		self.setLayout(self.layout)



if __name__ == '__main__': 

	app = QApplication(sys.argv) 
	root = Application() 
	sys.exit(app.exec_()) 
