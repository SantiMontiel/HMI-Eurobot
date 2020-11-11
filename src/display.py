#!/usr/bin/env python
# -*- coding: utf-8 -*-

import 	rospy
from 	std_msgs.msg	import Int32, String

from 	PyQt5 			import QtWidgets, QtGui, QtCore
from 	PyQt5.QtWidgets	import QApplication, QWidget, QLabel, QMainWindow
from 	PyQt5.QtGui		import QIcon, QPixmap
import 	sys

class MainWindow(QWidget):

	def __init__(self):
		super(MainWindow, self).__init__()
		self.initUI()

	def initUI(self):
		self.setFixedSize(1280, 720)
		self.setWindowTitle("Eurobot 2021: Sail the World!")
		self.setWindowIcon(QtGui.QIcon("SailTheWorld.png"))

		# Ejemplo de etiqueta de texto		
		self.label = QtWidgets.QLabel(self)
		self.label.setText("Hello, World!")
		self.label.move(50, 50)

		# Ejemplo de foto
		self.label2 = QLabel(self)
        self.pixmap = QPixmap('SailTheWorld.png')
        self.label2.setPixmap(self.pixmap)
        self.resize(self.pixmap.width(), self.pixmap.height())

if __name__ == "__main__":

	rospy.init_node('display')	

	app = QApplication(sys.argv)
	root = MainWindow()
	root.show()
	sys.exit(app.exec_())
