#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Tkinter as tk
from PIL import Image, ImageTk
import time

import DisplayGUI

import rospy
from std_msgs.msg import Int32, String

"""
def callbackTime(msg):    
    print msg.data
    textVar = tk.StringVar()
    textVar.set(msg.data)
    label = tk.Label(window, textvariable = textVar, bg = "green")
    label.place(x = 190, y = 50, in_ = window)

def callbackSide(msg):
    textVar2 = tk.StringVar()
    textVar2.set(msg.data)
    labelSide2 = tk.Label(window, textvariable = textVar2)
    labelSide2.place(x = 190, y = 100, in_ = window)

    if msg.data == "Azul":
        canvas.create_oval(175, 102, 185, 112, fill = "#0000ff")
    elif msg.data == "Amarillo":
        canvas.create_oval(175, 102, 185, 112, fill = "#ffff00")

def callbackWifi(msg):
    textVar3 = tk.StringVar()
    textVar3.set(msg.data)
    labelWifi2 = tk.Label(window, textvariable = textVar3)
    labelWifi2.place(x = 190, y = 125, in_ = window)

    if msg.data == "Sí":
        canvas.create_oval(175, 127, 185, 137, fill = "#00ff00")
    elif msg.data == "No":
        canvas.create_oval(175, 127, 185, 137, fill = "#ff0000")

def callbackBatt(msg):
    textVar4 = tk.StringVar()
    textVar4.set(msg.data)
    labelBatt2 = tk.Label(window, textvariable = textVar4)
    labelBatt2.place(x = 190, y = 150, in_ = window)

    if msg.data >= 80:
        canvas.create_oval(175, 152, 185, 162, fill = "#177615")
    elif 80 > msg.data >= 60:
        canvas.create_oval(175, 152, 185, 162, fill = "#00ff00")
    elif 60 > msg.data >= 40:
        canvas.create_oval(175, 152, 185, 162, fill = "#ffff00")
    elif 40 > msg.data >= 20:
        canvas.create_oval(175, 152, 185, 162, fill = "orange")
    elif msg.data < 20:
        canvas.create_oval(175, 152, 185, 162, fill = "red")

def callbackMode(msg):
    textVar5 = tk.StringVar()
    textVar5.set(msg.data)
    labelMode2 = tk.Label(window, textvariable = textVar5)
    labelMode2.place(x = 190, y = 75, in_ = window)

def callbackButtonReset():
    global pubReset
    pubReset.publish(1)
    
"""

if __name__ == "__main__":

    # Graphic window  
    """
    label1 = tk.Label(window, text = "Tiempo de juego: ")
    label1.place(x = 50, y = 50, in_ = window)

    labelMode1 = tk.Label(window, text = "Modo:")
    labelMode1.place(x = 50, y = 75, in_ = window)

    labelSide1 = tk.Label(window, text = "Lado de juego:")
    labelSide1.place(x = 50, y = 100, in_ = window)

    labelWifi1 = tk.Label(window, text = "Conexión WiFi:")
    labelWifi1.place(x = 50, y = 125, in_ = window)

    labelBatt1 = tk.Label(window, text = "Batería: ")
    labelBatt1.place(x = 50, y = 150, in_ = window)

    buttonReset = tk.Button(text = "Reset", command = callbackButtonReset)
    buttonReset.place(x = 50, y = 175, in_ = window)

    # Imágenes
    path = "/home/santi/Ros_ws/src/Pantalla/src/"
    img = ImageTk.PhotoImage(Image.open(path + "wifi-true.png"))
    canvas.create_image(20, 20, image = img)
    """

    # Node initialization
    rospy.init_node('Display')

    # Publications
    # pubReset    = rospy.Publisher('TestTopic6', Int32, queue_size = 32)

    # Suscriptions
    """
    sub         = rospy.Subscriber('TestTopic', Int32, callback=callbackTime)
    subSide     = rospy.Subscriber('TestTopic2', String, callback = callbackSide)
    subWifi     = rospy.Subscriber('TestTopic3', String, callback = callbackWifi)
    subBatt     = rospy.Subscriber('TestTopic4', Int32, callback = callbackBatt)
    subMode     = rospy.Subscriber('TestTopic5', String, callback = callbackMode)
    """
    # Main loop (no mirar aux, es una juja temporal)
    
    root = tk.Tk()
    app = DisplayGUI.Application(root)
    app.mainloop()
