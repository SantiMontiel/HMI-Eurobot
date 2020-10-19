#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random

import rospy
from std_msgs.msg import Int32, String

def callbackSubReset(msg):
    global Side, count

    Side = "Amarillo"
    count = 0

rospy.init_node('Tester')

# Publisher del tiempo de juego
pub = rospy.Publisher('TestTopic', Int32, queue_size=32)
rate = rospy.Rate(1)
count = 0

# Publisher del lado del campo
Side = "Azul"
pubSide = rospy.Publisher('TestTopic2', String, queue_size = 32)

# Publisher de conexión a Internet
Wifi = "No"
pubWifi = rospy.Publisher('TestTopic3', String, queue_size = 32)

# Publisher de batería
Batt = 68
pubBatt = rospy.Publisher('TestTopic4', Int32, queue_size = 32)

# Publisher de modo de funcionamiento
Mode = "Demo"
pubMode = rospy.Publisher('TestTopic5', String, queue_size = 32)

# Suscriptor del botón de reset
subReset = rospy.Subscriber('TestTopic6', Int32, callback = callbackSubReset)

while not rospy.is_shutdown():
    pub.publish(count)
    pubSide.publish(Side)
    pubWifi.publish(Wifi)
    pubBatt.publish(Batt)
    pubMode.publish(Mode)

    Batt -= 5
    if Batt < 0:
        Batt = 100

    count += 1
    rate.sleep()