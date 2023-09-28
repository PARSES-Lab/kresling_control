#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import sys
import numpy as np
from datetime import datetime
from std_msgs.msg import Float64
from sensors.msg import *
from sensors.msg import sensorMultiChannel
 

def read_pressure():
    
    pub = rospy.Publisher('pressure', sensorMultiChannel, queue_size=10)
    rospy.init_node('pressure_node', anonymous=True)
    rate = rospy.Rate(100) 
    msg = sensorMultiChannel()
    seq= 0

    args = rospy.myargv(argv = sys.argv)
    if len(args) != 2:
        print("error")
        sys.exit(1)

    connected_port = args[1]
    serial_port = rospy.get_param('~port', connected_port)
    serial_baud = rospy.get_param('~baudrate',115200)

    ser = serial.Serial(serial_port, serial_baud, timeout = 10)
    while not rospy.is_shutdown():
        message = ser.readline()
        decoded_message = message.decode()
        seq = seq + 1

        # Ensure data coming is for pressure values
        if "P" in decoded_message:
            data = decoded_message.split(",")

            now = rospy.get_rostime()
            sensor1 = float(data[1])
            sensor2 = float(data[2])
            sensor3 = float(data[3])
            sensor4 = float(data[4])

            msg.Header.seq = seq
            msg.Header.stamp.secs = int(now.secs)
            msg.Header.stamp.nsecs = int(now.nsecs)
            msg.Header.frame_id = 'Pressure'

            msg.channel0 = sensor1
            msg.channel1 = sensor2
            msg.channel2 = sensor3
            msg.channel3 = sensor4

            pub.publish(msg)
            rospy.loginfo(msg)

if __name__ == '__main__':
    try:
        read_pressure()
    except rospy.ROSInterruptException:
        pass
