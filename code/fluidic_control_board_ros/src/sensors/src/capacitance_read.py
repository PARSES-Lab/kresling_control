#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time
from platform import system
if system() == 'Windows':
    from msvcrt import getch
else:
    from getch import getch

from math import isnan
from array import *
import binascii
import numpy as np
import serial.tools.list_ports
import rospy
import sys
from datetime import datetime
from std_msgs.msg import Float64
from sensors.msg import *
from sensors.msg import sensorMultiChannel
import typing
from sensors.cap_TI_funcs import *

def main():
    pub = rospy.Publisher('capacitance', sensorMultiChannel, queue_size=10)
    rospy.init_node('capacitance_node', anonymous=True)
    port_var = rospy.get_param('~port', '/dev/ttyACM0')

    msg = sensorMultiChannel()
    
    serial_port = serial.Serial(port = port_var, baudrate = 115200)

    # Flush the data so we don't get a division by zero error when we try to 
    # calculate the pF of the first read
    write_reg(serial_port, CONFIG, '0200')
    ldc_config(serial_port)
    write_reg(serial_port, CONFIG, '0000')
    
    # This sleep is necessary to finish flushing the data
    time.sleep(0.5)
    
    serial_port.close()

    
    try:
        serial_port.open()

        device_id = read_reg(serial_port, DEVICE_ID)
        rospy.loginfo("device_id=%s" % (device_id))

        start_stream(serial_port)

        while not rospy.is_shutdown():
            # Initializing array for capacitance channels

            data_array = read_stream(serial_port)

            pF = [float('nan') for x in range(4)]
            
            # Apply the data_to_pF function and store in the pF array
            k = 0
            for data in data_array:
                if data <= 0:
                    continue
                pF_value = data_to_pF(data)
                if pF_value >= 0:
                    pF[k] = pF_value  # Set positive values
                k += 1

            msg.Header.stamp= rospy.Time.now()
            msg.Header.frame_id = 'Capacitance'
            
            msg.channel0 = pF[0]
            msg.channel1 = pF[1]
            msg.channel2 = pF[2]
            msg.channel3 = pF[3]

            pub.publish(msg)
            rospy.loginfo(msg)

    except KeyboardInterrupt:
        exit()


if __name__ == "__main__":
    main()
