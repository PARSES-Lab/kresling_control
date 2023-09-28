#!/usr/bin/env python

import rospy
from sensors.msg import sensorMultiChannel
from std_msgs.msg import Float32
import time
from statistics import median, mean
from math import isnan
import typing
import threading

# Poly-fit coefficients, kept here for backup and comparsion. Comments to be deleted
# C1 = -13747.57151316
# C2 = 27906.31912723
# C3 = -18982.50519102
# C4 = 4355.40024645

class LengthPublisherNode:
    def __init__(self, target_buffer_len : int, reject_cal_len : int):
        #calibration attributes
        self.cal_buffer = []
        self.cal_buffer_len = reject_cal_len + target_buffer_len
        self.skewed_val = 10 #maximum skew between median and mean for appropriate calibration.Set high for right now
        self.cal_med_cap = float('nan') # Set calibrated value to NaN to signal not calibrated
        self.reject_cal_val = reject_cal_len

        self.cap_polyfit = [4355.40024645, -18982.50519102, 27906.31912723, -13747.57151316] #polynomial fit coeffs starting with ZERO order
        self.capacitance_subscriber = rospy.Subscriber('/capacitance', sensorMultiChannel, self.capacitance_callback)
        self.length_publisher = rospy.Publisher('/kresling_length', sensorMultiChannel, queue_size=10)

    def calibrate(self, read_avg_cap: float):
        if not isnan(read_avg_cap): #add to buffer if all read-in capacitance values are positive
            self.cal_buffer.append(read_avg_cap)

        # Calculate stats of buffered calibration values
        if len(self.cal_buffer) > self.cal_buffer_len:
            if abs(median(self.cal_buffer[self.reject_cal_val:]) - mean(self.cal_buffer[self.reject_cal_val:])) < self.skewed_val: #if not too skewed from normal
                self.cal_med_cap = median(self.cal_buffer[self.reject_cal_val:])
                rospy.loginfo("Calibration complete.")
            else:
                rospy.logwarn("Difference between mean and median is too large. Restarting calibration routine.")
            rospy.loginfo("Median capacitance: %.2f", self.cal_med_cap) 
            rospy.loginfo("Mean capacitance: %.2f", mean(self.cal_buffer[self.reject_cal_val:]))

    def handle_cal_buffer(self, read_avg_cap: float):
        cal_thread = threading.Thread(target = self.calibrate, args=(read_avg_cap,))
        cal_thread.start()
        cal_thread.join()

        # If target calibration has been reached, flush the calibration buffer 
        if len(self.cal_buffer) > self.cal_buffer_len:
            self.cal_buffer = []

    def cap_to_length(self, delta_cap: float) -> float:
        delta_cap_FEA = round((delta_cap/4.7) + 0.605, 3) #re-factor capacitance to length for FEA fit
        length = 0
        for i in range(len(self.cap_polyfit)): 
            length += self.cap_polyfit[i] * delta_cap_FEA ** i
        return length

    def capacitance_callback(self, msg):
        read_avg_cap = ((msg.channel0 + msg.channel2 + msg.channel3)/3)

        if isnan(self.cal_med_cap): #if it's not calibrated, continue calibrating
            self.handle_cal_buffer(read_avg_cap)
        else:
            delta_avg_cap = self.cal_med_cap - read_avg_cap
            length = self.cap_to_length(delta_avg_cap)
            length_msg = sensorMultiChannel()
            length_msg.channel0 = length
            length_msg.Header.stamp = rospy.Time.now()
            self.length_publisher.publish(length_msg)
            rospy.loginfo(length)
            # , capacitance: %f", length, delta_avg_cap)


if __name__ == '__main__':
    rospy.init_node('length_publisher_node')
    target_buffer_len = rospy.get_param('~buffer_len', 100)
    reject_cal_vals = rospy.get_param('~reject_cal_len', 20)
    length_publisher_node = LengthPublisherNode(target_buffer_len, reject_cal_vals)
    rospy.spin()
