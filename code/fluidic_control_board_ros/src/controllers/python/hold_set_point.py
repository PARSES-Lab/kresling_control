#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensors.msg import sensorMultiChannel
from labjack import ljm
import time
import atexit
import typing
import serial

# Global variables for LabJack T7
DIOs = [0, 2, 3]
roll_value = 800000

class LabjackPID:
    def __init__(self):
        self.previous_pwm = 0
        self.handle = ljm.openS("T7", "ANY", "ANY")  # T7 device, Any connection, Any identifier
        self.last_time = rospy.get_time()

        # PID gain terms
        self.k_p = -0.3
        self.k_i = -0.0
        self.k_d = -0.0

        self.p_error = 0
        self.i_error = 0
        self.d_error = 0
        self.prev_error = 0

        self.prev_time = rospy.get_time()


    def pid_controller(self, length_set_point: float, length_read: float) -> float:
        # Calculate the error and its derivative
        self.p_error = length_set_point - length_read
        rospy.loginfo(self.p_error)
        rospy.loginfo(length_read)
        rospy.loginfo(length_set_point)
        self.d_error = (self.p_error - self.prev_error) / (rospy.get_time() - self.prev_time)
        self.i_error += 0.5 * (self.p_error + self.prev_error) * (rospy.get_time() - self.prev_time) #integration using trapezoid method

        #update
        self.prev_time = rospy.get_time()
        self.prev_error = self.p_error

        # Calculate the PWM value based on PID control
        self.pwm_value = (self.k_p * self.p_error + self.k_i * self.i_error + self.k_d * self.d_error)
        self.pwm_value = round(max(0, min(1, self.pwm_value)), 2)  # Ensure PWM value is within the range of 0 to 1

        
        return self.pwm_value   

    def set_PWM(self, pwm_DIO: int, pwm_value: float):
        aNames = ["DIO_EF_CLOCK0_DIVISOR", "DIO_EF_CLOCK0_ROLL_VALUE",
                "DIO_EF_CLOCK0_ENABLE", "DIO%i_EF_ENABLE" % pwm_DIO,
                "DIO%i_EF_INDEX" % pwm_DIO, "DIO%i_EF_CONFIG_A" % pwm_DIO,
                "DIO%i_EF_ENABLE" % pwm_DIO, "DIO18_EF_ENABLE",
                "DIO18_EF_INDEX", "DIO18_EF_ENABLE"]
        aValues = [1, roll_value,
                1, 0,
                0, pwm_value * roll_value,
                1, 0,
                7, 1]
        numFrames = len(aNames)
        results = ljm.eWriteNames(self.handle, numFrames, aNames, aValues)

    def control_pwm(self, pwm_value: float):
        for currDIO in DIOs:
            if pwm_value != self.previous_pwm:  # Compare with the previous PWM value
                self.set_PWM(currDIO, pwm_value)
        self.previous_pwm = pwm_value  # Update the previous PWM value

    def shutdown(self):
        # Set all PWM signals to 0
        for currDIO in DIOs:
            self.set_PWM(currDIO, 0)

        # Turn off PWM output
        for currDIO in DIOs:
            aNames = ["DIO%i_EF_ENABLE" % currDIO]
            aValues = [0]
            numFrames = len(aNames)
            results = ljm.eWriteNames(self.handle, numFrames, aNames, aValues)
        ljm.close(self.handle)


def length_callback(length_msg):
    # Your desired length set point
    length_set_point = 38

    # Create an instance of the LabjackPID class
    labjack_PID_instance = LabjackPID()

    # Get the length value from the received message
    length_read = length_msg.channel0

    # Calculate the PWM value using the PID controller
    pwm_value = labjack_PID_instance.pid_controller(length_set_point, length_read)

    # Control the PWM output
    labjack_PID_instance.control_pwm(pwm_value)
    rospy.loginfo(pwm_value)

if __name__ == '__main__':
    rospy.init_node('labjack_PID_controller')
    rospy.Subscriber('/kresling_length', sensorMultiChannel, length_callback)

    atexit.register(LabjackPID().shutdown)
    rospy.spin()
