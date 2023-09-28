#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32

from labjack import ljm

def set_PWM(pwm_DIO, pwm_value):
    '''
    pwm_DIO: The digital IO port to address. On the LabJack T7 with no accessories,
    ports 0, 2, and 3 will work. 
    
    pwm_value: A value between 0 and 1 that represents the PWM signal to send. 
    Values between 0.3 and 0.7 will give you basically the whole range. 
    '''
    roll_value = 800000
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
    results = ljm.eWriteNames(handle, numFrames, aNames, aValues)

def main():
    global handle, DIOs
    DIOs = [0, 2, 3]
    # Open first found LabJack T7
    handle = ljm.openS("T7", "ANY", "ANY")  # T7 device, Any connection, Any identifier

    # Set up ROS node
    rospy.init_node('labjack_pwm_controller')

    # Interactive loop to get PWM values from the user
    while not rospy.is_shutdown():
        try:
            # Get PWM value from user
            pwm_value = float(input("Enter PWM value (between 0 and 1, enter 99 to exit): "))
            if 0 <= pwm_value <= 1:
                for currDIO in DIOs:
                    set_PWM(currDIO, pwm_value)

            elif pwm_value == 99:
                for currDIO in DIOs:
                    set_PWM(currDIO, 0)
                exit()

            else:
                print("Invalid input. Please enter a number between 0 and 1.")

        except KeyboardInterrupt:
            print("\nExiting.")
            break

    # Set all PWM signals to 0
    for currDIO in DIOs:
        set_PWM(currDIO, 0)

    # Turn off PWM output
    for currDIO in DIOs:
        aNames = ["DIO%i_EF_ENABLE" % currDIO]
        aValues = [0]
        numFrames = len(aNames)
        results = ljm.eWriteNames(handle, numFrames, aNames, aValues)

    # Close handle
    ljm.close(handle)

if __name__ == '__main__':
    main()