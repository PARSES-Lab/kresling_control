'''
Code to read from the registers on the FDC2214EVM capacitance to digital board
Based on code from TI: https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/295036/faq-inductive-sensing-faq---frequently-asked-questions#Q41

Further modifications for the PARSES lab at Northeastern by Sonia Roberts, Immanuel Ampomah Mensah, Jessica Healey, Nathaniel Hanson, and Kris Dorsey

--- System check added to import proper library msvcrt for Windows, getch for Linux
--- Hard coded COM string prefix removed for linux applications

To run this file:
1. Plug your FDC2214 EVM board into your laptop with a USB cable
2. Run this file from the command line with python read_evm_registers.py
3. The program will display the available devices. The EVM board is likely listed as something like 
"USB Serial Device (COM8)". If you are confused about which one it is, try unplugging it and running 
the program again. See which device disappears from the list. 
4. Type the number of the port you want and hit Enter. The program will print the device ID and then 
start printing data to the terminal. 

Use read and write register functions for *slow* speeds, over 100 ms 
Use stream functions for faster read/write speeds. Time is not set but we have observed up to 90 Hz performance

2 line EVM synopsis: Capacitance is read from LC tank resonance and written to microcontroller register on board. This script reads/streams LC frequency and converts to cap in pF
'''

#!/usr/bin/env python
# -*- coding: utf-8 -*-

from platform import system
if system() == 'Windows':
    from msvcrt import getch
else:
    from getch import getch

from math import isnan, pi
from array import *
import binascii
import serial.tools.list_ports
import serial
import sys
import typing
import rospy

#############################################################################################
# FDC2214 addressing space
DATA_MSB_CH0        = '00'
DATA_LSB_CH0        = '01'
DATA_MSB_CH1        = '02'
DATA_LSB_CH1        = '03'
DATA_MSB_CH2        = '04'
DATA_LSB_CH2        = '05'
DATA_MSB_CH3        = '06'
DATA_LSB_CH3        = '07'
RCOUNT_CH0          = '08'
RCOUNT_CH1          = '09'
RCOUNT_CH2          = '0A'
RCOUNT_CH3          = '0B'
OFFSET_CH0          = '0C'
OFFSET_CH1          = '0D'
OFFSET_CH2          = '0E'
OFFSET_CH3          = '0F'
SETTLECOUNT_CH0     = '10'
SETTLECOUNT_CH1     = '11'
SETTLECOUNT_CH2     = '12'
SETTLECOUNT_CH3     = '13'
CLOCK_DIVIDERS_CH0  = '14'
CLOCK_DIVIDERS_CH1  = '15'
CLOCK_DIVIDERS_CH2  = '16'
CLOCK_DIVIDERS_CH3  = '17'
STATUS              = '18'
ERROR_CONFIG        = '19'
CONFIG              = '1A'
MUX_CONFIG          = '1B'
RESET_DEV           = '1C'
DRIVE_CURRENT_CH0   = '1E'
DRIVE_CURRENT_CH1   = '1F'
DRIVE_CURRENT_CH2   = '20'
DRIVE_CURRENT_CH3   = '21'
MANUFACTURER_ID     = '7E'
DEVICE_ID           = '7F'


DEBUG_PRINT_TX_DATA = 0
DEBUG_PRINT_RX_DATA = 0
DEBUG_PRINT_READ_DATA = 0

class crc8:
    '''
    Implements a cyclic redundancy check
    https://en.wikipedia.org/wiki/Cyclic_redundancy_check
    Verbatim from TI
    '''
    def __init__(self):
        self.crcTable = (0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38,
            0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77,
            0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46,
            0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
            0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4,
            0xc3, 0xca, 0xcd, 0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b,
            0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba,
            0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
            0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7,
            0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88,
            0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a, 0x27, 0x20, 0x29,
            0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
            0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b,
            0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74,
            0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b,
            0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
            0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1,
            0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e,
            0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f,
            0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
            0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d,
            0x3a, 0x33, 0x34, 0x4e, 0x49, 0x40, 0x47, 0x52, 0x55,
            0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64,
            0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
            0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae,
            0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91,
            0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83, 0xde, 0xd9, 0xd0,
            0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
            0xfa, 0xfd, 0xf4, 0xf3)
    def crc(self, msg):
        runningCRC = 0
        for c in msg:
            c = ord(str(c))
            runningCRC = self.crcByte(runningCRC, c)
        return runningCRC
 
    def crcByte(self, oldCrc, byte):
        res = self.crcTable[oldCrc & 0xFF ^ byte & 0xFF];
        return res 

def write_reg(serial_port, addr: str, data: str):
    crc = crc8()
    serial_string = '4C150100042A'+ addr + data
    serial_string = bytes.fromhex(serial_string)
    crc_byte = chr(crc.crc(serial_string.decode('latin-1')))
    serial_port.write(serial_string+bytes(crc_byte, 'latin-1'))
    s = serial_port.read(32)
    if DEBUG_PRINT_RX_DATA:
        print("Read:%s" % (binascii.hexlify(s)))
    if (s[3:4] != bytes('\x00','latin-1')):
        print("Error in write register")
        exit()

def read_reg(serial_port, addr: str):
    crc = crc8()
    # Set the register to be read
    serial_string = '4C150100022A'+ addr
    serial_string = bytes.fromhex(serial_string)
    crc_byte = chr(crc.crc(serial_string.decode('latin-1')))
    serial_port.write(serial_string+bytes(crc_byte,'latin-1')) 
    s = serial_port.read(32)
    if DEBUG_PRINT_RX_DATA:
        print("Read:%s" % (binascii.hexlify(s)))
    if (s[3:4] != bytes('\x00','latin-1')):
        print("Error in set register")
        exit()

    #Send read register command to microcontroller
    serial_string = '4C140100022A02'
    serial_string = bytes.fromhex(serial_string)
    crc_byte = chr(crc.crc(serial_string.decode('latin-1')))
    serial_port.write(serial_string+bytes(crc_byte,'latin-1')) 
    s = serial_port.read(32)
    if DEBUG_PRINT_RX_DATA:
        print("Read:%s" % (binascii.hexlify(s)))
    if (s[3:4] != bytes('\x00','latin-1')):
        print("Error in read register")
        exit()
    data_read = s[6:7] + s[7:8] 
    if DEBUG_PRINT_READ_DATA:
        print("Addr:", addr,"Data:", binascii.hexlify(data_read) )
    return binascii.hexlify(data_read)

def start_stream(serial_port):
    crc = crc8()
    serial_string = '4C0501000601290404302AC1' #start stream write string
    serial_string = bytes.fromhex(serial_string)
    crc_byte = chr(crc.crc(serial_string.decode('latin-1')))
    serial_port.write(serial_string+bytes(crc_byte, 'latin-1'))

def read_stream(serial_port):
    #Read in 32 bytes, bytes 6 through 22 contain capacitance data for chan 0-3 on EVM board
    s = serial_port.read(32)
    if (s[3:4] != bytes('\x00', 'latin-1')):
        print("Error in read register")
        rospy.log("Error in read register")
    ch0 = int(binascii.hexlify(s[6:10]), 16)
    ch1 = int(binascii.hexlify(s[10:14]), 16)
    ch2 = int(binascii.hexlify(s[14:18]), 16)
    ch3 = int(binascii.hexlify(s[18:22]), 16)
    data_array = [ch0, ch1, ch2, ch3]
    return data_array

def ldc_config(serial_port):
    write_reg(serial_port, MUX_CONFIG, 'C20F')

def data_to_pF(data: float) -> float:
        # frefx comes from the Sensing Solutions EVM GUI. Configuration > Fref > Calculated (MHz)
        frefx = 43400000
        # Oscillator reference frequency
        fsensor_0 = frefx * data / (2 ** 28)

        # L1-L4, C7, C10, C15, C17 specified in schematic and BOM for the tank in FDC2214
        inductance = 0.000018 #18 uH
        parallel_capacitance = 0.000000000033  # 33 pF

        F_0 = 1 / (inductance * (2 * pi * fsensor_0) ** 2) - parallel_capacitance

        pF_0 = F_0 * 10 ** 12
        return(pF_0)
