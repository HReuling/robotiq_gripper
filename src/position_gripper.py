#! /usr/bin/env python3

import rospy
import serial
import binascii
import time
from std_msgs.msg import Int16
import numpy as np

class SpacenavToGripper():
    def __init__(self, usb_port) -> None:
        # serial connection to the gripper
        self.ser = serial.Serial(port=usb_port,baudrate=115200,timeout=1,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)

        # initialize connection
        self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
        data_raw = self.ser.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        print ("Response 1 " + str(data))
        time.sleep(0.01)

        self.ser.write(b'\x09\x03\x07\xD0\x00\x01\x85\xCF')
        data_raw = self.ser.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        print ("Response 2 " + str(data))
        time.sleep(1)

        # initialize position to zero
        self.position = 0

        # subscriber for the target position
        rospy.init_node('robotiq_gripper', anonymous=True)
        rospy.Subscriber('/robotiq_gripper/position', Int16, self.position_change_cb, queue_size=1)

        self.rate = rospy.Rate(10)  # rate

        # keep the node running
        while not rospy.is_shutdown():
            self.rate.sleep()
            

    def position_change_cb(self, data):
        # callback function when the position has changed
        if data.data != self.position:
            # read position data
            self.position = data.data
            # build the data for serial communication
            msg = self.build_msg(self.position)
            # send data
            self.ser.write(msg)
            time.sleep(0.01)
            

    def build_msg(self, position:int):
        # beginning for the command to go to requested position
        msg_start = "x09 x10 x03 xE8 x00 x03 x06 x09 x00 x00 "
        # ending of the command first value specifies the speed second values specifies the force
        msg_end = " x2F xFF"
        # build the command with the position
        msg = msg_start + format(position, '02x') + msg_end
        # calculate the CRC and add it to the command
        msg = msg + self.modbusCrc(str(msg.replace("x", "")))
        # transform the string to bytes
        msg_bytes = bytes.fromhex(bytes(msg, 'utf-8').decode().replace('x', ''))
        return msg_bytes
        

    def modbusCrc(self, msg:str):
        '''
        CRC-16-ModBus Algorithm
        '''
        data = bytearray(bytes.fromhex(msg))
        poly = 0xA001
        crc = 0xFFFF
        for b in data:
            crc ^= (0xFF & b)
            for _ in range(0, 8):
                if (crc & 0x0001):
                    crc = ((crc >> 1) & 0xFFFF) ^ poly
                else:
                    crc = ((crc >> 1) & 0xFFFF)
        res = (hex(np.uint16(crc)).replace("0x", ""))

        # check whether the number of digits is correct
        if len(res)<3: res = "0" + res[:1] + "0" + res[1:]
        elif len(res)<4: res = "0" + res
        # change the order of the two numbers
        return " x" + res[2:] + " x" + res[:2]
        
    
if __name__ == '__main__':
    try:
        usb_port = rospy.get_param('~usb_port')
    except:
        usb_port = '/dev/ttyUSB0'
        rospy.loginfo("USB Port didn't set, falling back to /dev/ttyUSB0")
    spacenav_to_gripper = SpacenavToGripper()
