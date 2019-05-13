#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import math
PKG = 'beginner_tutorials'
import roslib; roslib.load_manifest(PKG)
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
import numpy as np
import tf
from beginner_tutorials.msg import Eulers
import smbus            #import SMBus module of I2C
from time import sleep          #import

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


#    print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)     
    

def imupub():
    self.euler_msg = Eulers()
    pub = rospy.Publisher("euler", Eulers, queue_size=10)
    rospy.init_node('tcc', anonymous=True)
    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():
    
        i=0;
        
        
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
    
        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
    
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
 
        
        angle_pitch = Gx * 0.0000611 # Calculate the traveled pitch angle and add this to the angle_pitch variable
        angle_roll = Gy * 0.0000611  #Calculate the traveled roll angle and add this to the angle_roll variable
        angle_pitch = angle_roll * math.sin(Gz * 0.000001066)  #If the IMU has yawed transfer the roll angle to the pitch angel
        angle_roll = angle_pitch * math.sin(Gz * 0.000001066)  #If the IMU has yawed transfer the pitch angle to the roll angel

        #Accelerometer angle calculations
     
        acc_total_vector = math.sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az)) #Calculate the total accelerometer vector
        angle_pitch_acc = np.arcsin(Ay / acc_total_vector) * 57.296   #Calculate the pitch angle
        angle_roll_acc = np.arcsin(Ax / acc_total_vector) * -57.296    #Calculate the roll angle

        #Calibrar automaticamente
        angle_pitch_acc -= 1.1 # Accelerometer calibration value for pitch
        angle_roll_acc -= -2.73 #Accelerometer calibration value for roll
        
        #If the IMU is already started
        if i!=0:                  
            angle_pitch = angle_pitch * 0.998 + angle_pitch_acc * 0.002  #Correct the drift of the gyro pitch angle with the accelerometer pitch angle
            angle_roll = angle_roll * 0.998 + angle_roll_acc * 0.002     #Correct the drift of the gyro roll angle with the accelerometer roll angle
 
        else:                                             #At first start
            angle_pitch = angle_pitch_acc                 #Set the gyro pitch angle equal to the accelerometer pitch angle
            angle_roll = angle_roll_acc                   #Set the gyro roll angle equal to the accelerometer roll angle
            i += 1
  

        self.got_new_msg = True
        self.euler_msg.header.stamp = msg.header.stamp
        self.euler_msg.roll  = angle_roll
        self.euler_msg.pitch = angle_pitch

 
         #  imu_str = "angle_pitch_acc: %.2f" %angle_pitch_acc, "angle_roll_acc: %.2f" %angle_roll_acc
        imu_str = "angle_pitch: %.2f" %angle_pitch, "angle_roll: %.2f" %angle_roll
        msg = Imu()

       
        rospy.loginfo(imu_str)
        pub_euler.publish(self.euler_msg)
        rate.sleep()



if __name__ == '__main__':
    try:
        bus = smbus.SMBus(1)     # or bus = smbus.SMBus(0) for older version boards
        Device_Address = 0x68   # MPU6050 device address

        beginner_tutorials = MPU_Init()
        

        print (" Reading Data of Gyroscope and Accelerometer")

        imupub()
    except rospy.ROSInterruptException:
        pass
