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

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic


import rospy
import math
from std_msgs.msg import Float64MultiArray


class Dif(object):
    def __init__(self):
        self.upper = 0
        self.lower = 0

    def upper_callback(self,msg1):
        self.upper0 = msg1.data[0]    
        self.upper1 = msg1.data[1]    
   
    def lower_callback(self,msg2):
        self.lower0 = msg2.data[0]
        self.lower1 = msg2.data[1]

        dif0 = self.upper0 - self.lower0
        dif1 = self.upper1 - self.lower1
        dif = [dif0, dif1]
        rospy.loginfo(dif)
   
    def listener(self,d):
      
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("imu_upper", Float64MultiArray, d.upper_callback)
       
        rospy.Subscriber("imu_lower", Float64MultiArray, d.lower_callback)
        rospy.spin()

        
if __name__ == '__main__':
    try:
        d = Dif()
        d.listener(d)
        
    except rospy.ROSInterruptException:
        pass

