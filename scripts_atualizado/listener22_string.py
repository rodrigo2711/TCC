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
from std_msgs.msg import String

upper=0
lower=0

def upper_callback(data):
    #global upper
    ##upper = float(upper)
    #rospy.loginfo(rospy.get_caller_id() + 'imu_upper %.2f', data.upper)
    rospy.loginfo('%s', data.data)
    #upper.append(data.data)
    #upper = float(data.data) 
     

def lower_callback(volt):
    ##obal lower
    #lower = float(lower)
    #ower = float(data.data)
    #rospy.loginfo(rospy.get_caller_id() + 'imu_lower %.2f', data.lower)
    rospy.loginfo('%.2f', volt.data)
    #lower.append(data.data)
    #lower = data.data

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
  
    rospy.init_node('listener', anonymous=False)

    #rospy.Subscriber("imu_lower", Float64, lower_callback)
    rospy.Subscriber("imu_upper", String, upper_callback)

    #dif =  float()
    #dif = (upper - lower)


        #rospy.loginfo(rospy.get_caller_id() + 'I heard dif %.2f', dif) 
   # rospy.loginfo( '%.2f' ,dif)
       # rospy.loginfo( upper)  
    
    rospy.spin()
    #i = 0
    #while i in range(15):
      #  print(upper,lower)
        #i = i + 1
        #print type(upper)  

        
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

