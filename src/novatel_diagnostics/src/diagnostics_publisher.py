#! /usr/bin/env python
# -*- coding: utf-8 -*-
#     _____
#    /  _  \
#   / _/ \  \
#  / / \_/   \
# /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
# \  / \_/ \  / /  _\| |  | __| / _ \ | ┌┐ \ | ┌┐ \ / _ \ |_   _|| | | |
#  \ \_/ \_/ /  | |  | |  | └─┐| |_| || └┘ / | └┘_/| |_| |  | |  | └─┘ |
#   \  \_/  /   | |_ | |_ | ┌─┘|  _  || |\ \ | |   |  _  |  | |  | ┌─┐ |
#    \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
#            ROBOTICS™
#
#
#  Copyright © 2012 Clearpath Robotics, Inc. 
#  All Rights Reserved
#  
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Clearpath Robotics, Inc. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Please send comments, questions, or patches to skynet@clearpathrobotics.com
#

# ROS
import rospy
import novatel_msgs.msg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class BitfieldRepublisher(object):
  def __init__(self, topic_name, topic_type, fields):
    flags = []
    attrs = dir(novatel_msgs.msg.GeneralStatus)
    for field in fields:
      prefix = field.upper() + "_"
      field_flags = []
      for attr in attrs:
        if attr.startswith(prefix):
          short_name = attr[len(prefix):len(attr)]
          mask = getattr(novatel_msgs.msg.GeneralStatus, attr)
          field_flags.append((mask, short_name))
      field_flags.sort()
      flags.append((field, tuple(field_flags)))
    self.flags = tuple(flags)

    self.status_msg = DiagnosticArray()
    self.status_msg.status.append(DiagnosticStatus(
      level = DiagnosticStatus.OK,
      name = "novatel AP10",
      message = "OK"))
    rospy.Subscriber(topic_name, topic_type, self._cb)
    self.pub = rospy.Publisher("/diagnostics", DiagnosticArray, latch=True)

  def _cb(self, msg):
    self.status_msg.status[0].values = []
    for field, field_flags in self.flags:
      field_value = getattr(msg, field)
      for mask, flag in field_flags:
        value = str(int((field_value & mask) != 0))
        self.status_msg.status[0].values.append(KeyValue(flag, value))
    self.pub.publish(self.status_msg) 


def main():
  rospy.init_node('novatel_diagnostics')
  BitfieldRepublisher("status/general", novatel_msgs.msg.GeneralStatus, \
                      ('status_a', 'status_b', 'status_c', 'fdir_1', \
                       'fdir_2', 'fdir_3', 'fdir_4', 'fdir_5', 'extended'))
  rospy.spin()
