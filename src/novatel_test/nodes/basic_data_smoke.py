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

PKG='novatel_test'

import roslib; roslib.load_manifest(PKG)
import rospy

from novatel_msgs.msg import NavigationSolution
from novatel_generated_msgs.msg import AllMsgs
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import NavSatFix, Imu

import rostest
import unittest


class MessageReceiver(object):
  def __init__(self, topic_name, topic_type):
    self.msgs = []
    rospy.Subscriber(topic_name, topic_type, self.cb)

  def cb(self, msg):
    self.msgs.append(msg)


class TestSmoke(unittest.TestCase):
  receivers = {}

  def test_diagnostics(self):
    self.assertTrue(len(self.receivers['diag'].msgs) > 70)
    fields = {}
    for msg in self.receivers['diag'].msgs:
      for field in msg.status[0].values:
        fields[field.key] = field.value
    self.assertEqual(fields['IMU_STATUS'], '0')
    self.assertEqual(fields['PRIMARY_GNSS_IN_CA_MODE'], '1')
    self.assertEqual(fields['INERTIAL_NAVIGATOR_INITIALIZED'], '1')
    self.assertEqual(fields['FULL_NAVIGATION_SOLUTION'], '0')

  def test_navigation(self):
    self.assertTrue(len(self.receivers['nav'].msgs) > 70)
    msg = self.receivers['nav'].msgs[-1]
    self.assertAlmostEqual(msg.latitude, 44.2449408681)
    self.assertAlmostEqual(msg.longitude, -76.5096210157)
    self.assertAlmostEqual(msg.roll, 2.04550977266)
    self.assertAlmostEqual(msg.pitch, 1.3181307736)
    self.assertAlmostEqual(msg.heading, 20.812475085)

  def test_fix(self):
    self.assertTrue(len(self.receivers['fix'].msgs) > 70)
    msg = self.receivers['fix'].msgs[-1]
    self.assertAlmostEqual(msg.latitude, 44.2449408681)
    self.assertAlmostEqual(msg.longitude, -76.5096210157) 

  def test_imu(self):
    self.assertTrue(len(self.receivers['imu'].msgs) > 70)
    msg = self.receivers['imu'].msgs[-1]
    self.assertAlmostEqual(msg.angular_velocity.x, -0.000960592777497)
    self.assertAlmostEqual(msg.linear_acceleration.y, 0.0397075638175) 

  def test_config(self):
    """ Exists primarily to test the correct parsing of complex messages/groups, including
      nested arrays, etc. """
    msg = self.receivers['cfg'].msgs[-1]
    self.assertEqual(len(msg.primary_data_port.groups), 3)
    self.assertEqual(msg.primary_data_port.groups[1].group, 10)
    self.assertEqual(msg.com_port_setup.ports[0].baud, 7)
    self.assertEqual(msg.com_port_setup.ports[0].parity, 0)
    self.assertEqual(msg.com_port_setup.ports[0].data_stop, 2)


if __name__ == '__main__':
  rospy.init_node('novatel_test', anonymous=True)
  TestSmoke.receivers['diag'] = MessageReceiver('/diagnostics', DiagnosticArray)
  TestSmoke.receivers['nav'] = MessageReceiver('nav', NavigationSolution)
  TestSmoke.receivers['fix'] = MessageReceiver('gps_fix', NavSatFix)
  TestSmoke.receivers['imu'] = MessageReceiver('imu_data', Imu)
  TestSmoke.receivers['cfg'] = MessageReceiver('config', AllMsgs)
  rospy.sleep(3.0)

  rostest.rosrun(PKG, 'test_smoke', TestSmoke)

