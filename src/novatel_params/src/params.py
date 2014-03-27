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
import roslib.message
import novatel_msgs.msg
import novatel_generated_msgs.srv 

response_codes = dict([(val, name) for name, val in novatel_msgs.msg.Ack.__dict__.items() if name.startswith("RESPONSE_")])


def main():
  rospy.init_node("novatel_params")

  com_ports = rospy.get_param('com_ports', None) 
  if com_ports != None:
    req_msg = novatel_msgs.msg.COMPortSetup()
    for port_num, port_params in enumerate(com_ports):
      # 8N1 is pretty universal now; less need to parameterize it. 
      port_msg = novatel_msgs.msg.COMPortParams()
      port_msg.parity = port_msg.PARITY_NONE
      port_msg.data_stop = port_msg.DATA_8_STOP_1
      port_msg.flow = port_msg.FLOW_NONE
      port_msg.baud = getattr(port_msg, "BAUD_%s" % port_params['baud'])
      port_msg.input_select = getattr(port_msg, "INPUT_%s" % port_params['input'])
      port_msg.output_select = getattr(port_msg, "OUTPUT_%s" % port_params['output'])
      req_msg.port_mask |= 2 << port_num
      req_msg.ports.append(port_msg)

    call_novatel_service("com_port_setup", req_msg)
    rospy.loginfo("Configured COM ports.")

  base_gnss = rospy.get_param('base_gnss', None) 
  if base_gnss != None:
    for base_num, base_params in enumerate(base_gnss):
      base_msg = novatel_msgs.msg.BaseGNSSSetup()
      base_msg.base_gnss_input_type = getattr(base_msg, "TYPE_%s" % base_params['type'])	
      base_msg.datum = getattr(base_msg, "DATUM_%s" % base_params['datum'])	
      call_novatel_service("base_gnss_%i_setup" % (base_num + 1), base_msg)
      rospy.loginfo("Configured base GNSS #%i." % (base_num + 1))

  geometry = rospy.get_param('geometry', None) 
  if geometry != None:
    req_msg = novatel_msgs.msg.GeneralParams()
    for vector_name in ['imu_lever_arm', 'primary_gnss_lever_arm', 'imu_mounting_angle', 'ref_mounting_angle']:
      if vector_name in geometry:
        vector = geometry[vector_name]
        getattr(req_msg, vector_name).x = vector['x']
        getattr(req_msg, vector_name).y = vector['y']
        getattr(req_msg, vector_name).z = vector['z']
    req_msg.time_types = 0x1
    req_msg.distance_type = 1
    req_msg.autostart = 1 
    req_msg.multipath = 0  # LOW setting, do not change. 
    call_novatel_service('general', req_msg)
    rospy.loginfo("Configured geometry.")

  # Default rate of 10Hz
  rate = rospy.get_param('rate', 10)
  rospy.Subscriber("subscribed_groups", novatel_msgs.msg.Groups, groups_callback)

  # Delay setting the sensor override msg until we have received notice that Fine Align is active.
  sensor_overrides = rospy.get_param('sensor_overrides', None) 
  if sensor_overrides != None:
    override_msg = novatel_msgs.msg.AidingSensorIntegrationControl()
    for override_str in sensor_overrides:
      override_msg.override |= getattr(override_msg, "OVERRIDE_%s" % override_str)
    rospy.loginfo("Waiting on Fine Align before configuring sensor overrides.")
    proceed = [False]
    def _cb(msg):
      if msg.status_a & novatel_msgs.msg.GeneralStatus.STATUS_A_FINE_ALIGN_ACTIVE != 0:
        proceed[0] = True
    sub = rospy.Subscriber("status/general", novatel_msgs.msg.GeneralStatus, _cb)
    while not proceed[0]:
      rospy.sleep(1.)
    sub.unregister()
    call_novatel_service("aiding_sensor_integration", override_msg)
    rospy.loginfo("Configured sensor overrides.")

  rospy.spin()
 


def call_novatel_service(name, req):
  service_defn = getattr(novatel_generated_msgs.srv, req.__class__.__name__)
  rospy.wait_for_service(name)
  ack = rospy.ServiceProxy(name, service_defn)(req).ack
  if ack.response_code != novatel_msgs.msg.Ack.RESPONSE_ACCEPTED:
    rospy.logwarn("Parameter change call to %s resulted in error code %d (%s)." %
                  (name, ack.response_code, response_codes[ack.response_code]))
  return ack


def groups_callback(message):
  req_msg = novatel_msgs.msg.PortControl()
  req_msg.rate = rospy.get_param('rate', 10)
  groups = set(message.groups)
  groups.add(10)
  for group_num in groups:
    req_msg.groups.append(novatel_msgs.msg.OutputGroup(group=group_num))
  call_novatel_service("primary_data_port", req_msg)
  #print message.groups
