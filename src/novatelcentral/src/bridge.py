#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
# 
#  file      @bridge.py
#  authors   Mike Purvis <mpurvis@clearpathrobotics.com>
#            NovAtel <novatel.com/support>
#  copyright Copyright (c) 2012, Clearpath Robotics, Inc., All rights reserved.
#            Copyright (c) 2014, NovAtel Inc., All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#    following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
#    following disclaimer in the documentation and/or other materials provided with the distribution.
#  * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
# RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
# DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
# OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ROS
import rospy
from novatel_msgs.msg import *

# Package modules
from data import DataPort
from control import ControlPort
from monitor import Monitor

# Standard
import socket
import serial
import struct
from cStringIO import StringIO
import time

import translator
from handlers import NullHandler, GroupHandler, MessageHandler, AckHandler

from std_msgs.msg import String

PORTS_DATA = {
    "realtime": 3001,
    "logging": 3001
    }
PORT_CONTROL = 3001

DEFAULT_IP = '198.161.73.9'
PREFIX_DEFAULTS = {
    "raw": True,
    "dmi": True,
    "status": True,
    "events": True
    }

SOCKET_TIMEOUT=100.0
socks = []
ports = {}
monitor = Monitor(ports)


def main():

  print("starting bridge");
  rospy.init_node('novatel_bridge')

  # Where to find the device. It does not support DHCP, so you'll need
  # to connect initially to the factory default IP in order to change it.
  ip = rospy.get_param('ip', DEFAULT_IP)

  # Select between realtime and logging data ports. The logging data is
  # optimized for completeness, whereas the realtime data is optimized for
  # freshness.
  data_port = PORTS_DATA[rospy.get_param('data', "realtime")]

  # Disable this to not connect to the control socket (for example, if you
  # want to control the device using novatel software rather
  # than ROS-based services.
  control_enabled = rospy.get_param('control', True)

  rospy.loginfo("Inside Bridge")

  # Disable any of these to hide auxiliary topics which you may not be
  # interested in.
  exclude_prefixes = []
  for prefix, default in PREFIX_DEFAULTS.items():
    if not rospy.get_param('include_%s' % prefix, default):
      exclude_prefixes.append(prefix)

  # Pass this parameter to use pcap data rather than a socket to a device.
  # For testing the node itself--to exercise downstream algorithms, use a bag.
  pcap_file_name = rospy.get_param('pcap_file', False)

  if not pcap_file_name:
    # To connect to serial instead of ethernet, just change the comments here
    sock = create_sock('data', ip, data_port)
    #sock = create_serial('data', 115200, '/dev/ttyS0')
  else:
    sock = create_test_sock(pcap_file_name)

  ports['data'] = DataPort(sock, exclude_prefixes=exclude_prefixes)

  if control_enabled:
    ports['control'] = ControlPort(create_sock('control', ip, PORT_CONTROL))

  configure_receiver(sock)

  for name, port in ports.items():
    port.start()
    rospy.loginfo("Port %s thread started." % name)
  monitor.start()

  rospy.on_shutdown(shutdown)
  rospy.spin()

def create_usb(name, port_):
  # TODO: this hasnt been tested
  usb = serial.Serial(
     port=port_,
     timeout=SOCKET_TIMEOUT)

  if usb.isOpen():
    usb.close()

  try:
    usb.open()
    rospy.loginfo("Successfully connected to %%s port at %s" % port_ % name)
  except usb.ValueError as e:
    rospy.logfatal("%s" % str(e))
  except serial.SerialException as e:
    rospy.logfatal("Couldn't connect usb: %s: %%s" % port_ % str(e))

  socks.append(usb)
  return usb

def create_serial(name, rate_, port_):    
  ser = serial.Serial(
     port=port_,
	  baudrate=rate_,
	  parity=serial.PARITY_NONE,
     bytesize=serial.EIGHTBITS,
	  stopbits=serial.STOPBITS_ONE,
     timeout=SOCKET_TIMEOUT/100,
     rtscts=False,
     dsrdtr=False,
     xonxoff=False)
 
  if ser.isOpen():
    ser.close()

  try:
    ser.open()
    rospy.loginfo("Successfully connected to %%s port at %s" % port_ % name)
  except serial.ValueError as e:
    rospy.logfatal("%s" % str(e))
  except serial.SerialException as e:
    rospy.logfatal("Couldn't connect serial: %s: %%s" % port_ % str(e))

  ser.flushInput()
  socks.append(ser)
  return ser

def create_sock(name, ip, port):
  try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ip_port = (ip, port)
    sock.connect(ip_port)
    rospy.loginfo("Successfully connected to %%s port at %s:%d" % ip_port % name)
  except socket.error as e:
    rospy.logfatal("Couldn't connect to %%s port at %s:%d: %%s" % ip_port % (name, str(e)))
    exit(1)
  sock.settimeout(SOCKET_TIMEOUT)
  socks.append(sock)
  return sock


def create_test_sock(pcap_filename):
  rospy.sleep(0.1)

  import pcapy
  from StringIO import StringIO
  from impacket import ImpactDecoder

  body_list = []
  cap = pcapy.open_offline(pcap_filename)
  decoder = ImpactDecoder.EthDecoder()

  while True:
    header, payload = cap.next()
    if not header: break
    udp = decoder.decode(payload).child().child()
    body_list.append(udp.child().get_packet())

  data_io = StringIO(''.join(body_list))

  class MockSocket(object):
    def recv(self, byte_count):
      rospy.sleep(0.0001)
      data = data_io.read(byte_count)
      if data == "":
        rospy.signal_shutdown("Test completed.")
      return data  
    def settimeout(self, timeout):
      pass

  return MockSocket()

def configure_receiver(port):
  receiver_config = rospy.get_param('configuration')
  if receiver_config != None:

    logger = receiver_config['log_request']
    imu_connect = receiver_config['imu_connect']
    commands = receiver_config['command']
    
    if type(port) == type(serial.Serial()):
      put = port.write
    else:
      put = port.send

    if imu_connect != None:
      put('connectimu '+imu_connect['port']+' '+imu_connect['type'] + '\r\n')

    for log in logger:
      put('log ' + log + ' ontime ' + str(logger[log]) + '\r\n')

    for cmd in commands:
      put(cmd + ' ' + commands[cmd] + '\r\n')


def shutdown():
  monitor.finish.set()
  monitor.join()
  rospy.loginfo("Thread monitor finished.") 
  for name, port in ports.items():
    port.finish.set()
    port.join()
    rospy.loginfo("Port %s thread finished." % name) 
  for sock in socks:
    if type(sock) != type(serial.Serial()):
      sock.shutdown(socket.SHUT_RDWR)
    sock.close()
  rospy.loginfo("Sockets closed.") 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
