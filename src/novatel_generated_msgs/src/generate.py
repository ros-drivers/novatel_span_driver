#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
# 
#  file      @generate.py
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

import imp
from os import path
import roslib; roslib.load_manifest('novatel_generated_msgs')
#roslib.load_manifest('novatel_msgs')
from roslib.packages import get_pkg_dir
msgs_dir = get_pkg_dir('novatel_msgs')
msgs_filename = path.join(msgs_dir, "src", "mapping.py")
mapping = imp.load_source('msgs', msgs_filename)

from sys import stdout

pkg_dir = get_pkg_dir('novatel_generated_msgs')
output = []

all_msgs_filename = path.join(pkg_dir, "msg", "AllMsgs.msg")
with open(all_msgs_filename, "w") as all_msgs_f:
  all_msgs_f.write("time last_changed\n")
  all_msgs_f.write("time last_sent\n")
  for msg_num in mapping.msgs.keys():
    msg_tuple = mapping.msgs[msg_num]
    if msg_tuple:
      name, msg_cls = msg_tuple
      if msg_cls.in_all_msgs: 
        all_msgs_f.write("novatel_msgs/%s %s\n" % (msg_cls.__name__, name))

      if name != "ack":
        msg_srv_filename = path.join(pkg_dir, "srv", "%s.srv" % msg_cls.__name__)
        output.append("%s.srv" % msg_cls.__name__)
        with open(msg_srv_filename, "w") as msg_srv_f:
          msg_srv_f.write("novatel_msgs/%s request\n" % msg_cls.__name__)
          msg_srv_f.write("---\n")
          msg_srv_f.write("novatel_msgs/Ack ack")

stdout.write(";".join(output))
