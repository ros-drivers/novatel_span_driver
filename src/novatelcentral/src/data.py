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
#  Copyright © 2014 NovAtel, Inc. 
#  All Rights Reserved
#  Modifications to support NovAtel messages
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

# ROS messages
import novatel_msgs.msg
import novatel_generated_msgs.msg
from novatel_generated_msgs.msg import AllMsgs

# Node source
from port import Port
from mapping import msgs
from handlers import GroupHandler, MessageHandler
import translator

# Python
from cStringIO import StringIO
from threading import Lock

def all_same(dict_):
    # return true if all of the values in this dict are the same 
    # ignores those with values == 0
    vls = dict_.values()
    j = 0
    while j < len(vls) and vls[j] == 0:
      j+=1

    for i in range(j+1, len(vls)):
      if vls[j] != vls[i] and vls[i] != 0 and vls[i] != None:
        return False

    return True 

class DataPort(Port):
  ALLMSGS_SEND_TIMEOUT = rospy.Duration.from_sec(0.05)

  def run(self):
    # Aggregate message for republishing the sensor config as a single blob.
    all_msgs = AllMsgs()
    all_msgs_pub = rospy.Publisher("config", all_msgs.__class__, latch=True) 

    # Listener object which tracks what topics have been subscribed to.
    listener = SubscribeListenerManager()
  
    # Set up handlers for translating different novatel messages as they arrive.
    handlers = {}
    pkt_counters = {}
    pkt_times = {}

    for msg_num in msgs.keys():
      handlers[(novatel_msgs.msg.CommonHeader.START_MESSAGE, msg_num)] = \
          MessageHandler(*msgs[msg_num], all_msgs=all_msgs)
      pkt_counters[(novatel_msgs.msg.CommonHeader.START_MESSAGE, msg_num)] = 0
      pkt_times[(novatel_msgs.msg.CommonHeader.START_MESSAGE, msg_num)] = 0

    bad_pkts = set()
    pkt_id = None

    while not self.finish.is_set():
      try:
        pkt_id, pkt_str, pkt_time = self.recv()
        if pkt_id != None:
          handlers[pkt_id].handle(StringIO(pkt_str))

      except ValueError as e:
        # Some problem in the recv() routine.
        rospy.logwarn(str(e))
        continue

      except KeyError as e:
        # No handler for this pkt_id. Only warn on the first sighting.
        print("KEYERROR")
        rospy.logwarn(str(e))

        if pkt_id not in handlers:
          rospy.logwarn("Uninitialised Handler")
          handlers[pkt_id] = MessageHandler(*msgs[msg_num], all_msgs=all_msgs)

        if pkt_id not in pkt_counters:
            rospy.logwarn("Unhandled packet")

      except translator.TranslatorError:
        if pkt_id not in bad_pkts:
          rospy.logwarn("Error parsing %s.%d" % pkt_id)
          bad_pkts.add(pkt)

      if pkt_id not in pkt_counters:
        pkt_counters[pkt_id] = 0  
      else:
        pkt_counters[pkt_id] += 1
        pkt_times[pkt_id] = pkt_time # only track times of msgs that are part of novatel msgs

      # wait until all the msgs have the same GNSS time before sending
      if all_same(pkt_times):
        all_msgs_pub.publish(all_msgs)
        all_msgs.last_sent = rospy.get_rostime() 


class SubscribeListenerManager():
  def __init__(self):
    self.lock = Lock()
    self.groups = set()
    self.publisher = rospy.Publisher("subscribed_groups", novatel_msgs.msg.Groups, latch=True)
    self.publish()

  def publish(self):
    self.publisher.publish(groups=list(self.groups))

  def listener_for(self, group_num):
    return self.Listener(self, group_num)

  class Listener(rospy.SubscribeListener):
    def __init__(self, manager, group_num):
      self.manager = manager
      self.group_num = group_num

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
      with self.manager.lock:
        self.manager.groups.add(self.group_num)
        self.manager.publish()

    def peer_unsubscribe(self, topic_name, num_peers):
      if num_peers == 0:
        with self.lock:
          self.manager.groups.discard(self.group_num)
          self.manager.publish()
