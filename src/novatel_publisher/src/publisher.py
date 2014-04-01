#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
# 
#  file      @publisher.py
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
import roslib; roslib.load_manifest('novatel_publisher')
import rospy
import tf
# no success installing KDL yet
#import pykdl_utils.kdl_parser #PyKDL

# novatel node internal messages & modules
from novatel_msgs.msg import * 
from novatel_generated_msgs.msg import AllMsgs
from gps_utm import LLtoUTM

# ROS standard messages
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Twist

# Other
from math import radians as RAD
from math import degrees as DEG
from math import pow as POW

# FIXED COVARIANCES
# TODO: Work these out...
IMU_ORIENT_COVAR = [1e-3, 0, 0,
                    0, 1e-3, 0,
                    0, 0, 1e-3]

IMU_VEL_COVAR = [1e-3, 0, 0,
                 0, 1e-3, 0,
                 0, 0, 1e-3]

IMU_ACCEL_COVAR = [1e-3, 0, 0,
                   0, 1e-3, 0,
                   0, 0, 1e-3]

NAVSAT_COVAR = [1, 0, 0,
                0, 1, 0,
                0, 0, 1]

POSE_COVAR = [1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0,
              0, 0, 0, 0.1, 0, 0,
              0, 0, 0, 0, 0.1, 0,
              0, 0, 0, 0, 0, 0.1]

TWIST_COVAR = [1, 0, 0, 0, 0, 0,
               0, 1, 0, 0, 0, 0,
               0, 0, 1, 0, 0, 0,
               0, 0, 0, 0.1, 0, 0,
               0, 0, 0, 0, 0.1, 0,
               0, 0, 0, 0, 0, 0.1]

class novatelPublisher(object):

    def __init__(self):
        rospy.init_node('novatel_publisher')

        # Parameters
        self.publish_tf = rospy.get_param('~publish_tf', False)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom_combined')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.zero_start = rospy.get_param('~zero_start', False) # If this is True, UTM will be pub'd wrt. our first recv'd coordinate

        self.keep_az = 0 # want delta az not abs
        self.init_az = False # have we initialised the azimuth

        self.imu_rate = rospy.get_param('rate')
        # IMU scale factors, needed for RAWIMU log only
        self.imu_scale = { 'gyro':RAD(720.0/pow(2.0,31.0)), 'accel': 200.0/pow(2.0,31.0) } # ADIS16488

        # Topic publishers
        self.pub_imu = rospy.Publisher('imu_data', Imu)
        self.pub_odom = rospy.Publisher('gps_odom', Odometry)
        self.pub_origin = rospy.Publisher('origin', Pose)
        self.pub_navsatfix = rospy.Publisher('gps_fix', NavSatFix)
        self.pub_navsatstatus = rospy.Publisher('gps_status', NavSatStatus)
        #self.pub_geomtwist = rospy.Publisher('cmd_vel', Twist)

        if self.publish_tf:
            self.tf_broadcast = tf.TransfromBroadcaster()

        # Init nav status
        self.nav_status = NavSatStatus()    # We need this for the NavSatFix broadcaster
        self.nav_status.status = NavSatStatus.STATUS_NO_FIX
        self.nav_status.service = NavSatStatus.SERVICE_GPS

        self.init = False       # If we've been initialized
        self.origin = Point()   # Where we've started
        
        # Subscribed topics
        all_msgs = AllMsgs()
        rospy.Subscriber('config', all_msgs.__class__, self.everything_handler)
    

    def everything_handler(self, data):
      self.status_handler(data.bestpos)
      self.navigation_handler(data)
      # self.turtle_handler(data.rawimu) # for demo

    def turtle_handler(self, data):
      # specifically for demonstrating control of turtlesim using IMU
      twist = Twist()
      twist.linear.x = -data.accy*self.imu_scale['accel']
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0
      twist.angular.y = 0
      twist.angular.z = data.gyrz*self.imu_scale['gyro']

      self.pub_geomtwist.publish(twist)

    def navigation_handler(self, data):
        """ Rebroadcasts navigation data in the following formats:
        1) /odom => /base footprint transform (if enabled, as per REP 105)
        2) Odometry message, with parent/child IDs as in #1
        3) NavSatFix message, for systems which are knowledgeable about GPS stuff
        4) IMU messages
        """
        rospy.logdebug("Navigation received")
        # If we don't have a fix, don't publish anything...
        if self.nav_status.status == NavSatStatus.STATUS_NO_FIX:
            return
        
        # UTM conversion
        #(zone, easting, northing) = LLtoUTM(23, data.inspvax.latitude, data.inspvax.longitude)
        easting, northing = data.bestxyz.easting, data.bestxyz.northing 
        # Initialize starting point if we haven't yet
        # TODO: Do we want to follow UTexas' lead and reinit to a nonzero point within the same UTM grid?
        # TODO check INSPVAX sol stat for valid position before accepting 
        if not self.init and self.zero_start:
            self.origin.x = easting
            self.origin.y = northing
            self.init = True

        # Publish origin reference for others to know about
        p = Pose()
        p.position.x = self.origin.x
        p.position.y = self.origin.y
        self.pub_origin.publish(p) 

        # IMU
        # TODO: Work out these covariances properly. Logs provide covariances in local frame, not body
        #
        imu = Imu()
        imu.header.stamp == rospy.Time.now()
        imu.header.frame_id = self.base_frame
      
        # Orientation
        # orient=PyKDL.Rotation.RPY(RAD(data.roll),RAD(data.pitch),RAD(data.heading)).GetQuaternion() 
        # imu.orientation = Quaternion(*orient)
        imu.orientation.x = data.inspvax.pitch
        imu.orientation.y = data.inspvax.roll
        imu.orientation.z = data.inspvax.azimuth
        imu.orientation.w = 0
        IMU_ORIENT_COVAR[0] = POW(2,data.inspvax.pitch_std)
        IMU_ORIENT_COVAR[4] = POW(2,data.inspvax.roll_std)
        IMU_ORIENT_COVAR[8] = POW(2,data.inspvax.azimuth_std)
        imu.orientation_covariance = IMU_ORIENT_COVAR
      
        # Angular rates
        # corrimudata log provides instantaneous rates so multiply by IMU rate in Hz
        imu.angular_velocity.x = DEG(data.corrimudata.pitch_rate*self.imu_rate)
        imu.angular_velocity.y = DEG(data.corrimudata.roll_rate*self.imu_rate)
        imu.angular_velocity.z = DEG(data.corrimudata.yaw_rate*self.imu_rate)
        imu.angular_velocity_covariance = IMU_VEL_COVAR
      
        # Linear acceleration
        imu.linear_acceleration.x = data.corrimudata.x_accel*self.imu_rate
        imu.linear_acceleration.y = data.corrimudata.y_accel*self.imu_rate
        imu.linear_acceleration.z = data.corrimudata.z_accel*self.imu_rate
        imu.linear_acceleration_covariance = IMU_ACCEL_COVAR

        self.pub_imu.publish(imu)


        #
        # Odometry 
        # TODO: Work out these covariances properly
        #
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = easting  - self.origin.x
        odom.pose.pose.position.y = northing - self.origin.y
        odom.pose.pose.position.z = data.inspvax.altitude
        #odom.pose.pose.orientation = Quaternion(*orient)
        odom.pose.pose.orientation = imu.orientation
        POSE_COVAR[21] = IMU_ORIENT_COVAR[0]
        POSE_COVAR[28] = IMU_ORIENT_COVAR[4]
        POSE_COVAR[35] = IMU_ORIENT_COVAR[8]
        odom.pose.covariance = POSE_COVAR

        # Twist is relative to vehicle frame
        odom.twist.twist.linear.x = data.bestxyz.velx
        odom.twist.twist.linear.y = data.bestxyz.vely
        odom.twist.twist.linear.z = data.bestxyz.velz
        TWIST_COVAR[0]  = pow(2,data.bestxyz.velx_std)
        TWIST_COVAR[7]  = pow(2,data.bestxyz.vely_std)
        TWIST_COVAR[14] = pow(2,data.bestxyz.velz_std)
        odom.twist.twist.angular = imu.angular_velocity
        odom.twist.covariance = TWIST_COVAR

        self.pub_odom.publish(odom)

        #
        # Odometry transform (if required)
        #
        if self.publish_tf:
            self.tf_broadcast.sendTransform(
                (odom.pose.pose.position.x, odom.pose.pose.position.y,
                 odom.pose.pose.position.z), odom.pose.pose.orientation, #Quaternion(*orient),
                 odom.header.stamp,odom.child_frame_id, odom.frame_id)

        # 
        # NavSatFix
        # TODO: Work out these covariances properly from DOP
        #
        navsat = NavSatFix()
        navsat.header.stamp = rospy.Time.now()
        navsat.header.frame_id = self.odom_frame
        navsat.status = self.nav_status
        # position, in degrees
        navsat.latitude  = data.bestpos.latitude
        navsat.longitude = data.bestpos.longitude
        navsat.altitude  = data.bestpos.altitude
        NAVSAT_COVAR[0] = pow(2,data.bestpos.lat_std) # in meters
        NAVSAT_COVAR[4] = pow(2,data.bestpos.lon_std)
        NAVSAT_COVAR[8] = pow(2,data.bestpos.hgt_std)

        navsat.position_covariance = NAVSAT_COVAR
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.pub_navsatfix.publish(navsat)
        
        pass

    def status_handler(self, data):
        """ Rebroadcasts GNSS status as a standard NavSatStatus message """
        solution_map = {
            BESTPOSB.NONE: NavSatStatus.STATUS_NO_FIX,
            BESTPOSB.FIXED: NavSatStatus.STATUS_FIX,
            BESTPOSB.FIXEDHEIGHT: NavSatStatus.STATUS_FIX,
            BESTPOSB.FLOATCONV: NavSatStatus.STATUS_FIX,
            BESTPOSB.WIDELANE: NavSatStatus.STATUS_FIX,
            BESTPOSB.NARROWLANE: NavSatStatus.STATUS_FIX,
            BESTPOSB.DOPPLER_VELOCITY: NavSatStatus.STATUS_FIX,
            BESTPOSB.SINGLE: NavSatStatus.STATUS_FIX,
            BESTPOSB.PSRDIFF: NavSatStatus.STATUS_GBAS_FIX, 
            BESTPOSB.WAAS: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.PROPOGATED: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.OMNISTAR: NavSatStatus.STATUS_SBAS_FIX,
            BESTPOSB.L1_FLOAT: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.IONOFREE_FLOAT: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.NARROW_FLOAT: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.L1_INT: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.WIDE_INT: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.NARROW_INT: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.RTK_DIRECT_INS: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.INS_SBAS: NavSatStatus.STATUS_SBAS_FIX,
            BESTPOSB.INS_PSRSP: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.INS_PSRDIFF: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.INS_RTKFLOAT: NavSatStatus.STATUS_GBAS_FIX,
            BESTPOSB.INS_RTKFIXED: NavSatStatus.STATUS_GBAS_FIX,
            }

        self.nav_status.status = solution_map.get(data.pos_type,NavSatStatus.STATUS_NO_FIX)

        # Assume GPS - this isn't exposed
        self.nav_status.service = NavSatStatus.SERVICE_GPS
            
        self.pub_navsatstatus.publish(self.nav_status)

def main():
  node = novatelPublisher()
  rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
