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

import rospy
import tf
import geodesy.utm

from novatel_msgs.msg import *
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Twist

# Other
from math import radians, pow

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


class NovatelPublisher(object):
    """ Subscribes to the directly-translated messages from the SPAN system
        and repackages the resultant data as standard ROS messages. """

    def __init__(self):
        # Parameters
        self.publish_tf = rospy.get_param('~publish_tf', False)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom_combined')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')

        # When True, UTM odom x, y pose will be published with respect to the
        # first coordinate received.
        self.zero_start = rospy.get_param('~zero_start', False)

        self.imu_rate = rospy.get_param('~rate')

        # Topic publishers
        self.pub_imu = rospy.Publisher('imu/data', Imu, queue_size=1)
        self.pub_odom = rospy.Publisher('navsat/odom', Odometry, queue_size=1)
        self.pub_origin = rospy.Publisher('navsat/origin', Pose, queue_size=1)
        self.pub_navsatfix = rospy.Publisher('navsat/fix', NavSatFix, queue_size=1)

        if self.publish_tf:
            self.tf_broadcast = tf.TransfromBroadcaster()

        self.init = False       # If we've been initialized
        self.origin = Point()   # Where we've started
        self.orientation = [0] * 4  # Empty quaternion until we hear otherwise
        self.orientation_covariance = IMU_ORIENT_COVAR

        # Subscribed topics
        rospy.Subscriber('novatel_data/bestpos', BESTPOSB, self.bestpos_handler)
        rospy.Subscriber('novatel_data/corrimudata', CORRIMUDATA, self.corrimudata_handler)
        rospy.Subscriber('novatel_data/inscov', INSCOV, self.inscov_handler)
        rospy.Subscriber('novatel_data/inspvax', INSPVAX, self.inspvax_handler)

    def bestpos_handler(self, bestpos):
        navsat = NavSatFix()

        # TODO: The timestamp here should come from SPAN, not the ROS system time.
        navsat.header.stamp = rospy.Time.now()
        navsat.header.frame_id = self.odom_frame

        # Assume GPS - this isn't exposed
        navsat.status.service = NavSatStatus.SERVICE_GPS

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
        navsat.status.status = solution_map.get(bestpos.pos_type, NavSatStatus.STATUS_NO_FIX)

        # Position in degrees.
        navsat.latitude = bestpos.latitude
        navsat.longitude = bestpos.longitude

        # Altitude in metres.
        navsat.altitude = bestpos.altitude
        navsat.position_covariance[0] = pow(2, bestpos.lat_std)
        navsat.position_covariance[4] = pow(2, bestpos.lon_std)
        navsat.position_covariance[8] = pow(2, bestpos.hgt_std)
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # Ship it.
        self.pub_navsatfix.publish(navsat)

    def inspvax_handler(self, inspvax):
        # Convert the latlong to x,y coordinates and publish an Odometry
        try:
            utm_pos = geodesy.utm.fromLatLong(inspvax.latitude, inspvax.longitude)
        except ValueError:
            # Probably coordinates out of range for UTM conversion.
            return

        if not self.init and self.zero_start:
            self.origin.x = utm_pos.easting
            self.origin.y = utm_pos.northing
            self.pub_origin.publish(self.origin)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = utm_pos.easting - self.origin.x
        odom.pose.pose.position.y = utm_pos.northing - self.origin.y
        odom.pose.pose.position.z = inspvax.altitude

        # Orientation
        # Save this on an instance variable, so that it can be published
        # with the IMU message as well.
        self.orientation = tf.transformations.quaternion_from_euler(
            radians(inspvax.roll),
            radians(inspvax.pitch),
            radians(inspvax.azimuth))
        odom.pose.pose.orientation = Quaternion(*self.orientation)
        odom.pose.covariance[21] = self.orientation_covariance[0] = pow(2, inspvax.pitch_std)
        odom.pose.covariance[28] = self.orientation_covariance[4] = pow(2, inspvax.roll_std)
        odom.pose.covariance[35] = self.orientation_covariance[8] = pow(2, inspvax.azimuth_std)

        # Twist is relative to vehicle frame
        odom.twist.twist.linear.x = inspvax.velx
        odom.twist.twist.linear.y = inspvax.vely
        odom.twist.twist.linear.z = inspvax.velz
        TWIST_COVAR[0] = pow(2, inspvax.velx_std)
        TWIST_COVAR[7] = pow(2, inspvax.vely_std)
        TWIST_COVAR[14] = pow(2, inspvax.velz_std)
        #odom.twist.twist.angular = imu.angular_velocity
        odom.twist.covariance = TWIST_COVAR

        self.pub_odom.publish(odom)

        # Odometry transform (if required)
        if self.publish_tf:
            self.tf_broadcast.sendTransform(
                (odom.pose.pose.position.x, odom.pose.pose.position.y,
                 odom.pose.pose.position.z),
                odom.pose.pose.orientation,
                odom.header.stamp, odom.child_frame_id, odom.frame_id)

        # Mark that we've received our first fix, and set origin if necessary.
        self.init = True

    def corrimudata_handler(self, corrimudata):
        # TODO: Work out these covariances properly. Logs provide covariances in local frame, not body
        imu = Imu()
        imu.header.stamp == rospy.Time.now()
        imu.header.frame_id = self.base_frame

        # Populate orientation field with one from inspvax message.
        imu.orientation = Quaternion(*self.orientation)
        imu.orientation_covariance = self.orientation_covariance

        # Angular rates (rad/s)
        # corrimudata log provides instantaneous rates so multiply by IMU rate in Hz
        imu.angular_velocity.x = corrimudata.pitch_rate * self.imu_rate
        imu.angular_velocity.y = corrimudata.roll_rate * self.imu_rate
        imu.angular_velocity.z = corrimudata.yaw_rate * self.imu_rate
        imu.angular_velocity_covariance = IMU_VEL_COVAR

        # Linear acceleration (m/s^2)
        imu.linear_acceleration.x = corrimudata.x_accel * self.imu_rate
        imu.linear_acceleration.y = corrimudata.y_accel * self.imu_rate
        imu.linear_acceleration.z = corrimudata.z_accel * self.imu_rate
        imu.linear_acceleration_covariance = IMU_ACCEL_COVAR

        self.pub_imu.publish(imu)


    def inscov_handler(self, inscov):
        # TODO: Supply this data in the IMU and Odometry messages.
        pass


