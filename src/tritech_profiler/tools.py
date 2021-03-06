# -*- coding: utf-8 -*-

"""Tritech Profiler sonar tools."""

import math
import rospy
#from tritech_profiler.msg import TritechMicronConfig
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import ChannelFloat32, PointCloud, LaserScan
from geometry_msgs.msg import Point32, Pose, PoseStamped, Quaternion

__author__ = "Anass Al-Wohoush, Olaya Alvarez"

"""
.. codeauthor:: Olaya Alvarez Tunon
: file tools.py
"""

def to_sonar_angles(rad):
    """Converts radians to units of 1/16th of a gradian.

    Args:
        rad: Angle in radians.

    Returns:
        Integral angle in units of 1/16th of a gradian.
    """
    return int(rad * 3200 / math.pi) % 6400


def to_radians(angle):
    """Converts units of 1/16th of a gradian to radians.

    Args:
        angle: Angle in units of 1/16th of a gradian.

    Returns:
        Angle in radians.
    """
    return angle / 3200.0 * math.pi


def reconfigured(previous_slice, current_slice):
    """Determines whether the sonar has been reconfigured to the point that all
    upcoming data is incompatible with previous data and cannot be stitched
    together to form an image.

    Args:
        previous_slice: Previous slice.
        current_slice: Current slice.

    Returns:
        True if scan data should be reset due to reconfiguration, False
        otherwise.
    """
    for key in current_slice.config:
        if current_slice.config[key] != previous_slice.config[key]:
            return True

    return False


class ScanSlice(object):

    """
    *Scan slice.*

    Attributes:
        bins: Array of intensities of each return.
        config: Sonar configuration at time of this slice.
        heading: Heading of sonar in radians.
        range: Range of scan in meters.
        timestamp: ROS timestamp.
    """

    def __init__(self, heading, bins, config):
        """Constructs ScanSlice instance.

        Args:
            heading: Heading of sonar in radians.
            bins: Array of intensities of each return.
            config: Sonar configuration at time of this slice.
        """
        self.heading = heading
        self.bins = bins
        self.config = config
        self.range = config["range"]
        self.step = config["step"]
        self.angle_min = config["left_limit"]
        self.angle_max = config["right_limit"]
        self.timestamp = rospy.get_rostime()

    def to_config(self, frame):
        """Returns a TritechMicronConfig message corresponding to slice
        configuration.

        Args:
            frame: Frame ID.

        Returns:
            TritechMicronConfig.
        """
        #config = TritechMicronConfig(**self.config)
        #config.header.frame_id = frame
        #config.header.stamp = self.timestamp

        #return config
        return True

    def to_pointcloud(self, frame):
        """Returns a PointCloud message corresponding to slice.

        Args:
            frame: Frame ID.

        Returns:
            A sensor_msgs.msg.PointCloud.
        """

        # Construct PointCloud message.
        cloud = PointCloud()
        cloud.header.frame_id = frame
        cloud.header.stamp = self.timestamp

        # Convert bins to list of Point32 messages.
        nbins = self.config["nbins"]
        r_step = self.config["step"]
        x_unit = math.cos(self.heading) * r_step
        y_unit = math.sin(self.heading) * r_step

        cloud.points = [
            Point32(x=self.bins[r]*math.cos(r*self.step), y=self.bins[r]*math.sin(r*self.step), z=0.00)
            for r in range(0, nbins)
        ]

        return cloud

    def to_laserscan(self, frame):
        """Returns a LaserScan message correspon2ding to slice.

        Args:
            frame: Frame ID.

        Returns:
            A sensor_msgs.msg.LaserScan.
        """

        scan = LaserScan()
        scan.header.frame_id = frame
        scan.header.stamp = self.timestamp

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.step
        #scan.ranges = self.bins
        scan.ranges = [b*1.45/2000 for b in self.bins]

        # points out if this range are discarded
        scan.range_min = 0.0
        scan.range_max = 70000.0

        return scan

    def to_posestamped(self, frame):
        """Returns a PoseStamped message corresponding to slice heading.

        Args:
            frame: Frame ID.

        Returns:
            A geometry_msgs.msg.PoseStamped.
        """
        # Construct PoseStamped message.
        posestamped = PoseStamped()
        posestamped.header.frame_id = frame
        posestamped.header.stamp = self.timestamp

        # Convert to quaternion.
        q = Quaternion(*quaternion_from_euler(0, 0, self.heading))
        # print '_____________________________________'
        # print frame
        # print '+++++++++++++++++++++++++++++++++++++'
        # print q

        # Make Pose message.
        pose = Pose(orientation=q)
        posestamped.pose = pose
        # print '-------------------------------------'
        # print posestamped

        return posestamped
