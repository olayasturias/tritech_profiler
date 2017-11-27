#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tritech Profiler sonar scanner.

This publishes one PointCloud message per scan slice. In order to visualize in
rviz, play with the 'Decay Time' parameter. This node also provides parameters
that can be dynamically reconfigured.
"""

import rospy
from sensor_msgs.msg import PointCloud, LaserScan
from tritech_profiler import TritechProfiler
from geometry_msgs.msg import PoseStamped
from tritech_profiler.cfg import ScanConfig
from dynamic_reconfigure.server import Server
from tritech_profiler.msg import TritechMicronConfig

__author__ = "Anass Al-Wohoush, Olaya Alvarez"

"""
.. codeauthor:: Olaya Alvarez Tunon
: file scan.py
"""

def reconfigure(config, level):
    """Reconfigures sonar dynamically.

    Args:
        config: New configuration.
        level: Level bitmask.

    Returns:
        Configuration.
    """
    rospy.loginfo("Reconfiguring sonar")
    rospy.logdebug("Configuration requested: %r, %r", config, level)

    # Remove additional keys.
    if "groups" in config:
        config.pop("groups")

    # Set parameters.
    print 'llamo a set y pongo lo que me sale de ahí'
    sonar.set(**config)
    return config


def publish(sonar, slice):
    """Publishes PointCloud, PoseStamped and TritechMicronConfig of current
    scan slice on callback.

    Args:
        sonar: Sonar instance.
        slice: Current scan slice.
    """

    # Publish heading as PoseStamped.
    posestamped = slice.to_posestamped(frame)
    heading_pub.publish(posestamped)

    # Publish data as PointCloud.

    cloud = slice.to_pointcloud(frame)
    scan_pub.publish(cloud)

    # Publish data as LaserScan.

    scan = slice.to_laserscan(frame)
    laser_pub.publish(scan)


    # Publish data as TritechMicronConfig.
    config = slice.to_config(frame)
    conf_pub.publish(config)


if __name__ == "__main__":
    # Initialize node and publishers.
    rospy.init_node("tritech_profiler",log_level=rospy.DEBUG)
    scan_pub    = rospy.Publisher("~scan", PointCloud, queue_size=800)
    laser_pub   = rospy.Publisher("~singlescan", LaserScan, queue_size=800)
    heading_pub = rospy.Publisher("~heading", PoseStamped, queue_size=800)
    conf_pub    = rospy.Publisher("~config", TritechMicronConfig, queue_size=800)

    # Get frame name and port.
    frame = rospy.get_param("~frame")
    port = rospy.get_param("~port")

    with TritechProfiler(port=port) as sonar:
        try:
            # Initialize dynamic reconfigure server and scan.
            Server(ScanConfig, reconfigure)

            # Scan.
            sonar.scan(callback=publish)
        except KeyboardInterrupt:
            sonar.preempt()
