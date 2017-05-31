#!/usr/bin/env python

"""Tritech Micron sonar ROS package setup."""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

__author__ = "Olaya Alvarez"

d = generate_distutils_setup(
    packages=["tritech_profiler"],
    package_dir={"": "src"}
)

setup(**d)
