#!/bin/bash

SESSION=$USER



PYTHONPATH=/home/olaya/catkin_ws/src/tritech_profiler/msg:$PYTHONPATH

PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages:$PYTHONPATH 

PYTHONPATH=/home/olaya/catkin_ws/devel/lib/python2.7/dist-packages:$PYTHONPATH 

export PYTHONPATH
echo $PYTHONPATH

pytest test.py
