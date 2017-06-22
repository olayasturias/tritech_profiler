**********************************
Dependencies
**********************************
The required tools for running or modifying this source code are the following:

+---------------------------+------------------------+
| | Device                  | | Visualization tools  |
+===========================+========================+
| | Ubuntu 14.04            | | Operating system     |
+---------------------------+------------------------+
| | ROS (Robot Operating    | | Meta-Operating       |
| | System), Indigo version | | System               |
+---------------------------+------------------------+
| | Python 2.7.1 or up      | | Programming lenguage |
+---------------------------+------------------------+

**********************************
Source code for Sensor Operation
**********************************

TritechProfiler class
##################################

The main activities regarding sensor operation take part in the methods
from the *TritechProfiler* class

.. autoclass:: sonar.TritechProfiler
   :members:

Resolution class
##################################

The default resolutions for the Profiling Sonar are as follows:

+------------+-------------+
| Resolution |  1/16 Grad  |
+============+=============+
|  LOWEST    |      255    |
+------------+-------------+
|  LOWER     |      128    |
+------------+-------------+
|  LOWERISH  |       64    |
+------------+-------------+
|  LOW       |       32    |
+------------+-------------+
|  MEDIUM    |       24    |
+------------+-------------+
|  HIGH      |       16    |
+------------+-------------+
|  ULTIMATE  |        8    |
+------------+-------------+

They are saved in the *Resolution* class:

.. autoclass:: sonar.Resolution
   :members:

Profiler Commands class
#################################

.. automodule:: commands
   :members:

Messages class with packet information
#######################################

.. autoclass:: messages.Message
   :members:

Replies class for replies handling
#######################################

.. autoclass:: replies.Reply
   :members:

Socket class for Serial Communication
#######################################

.. autoclass:: socket.Socket
   :members:

Tools
########################################

.. automodule:: tools
   :members:

**********************************
FAQs
**********************************

How to find the sonar port
##################################

When running *tritech_profiler* package you must specify the port. In order to determine the port where the sonar is connected, follow the next steps:

1. With the sensor disconnected do:
::
	ls/dev/ > ~/before.txt

This creates a file which enumerates all the connected devices.

2. With the sensor connected now, do the same but saving the data in a different file:
::
	ls/dev/ > ~/after.txt

3. Compare the differences between both files:
::
	sdiff ~/beforte.txt ~/after.txt

This marks the elements which has been added to the list when you connected the sensor. The ouputs you see should be something like this:

* serial
* ttyUSB0

4. Check which of both is your sensor by showing the raw data that it is sending:
::
	cat /dev/ttyUSB0


How to run tritech_profiler package by console
################################################
Once you have sourced your ROS environment with:
::
	source ./devel setup.bash

You can run the package with:
::
	roslaunch tritech_micron tritech_micron.launch port:=</path/to/sonar> frame:=<frame_id>

Where *port* is the serial port from where the sonar is transmitting, and *frame* is the tf to stamp the messages with.



