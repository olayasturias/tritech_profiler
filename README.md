# Tritech Profiler ROS Package

This ROS package configures and communicates with the Tritech Profiling sonar.
**This has only been tested on ROS Indigo over RS232.**

## Setting up

You must clone this repository as `tritech_profiler` into your catkin workspace:

```bash
git clone https://github.com/olayasturias/tritech_profiler
```

## Dependencies

Before proceeding, make sure to install all the dependencies by running:

```bash
rosdep update
rosdep install tritech_profiler
```

## Compiling

You **must** compile this package before being able to run it. You can do so
by running:

```bash
catkin_make
```

from the root of your workspace.

## Running

To run, simply connect the Tritech Profiler sonar over RS232 and launch the
package with:

```bash
roslaunch tritech_profiler tritech_profiler.launch port:=</path/to/sonar> frame:=<frame_id>
```

`port` and `frame` are run-time ROS launch arguments:

-   `port`: Serial port to read from, default: `/dev/sonar`.
-   `frame`: `tf` frame to stamp the messages with, default: `sonar`.

The package will keep trying to connect to the sonar until it is successful.

The `tritech_micron` node will output to the following ROS topics:

-   `~scan`: `PointCloud` message. Scan data of the current heading only.
-   `~heading`: `PoseStamped` message. Current heading of the sonar.
-   `~config`: `TritechMicronConfig` message. Sonar config published on change.

## Configuring

To configure the Tritech Micron sonar, take a look at the parameters defined
in [Scan.cfg](cfg/Scan.cfg).

These paramaters can also be updated on the fly with ROS `dynamic_reconfigure`
as such:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```


