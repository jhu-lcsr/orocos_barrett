RTT Barret Interface
====================

This repository contains Orocos/RTT components for interfacing with Barret WAM
and BHand hardware supported by libbarrett.

This package aims to support developing real-time state estimation and control
algorithms in the Orocos Toolchain for the Barrett WAM robot.

## libbarrett-Orocos Interface

The Orocos/RTT interfaces are meant to provide interfaces similar to
libbarrett's "low-level" interface. These interfaces include
`barrett::LowLevelWam` and `barrett:Hand`, which each provide direct access to
joint-level torque, position, and velocity information.

The package [**rtt\_barrett\_interface**](rtt_barrett_interface) includes an
Orocos component which is represents the set of devices that can be represented
by a single libbarrett `barrett::ProductManager` and is used in
[**rtt\_barrett\_hw**](rtt_barrett_hw) and
[**rtt\_barrett\_gazebo**](rtt_barrett_gazebo) for talking to the real hardware
and simulated hardware, respectively. See each package for more information
about its contents.

## Building

Building the rtt\_barrett packages from source is most easily done with a pair
of Catkin workspaces. One workspace is an "isolated" workspace, and the other is
a "normal" workspace.

First, clear your catkin environment:
```bash
unset CMAKE_PREFIX_PATH
source /opt/ros/$ROS_DISTRO/setup.sh
```

Then, checkout the Eigen-3-based version of catkin to an "isolated" workspace
and build it:
```bash
mkdir -p ~/ws/underlay_isolated/src
cd ~/ws/underlay_isolated
git clone git@github.com:jhu-lcsr-forks/barrett.git src/barrett
catkin_make_isolated --install
source install_isolated/setup.bash
```

Then in the same shell, create a "normal" workspace for these packages and yours:
```bash
mkdir -p ~/ws/underlay
git clone git@github.com:jhu-lcsr/rtt_barrett.git src/rtt_barrett
catkin_make
source devel/setup.sh
```

Now you can move on to trying the examples.

## Examples


