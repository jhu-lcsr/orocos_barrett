Orocos Barret Interface
=======================

This repository contains Orocos/RTT components for interfacing with Barret WAM
and BHand hardware supported by libbarrett.

This package aims to support developing real-time state estimation and control
algorithms in the Orocos Toolchain for the Barrett WAM robot.

## libbarrett-Orocos Interface

The Orocos/RTT interfaces are meant to provide interfaces similar to
libbarrett's "low-level" interface. These interfaces include
`barrett::LowLevelWam` and `barrett:Hand`, which each provide direct access to
joint-level torque, position, and velocity information.

The package [**oro\_barrett\_interface**](oro_barrett_interface) includes an
Orocos component which is represents the set of devices that can be represented
by a single libbarrett `barrett::ProductManager` and is used in
[**oro\_barrett\_hw**](oro_barrett_hw) and
[**oro\_barrett\_gazebo**](oro_barrett_gazebo) for talking to the real hardware
and simulated hardware, respectively. See each package for more information
about its contents.

These packages require Orocos and rtt_ros_integration version 2.7 or greater.

## ROS Interface

These components also expose a ROS interface via the `rtt_ros_integration`
packages.

## Theory of Operation

Since this set of Orocos components uses `libbarrett`, it's designed with the
same concepts in mind. For example, the `oro_barrett_hw::BarrettHWManager`
corresponds to a `libbarrett::ProductManager` and its associated CANBus. As
such, this manager is used to configure and instantiate different products on
that bus, like a WAM or BHand. These products then appear as RTT services of
the manager.

### Soft Running and Idling

Both the hand and the arm must be initialized if they are being brought up for
the first time (see below for more detail). Before they are initialized, they
will still query the hardware for device state. Once they have been
initialized, they can be `run`. Once the system is activated via the control
pendant, they will send commands to their respective devices. 

At any point, the WAM can be *soft-idled* which will cause it to stop sending
effort commands. This is not the ideal way to start and stop the robot,
however, because it will also stop compensating for gravity. It is better to
*hard-idle* the robot via the control pendant.

Once the robot has been *hard-idled*, it will automatically *soft-idle* and
before it will begin sending commands again, it must be *soft-run*.

### Safety Features

The Orocos Barrett WAM abstraction includes both velocity and effort limits per
joint. If at any point any one joint velocity exceeds its velocity limit or the
commanded effort exceeds its effort limit, the Barrett manager component will
zero the effort command, stop running, and enter the RTT Error state. This will
terminate the sending of joint commands and trigger an E-STOP on the Barrett
Safety System. Once this happens, the robot must be re-set.

In addition to these limits, there is a scalar warning ratio such that any
velocities or efforts which exceed that ratio of the limit will prompt a
warning as often as once per second.

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
git clone git@github.com:jhu-lcsr/orocos_barrett.git src/orocos_barrett
catkin_make
source devel/setup.sh
```

Now you can move on to trying the examples.


## Examples with Gazebo

```
roslaunch oro_barrett_sim wam.launch
```

```
roslaunch oro_barrett_sim hand.launch
```

```
roslaunch oro_barrett_sim wam_hand.launch
```

## Examples with Real Hardware

See each package for usage examples for both simulated and real robots.

### Bringing up a Real WAM

First, load the following parameters onto the ROS parameter server. These include the CANBus ID, the home position, and the resolver (MECH encoder) offsets in radians at the home position. In addition, make sure the `/robot_description` ROS parameter is set with the URDF from the [barrett_model](http://github.com/jhu-lcsr/barrett_model) package.

```yml
deployer:
  barrett_hw_manager:
    bus_id: 0
    wam:
      home_position: [0.0, -1.5708, 0.0, 3.1415, 0.0, -1.5708, 1.5708]
      home_resolver_offset: [0.544563, -2.09235, 0.944932, -1.35757, 2.11383, 1.18423, 2.23808]
```

Second, import the Orocos Barrett components and create and configure a Barrett Hardware Manager and a WAM like the following orocos script:

```cpp
import("rtt_ros");
ros.import("oro_barrett_hw");

/* Create the barrett manager */
loadComponent("barrett_hw_manager","oro_barrett_hw::BarrettHWManager");
loadService("barrett_hw_manager","rosparam");

/* Load parameters from ROS */
barrett_hw_manager.rosparam.getAll();
barrett_hw_manager.rosparam.getAbsolute("robot_description");

/* Configure a 7-DOF WAM */
barrett_hw_manager.configure();
barrett_hw_manager.configureWam7("wam");
barrett_hw_manager.rosparam.getComponentPrivate("wam");
```

Second, the WAM might need to be homed. To do this, move the WAM _near_ the known calibration pose, and run the following Orocos script:

```cpp
barrett_hw_manager.wam.initialize();
barrett_hw_manager.wam.run()
```

After homing the WAM, you can enable your controllers and activate the WAM.

If the calibration offsets are unknown, you can inspect the current resolver offsets (as `resolver_offset_out`) by listing the WAM's properties in the deployer:

```cpp
ls barrett_hw_manager.wam
```

If the wam has already been homed since it was shut down, you don't need to home it, but you can turn off the reading of the resolver angles to save CANBus bandwidth:

```cpp
barrett_hw_manager.wam.run()
```

### Bringing up a Real BHand

You can also add a Barrett BHand 280 to the Hardware Manager. Similarly to above, construct the BHand like the following Orocos script:

```cpp
barrett_hw_manager.configureHand("wam/bhand");
```

Before it can be used, the hand also needs to be homed:

```cpp
barrett_hw_manager.hand.initialize();
```

This will open and close the hand, and afterwards the hand will output state and accept commands. 

If the BHand has already been homed since it was turned on, you only need to run the following to start it reporting position and accepting commands:

```cpp
barrett_hw_manager.hand.run();
```
