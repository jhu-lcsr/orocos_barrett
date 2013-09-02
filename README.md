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
