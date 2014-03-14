Orocos Barrett Simulation
=========================

This package contains Orocos RTT components for interfacing with Barrett
hardware in the Gazebo simulator.

These components present the same RTT data ports as the components in the
`oro_barrett_hw` package, but instead of connecting with real hardware, they
are loaded with the `rtt_gazebo_deployer` Gazebo plugin. See the readme at the
root of this repository for documentation on the component usage and
interfaces.

There are four examples in the `test` directory which bring up various
configurations of Barrett hardware in Gazebo with no controllers:

* `wam.launch` A single 7-DOF WAM robot
* `hand.launch` A single BH8-280 BHand
* `wam_hand.launch` A single 7-DOF WAM robot with an attached BH8-280 BHand
* `dual_wam_bhand.launch` A pair of 7-DOF WAM robots with BH8-280 BHands

