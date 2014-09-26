
Orocos Barrett Hardware Interface
=================================

## Examples

For a 7-DOF WAM on bus 0 with a BHand 

```sh
roslaunch test/dual_barrett.launch BUS:=0 7DOF:=true HAND:=true
```

For a 4-DOF WAM on bus 0 with a BHand and a 7-DOF WAM on bus 1:

```sh
roslaunch test/dual_barrett.launch LEFT_4DOF:=true LEFT_HAND:=true RIGHT_7DOF:=true
```
