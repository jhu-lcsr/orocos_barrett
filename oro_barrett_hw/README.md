
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

## Zero-Calibrating the WAM Arm

Calibration of the WAM arm is done by storing the motor encoder angles at a
given joint-space configuration. This involves preciscely positioning the arm
manually while it is powered, but idle, and storing the encoder angles in a
YAML file. Once these encoder angles have been stored, they can be used to
determine the absolute position of each joint of the arm in a five-degree
window around the desired joint position.

### Using an existing calibration

The Orocos WAM component requires two properties to be set in order to set the
position of the arm and switch from `IDLE` to `RUN` mode:

* **home_position** -- an exact joint-space configuration 
* **home_resolver_offset** -- the motor-space encoder angles at the exact
  joint-space configuration

Given these two parameters, the arm can be precisely homed if it is manually
positioned within five-degrees of the `home_position` and then initiailzed.
Calling the `wam.initialize` service will use these parameters to determine the
actual position of the wam.

The initialization operation is what is called by the `Calibrate` button in the
GUI in the `rqt_barrett` package.

### Saving a new calibration

The WAM robots should not need to be recalibrated frequently. They should only
need to be recalibrated after re-tensioning, recabling, repairing, or if it
appears that the calibration has drifted for some other reason.

In practice, the reading and saving of the encoder angles and can be done
with the `zerocal.py` script.

1. Bring up the WAM robot you want to calibrate and make sure it is in `IDLE`
   mode.
2. Run the `zerocal.py` script in the namespace of the barrett manager:

    ```
    rosrun oro_barrett_hw zerocal.py path/to/config/for/wam_60.yaml __ns:=/barrett/barrett_manager
    ```

3. Manually position the arm to the exact home position using the appropriate
   jigs and levels.
4. Once the arm is exactly position, hit the `enter` key in the shell running
   `zerocal.py` and save the data to the appropriate yaml file.

Note that `zerocal.py` will overwrite the given file. While it will preserve
YAML comments, it will re-order any block of parameters alphabetically.

## Calibrating the WAM torque constants

Sometimes the WAM torque constants are incorrect. In order to compensate, the
Orocos WAM component uses the `torque_scales` property. This is an array of
scalar values which are used to multiply the output "raw" torque to the
low-level driver.

The best way to calibrate these is by running the arm with a gravity
compensation controller and making sure the arm can compensate for the weight
of the links in various configurations.

It's also been observed that these torque constants can change slightly when
using the arm for long periods of time. It may be dependent on the temperature
of the motors, but normally it makes the motors weaker.
