^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package oro_barrett_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* moved catkin_modules dependency to catkin COMPONENTS list
* Attempting indigo build.  Fixed eigen dependency by adding cmake_modules to the dependencies. Build fails at oro_barrett_sim due to a separate Eigen error.
* rqt: add high-level hand commands to rqt dashboard
* hw: Numerous updatest to hand control
* adding new hand actions
* adding new hand actions
* updating bhand status and bhand message imports in scripts
* hw: adding zerocal script and zerocal documentation
* improved torqueswitch behavior and improved high-level bhand actions
* cleaning up grasp metrics so they can be used in other hand actions
* adding high-level grasp metrics to grasp action: cage radii and fingertip radii
* bhand grasp: adding proper radius computation
* adding options to grasp/release
* adding spread action nd updating bhand action defs
* hand: adding release action, improving robustness of grasp action, fixing trapezoidal mode in simulation
* hand: adding high-level grasp command and cmd status reporting
* updating sim test parameters and adding extra urdf debug info
* adding software estop trigger
* hw: asynchronous calibration/mode changing
* rqt_barrett: Fixing connection between gui and hw interfaces
* Merging multithreaded branch into master with introspection changes from gui features
* updating gui connections to home/idle/run the arm
* running, but threads are behaving badly
* adding torque scales to adjust motor constants
* hw: updating build and test
* oro_barrett_sim: Guaranteed initialization in case this gets called before the gazebo update.
* Adding auto-configure capability
* simplifying oro_barrett interfaces
* oro_barrett_sim: Adding hand simulation
* Adding barrett hand sim and refactoring sim/hw interface
* Updating device functions to accept ros::Time structures
* Updating cmakelists, idle test
* Adding orocos barrett simulation for use with rtt_gazebo_deployer plugin
* Small tweaks
* Splitting oro_barrett_msgs
* Going back to standard kdl usage
* adding doc
* Cleaning up run/idle behavior, simp[lifying velocity smoothing
* More safety features
* Updating safety features
* Numerous robusness updates, fixes, adding 4-DOF wam support
* Fixing tree inertia computation
* bhand: Updating frame that the cg is computed in. Commenting out additional debug statements so that cg computation can be run in realtime.
* bhand: Properly computing COM and Properly constructing KDL subtree (for real)
* bhand: Properly constructing KDL subtree
* Unifying wam/hand apis
* Properly working hand behaviors
* Numerous updates to bhand
* Adding hand messages and better hand support
* Lots of updates, stability improvements, dealing with corner cases
* Protecting against bad inputs
* Numerous additions for zerocal and bhand support
  Adding angles package as a dependency, using it for reading resolver angles
  Adding calibration support
  Adding support for hand hardware
  Adding open and close operations for bhand
  Adding hand joint state publishing to ros
  Adding joint state publisher for whole arm
* New rtt-ros stuf + Several improvements
* adding idle test
* beginning to add bhand support
* Fixing bus configuration, adding a lot of error handling, adding test
* rtt -> oro
* Contributors: John Choi, Jonathan Bohren
