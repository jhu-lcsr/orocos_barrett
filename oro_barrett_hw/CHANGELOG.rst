^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package oro_barrett_hw
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2014-12-09)
------------------

0.1.0 (2014-12-08)
------------------
* hw: Numerous updatest to hand control
* more reactive zerocal script
* hw: fixing eigen map problem in bhand and adding estop case to wam
* improving zerocal usability
* Update README.md
* hw: adding zerocal script and zerocal documentation
* hand: adding high-level grasp command and cmd status reporting
* hw: always update velocity command modes
* hw: adding readme
* hw: Switching cpu affinity assignment and fixing new hand temp checking rate
* hw: fixing cpu affinity setting, adding dual arm launchfile test
* hw: simplifying launchfiles
* hand: decreasing rate at which the temperature is polled
* updating basic tests
* bhand: adding fingertip torque publishing and fixing velocity mode commanding
* adding software estop trigger
* zeroing  output on error if running
* adding estop runmode
* changing this back to 1khz
* hw: asynchronous calibration/mode changing
* adding homing introspection
* Merge branch 'master' of github.com:jhu-lcsr/orocos_barrett
* adding homing introspection
* hw: fix idle on cleanup, see fixme
* rqt_barrett: Fixing connection between gui and hw interfaces
* Merging multithreaded branch into master with introspection changes from gui features
* updating gui connections to home/idle/run the arm
* fixing hand interface
* more explicit synchronization, should cause whatever's scheduling the
  component to sync up with the device thread
  notes:
  - running faster than 0.0005 s period starves the whole system
  - all the barret calls need to happen from the same thread?
* running, but threads are behaving badly
* starting mt version
* adding torque scales to adjust motor constants
* Updated wam experimentally verified
  - updated for new interface changes to match simulation
  - updated test launchfile
  - now using butterworth velocity smoothing (butterworth code care of the
  open wam driver project over at cmu:
  https://personalrobotics.ri.cmu.edu/pr-ros-pkg/owd/html/)
* removing old test
* hw: updating build and test
* rtt_rosclock api changes
* Adding rosparam auto config
* Adding auto-configure capability
* oro_barrett_sim: Adding hand simulation
* Adding barrett hand sim and refactoring sim/hw interface
* Updating device functions to accept ros::Time structures
* updating test
* Updating cmakelists, idle test
* Adding orocos barrett simulation for use with rtt_gazebo_deployer plugin
* removing old dep
* commenting out barrett dep in package.xml
* updating package.xml dep
* Small tweaks
* Splitting oro_barrett_msgs
* Going back to standard kdl usage
* Cleaning up run/idle behavior, simp[lifying velocity smoothing
* More safety features
* Updating safety features
* Numerous robusness updates, fixes, adding 4-DOF wam support
* Fixing tree inertia computation
* always write hand cog
* bhand: Updating frame that the cg is computed in. Commenting out additional debug statements so that cg computation can be run in realtime.
* bhand: Properly constructing KDL subtree
* Unifying wam/hand apis
* updating time api
* Making run do the run thing
* Adding more documentation
* Properly working hand behaviors
* Fixing torque command set
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
* fixing published time
* New rtt-ros stuf + Several improvements
* disabling this test
* adding idle test
* beginning to add bhand support
* Fixing bus configuration, adding a lot of error handling, adding test
* rtt -> oro
* Contributors: Jonathan Bohren
