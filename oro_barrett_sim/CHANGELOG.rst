^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package oro_barrett_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2014-12-09)
------------------

0.1.0 (2014-12-08)
------------------
* sim: fixing bad initialization of max torques
* hw: Numerous updatest to hand control
* improved torqueswitch behavior and improved high-level bhand actions
* adding spread action nd updating bhand action defs
* breakaway logic comment
* hand: adding release action, improving robustness of grasp action, fixing trapezoidal mode in simulation
* merge
* hand: adding high-level grasp command and cmd status reporting
* adding a bunch of debug info for hand controller
* updating ops script test
* updating sim test parameters and adding extra urdf debug info
* adding software estop trigger
* removing plugin_depends which are unnecessary because typekits are loaded automatically, this was preventing this plugin from being loaded
* updating gui connections to home/idle/run the arm
* fixed typo and safety limit warning
* fixed orocosScript tag
* sim: zeroing force if the command stops
* oro_barrett_sim: Guaranteed initialization in case this gets called before the gazebo update.
* rtt_rosclock api changes
* Adding rosparam auto config
* Adding auto-configure capability
* adjusting breakaway torque simulation
* removing obvious comments
* simplifying oro_barrett interfaces
* Adding documentation
* Adding example with two WAMs in simulation
* Reducing repeated code
* slight reorg
* cleaning up test names
* Updating launchfiles and adding wam-hand xacro
* sim: bhand with torque-switch behavior
* oro_barrett_sim: Adding hand simulation
* decreasing step now that damping is working
* Adding barrett hand sim and refactoring sim/hw interface
* updating deps
* Updating device functions to accept ros::Time structures
* Disabling torque limit error mode in simulation
* Updating simulation dialtone
* Updating cmakelists, idle test
* Adding orocos barrett simulation for use with rtt_gazebo_deployer plugin
* Contributors: Christopher Paxton, Jonathan Bohren, Zihan Chen
