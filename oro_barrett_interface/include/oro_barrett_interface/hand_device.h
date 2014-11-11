#ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
#define __ORO_BARRETT_INTERFACE_HAND_DEVICE_H

#include <boost/assign/std/vector.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <sensor_msgs/JointState.h>

#include <rtt_ros_tools/throttles.h>
#include <rtt_roscomm/rtt_rostopic.h>

#include <oro_barrett_msgs/BHandCmd.h>
#include <oro_barrett_msgs/BHandStatus.h>
#include <oro_barrett_msgs/BHandSetModeAction.h>
#include <oro_barrett_msgs/BHandInitAction.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

#include <geometry_msgs/PoseStamped.h>

#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>

namespace oro_barrett_interface {

  class HandDevice {
  public:
    //! Initialize the hand (get pose and hold open)
    virtual void initialize() = 0;
    //! Idle the hand (open it up and disable the motors)
    virtual void idle() = 0;
    //! Run the hand (process commands)
    virtual void run() = 0;
    //! Set finger compliance
    virtual void setCompliance(bool enable) = 0;

    //! Set joint to torque mode
    virtual void setTorqueMode(unsigned int joint_index) = 0;
    //! Set joint to position mode
    virtual void setPositionMode(unsigned int joint_index) = 0;
    //! Set joint to velocity mode
    virtual void setVelocityMode(unsigned int joint_index) = 0;
    //! Set joint to trapezoidal mode
    virtual void setTrapezoidalMode(unsigned int joint_index) = 0;

    //! Read the hardware state and publish it
    virtual void readDevice(ros::Time time, RTT::Seconds period) = 0;
    //! Write the command to the hardware
    virtual void writeDevice(ros::Time time, RTT::Seconds period) = 0;

    //! Read the simulation state
    virtual void readSim(ros::Time time, RTT::Seconds period) { }
    //! Write the simulation command
    virtual void writeSim(ros::Time time, RTT::Seconds period) { }

    //! Read the input ports
    //TODO: make pure virtual
    virtual void readPorts() {};
    //! Write the output ports
    //TODO: make pure virtual
    virtual void writePorts() {};

    HandDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix);

    //! Removes the added "wam" service
    virtual ~HandDevice();

    virtual void open() = 0;
    virtual void close() = 0;

    //! Number of pucks in the hand
    static const unsigned int N_PUCKS = 4;
    static const unsigned int DOF = 8;

    //! Hand run mode
    enum RunMode {
      IDLE = 0,
      INITIALIZE,
      RUN
    };

    //! Initialization state
    enum InitState {
      INIT_FINGERS = 0,
      SEEK_FINGERS,
      INIT_SPREAD,
      SEEK_SPREAD,
      INIT_CLOSE
    };

  protected:

    //! RTT Service for BHand interfaces
    RTT::Service::shared_ptr parent_service_;

    //! The current run mode
    RunMode run_mode;

    //! The current initialization state
    InitState init_state;

    //! \name Configuration & State
    //\{
    KDL::Tree full_tree;
    //! The names of all the joints
    std::vector<std::string>
      joint_names;
    // True corresponds to an active joint, False corresponds to a passive one
    std::vector<bool>
      joint_actuation;

    std::string urdf_prefix;
    KDL::Tree kdl_tree;
    KDL::Frame base_to_parent_transform;

    // Joint state for active and passive joints
    Eigen::VectorXd
      joint_position,
      joint_velocity,
      joint_torque_cmd,
      joint_position_cmd,
      joint_velocity_cmd,
      joint_trapezoidal_cmd,
      center_of_mass;

    //! Mode bitmasks
    unsigned int
      mode_torque,
      mode_position,
      mode_velocity,
      mode_trapezoidal;

    //! Flag desingating any modes have changed
    bool modes_changed;

    std::map<std::string, double> q_map;
    geometry_msgs::PoseStamped com_msg;

    Eigen::VectorXi
      knuckle_torque;

    sensor_msgs::JointState
      joint_state;
    oro_barrett_msgs::BHandCmd
      joint_cmd;
    oro_barrett_msgs::BHandStatus
      status_msg;
    //\}

    //! \name Input ports
    //\{
    RTT::InputPort<Eigen::VectorXd >
      joint_torque_in,
      joint_position_in,
      joint_velocity_in,
      joint_trapezoidal_in;

    RTT::InputPort<oro_barrett_msgs::BHandCmd>
      joint_cmd_in;
    //\}

    //! \name Output ports
    //\{
    RTT::OutputPort<Eigen::VectorXd >
      joint_torque_out,
      joint_position_out,
      joint_velocity_out,
      center_of_mass_out;
    RTT::OutputPort<sensor_msgs::JointState >
      joint_state_out;
    RTT::OutputPort<geometry_msgs::PoseStamped>
      center_of_mass_debug_out;
    RTT::OutputPort<oro_barrett_msgs::BHandStatus>
      status_out;
    //\}

    rtt_ros_tools::PeriodicThrottle joint_state_throttle;

    //! Compute the hand center of mass
    void computeCenterOfMass(Eigen::VectorXd &xyzm);
    void computeCenterOfMassDebug();
    void computeCenterOfMass(Eigen::VectorXd &xyzm, bool debug);

    //! Action servers
    rtt_actionlib::RTTActionServer<oro_barrett_msgs::BHandInitAction> initialize_action_server_;
    rtt_actionlib::RTTActionServer<oro_barrett_msgs::BHandSetModeAction> set_mode_action_server_;

    void initialize_goal_cb(actionlib::ServerGoalHandle<oro_barrett_msgs::BHandInitAction> gh);
    void set_mode_goal_cb(actionlib::ServerGoalHandle<oro_barrett_msgs::BHandSetModeAction> gh);
  };

}

#endif // ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
