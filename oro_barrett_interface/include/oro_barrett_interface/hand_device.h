#ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
#define __ORO_BARRETT_INTERFACE_HAND_DEVICE_H

#include <boost/assign/std/vector.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <sensor_msgs/JointState.h>

#include <rtt_ros_tools/throttles.h>

#include <oro_barrett_msgs/BHandCmd.h>

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
    virtual void readHW(RTT::Seconds time, RTT::Seconds period) = 0;
    //! Write the command to the hardware
    virtual void writeHW(RTT::Seconds time, RTT::Seconds period) = 0;

    HandDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix);

    //! Removes the added "wam" service 
    virtual ~HandDevice();

    virtual void open() = 0;
    virtual void close() = 0;

  protected:
    //! RTT Service for BHand interfaces
    RTT::Service::shared_ptr parent_service_;

    //! \name Configuration & State
    //\{
    //! The names of all the joints
    std::vector<std::string> 
      joint_names;
    // True corresponds to an active joint, False corresponds to a passive one
    std::vector<bool>
      joint_actuation;

    // Joint state for active and passive joints
    Eigen::VectorXd
      joint_position,
      joint_velocity,
      joint_torque_cmd,
      joint_position_cmd,
      joint_velocity_cmd,
      joint_trapezoidal_cmd;

    Eigen::VectorXi
      knuckle_torque;

    sensor_msgs::JointState
      joint_state;
    oro_barrett_msgs::BHandCmd
      joint_cmd;
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
      joint_velocity_out;
    RTT::OutputPort<sensor_msgs::JointState >
      joint_state_out;
    //\}

    rtt_ros_tools::PeriodicThrottle joint_state_throttle;
  };

  HandDevice::HandDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix) :
    parent_service_(parent_service),

    joint_actuation(8),

    joint_position(8),
    joint_velocity(8),
    joint_torque_cmd(4),
    joint_position_cmd(4),
    joint_velocity_cmd(4),
    joint_trapezoidal_cmd(4),

    knuckle_torque(4),

    // Throttles
    joint_state_throttle(0.01)
  {
    RTT::Service::shared_ptr hand_service = parent_service->provides("hand");
    hand_service->doc("Barrett Hand interface.");

    // Operations
    hand_service->addOperation("initialize", &HandDevice::initialize, this, RTT::OwnThread);
    hand_service->addOperation("run", &HandDevice::run, this, RTT::OwnThread);
    hand_service->addOperation("idle", &HandDevice::idle, this, RTT::OwnThread);
    hand_service->addOperation("open", &HandDevice::open, this, RTT::OwnThread);
    hand_service->addOperation("close", &HandDevice::close, this, RTT::OwnThread);
    hand_service->addOperation("setCompliance", &HandDevice::setCompliance, this, RTT::OwnThread);

    hand_service->addOperation("setTorqueMode", &HandDevice::setTorqueMode, this, RTT::OwnThread);
    hand_service->addOperation("setPositionMode", &HandDevice::setPositionMode, this, RTT::OwnThread);
    hand_service->addOperation("setVelocityMode", &HandDevice::setVelocityMode, this, RTT::OwnThread);
    hand_service->addOperation("setTrapezoidalMode", &HandDevice::setTrapezoidalMode, this, RTT::OwnThread);

    hand_service->addConstant("F1",0);
    hand_service->addConstant("F2",1);
    hand_service->addConstant("F3",2);
    hand_service->addConstant("SPREAD",3);

    // ROS data ports
    hand_service->addPort("joint_cmd_in", joint_cmd_in);
    hand_service->addPort("joint_state_out", joint_state_out);

    using namespace boost::assign;
    joint_names.clear();
    joint_names += 
      urdf_prefix+"/finger_1/prox_joint",
      urdf_prefix+"/finger_2/prox_joint",
      urdf_prefix+"/finger_1/med_joint",
      urdf_prefix+"/finger_2/med_joint",
      urdf_prefix+"/finger_3/med_joint",
      urdf_prefix+"/finger_1/dist_joint",
      urdf_prefix+"/finger_2/dist_joint",
      urdf_prefix+"/finger_3/dist_joint";

    for(std::vector<std::string>::iterator joint_name_it = joint_names.begin();
        joint_name_it != joint_names.end();
        ++joint_name_it)
    {
      boost::shared_ptr<const urdf::Joint> joint = urdf_model.getJoint(*joint_name_it);

      if(!joint) {
        std::ostringstream oss;
        RTT::log(RTT::Error) << "Could not get needed BHand joint: \""
          << *joint_name_it << "\"" << RTT::endlog();
        throw std::runtime_error(oss.str());
      }
    }
    
    // Resize joint state
    joint_state.name.resize(8);
    joint_state.position.resize(8);
    joint_state.velocity.resize(8);
    joint_state.effort.resize(8);
  }

  HandDevice::~HandDevice()
  {
    parent_service_->removeService("hand");
  }
}

#endif // ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
