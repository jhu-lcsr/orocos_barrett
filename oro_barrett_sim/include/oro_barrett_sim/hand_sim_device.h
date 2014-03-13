#ifndef __ORO_BARRETT_SIM_HAND_SIM_DEVICE_H
#define __ORO_BARRETT_SIM_HAND_SIM_DEVICE_H

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <kdl/velocityprofile_trap.hpp>

#include <oro_barrett_interface/hand_device.h>

namespace oro_barrett_sim {

  /** \brief Orocos/ROS interface for a simulated Barrett Hand
   *
   */
  class HandSimDevice : public oro_barrett_interface::HandDevice
  {
  public:
    virtual void initialize();
    virtual void idle();
    virtual void run();

    virtual void setCompliance(bool enable);

    virtual void setTorqueMode(unsigned int joint_index);
    virtual void setPositionMode(unsigned int joint_index);
    virtual void setVelocityMode(unsigned int joint_index);
    virtual void setTrapezoidalMode(unsigned int joint_index);

    virtual void readSim(ros::Time time, RTT::Seconds period);
    virtual void writeSim(ros::Time time, RTT::Seconds period);

    virtual void readDevice(ros::Time time, RTT::Seconds period);
    virtual void writeDevice(ros::Time time, RTT::Seconds period);

    virtual void open();
    virtual void close();

    HandSimDevice(
        RTT::Service::shared_ptr parent_service, 
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix,
        std::vector<gazebo::physics::JointPtr> joints);
     
  protected:

    bool doneMoving(const unsigned pair_index);
    bool withinTorqueLimits(const unsigned joint_index);

    std::vector<gazebo::physics::JointPtr> gazebo_joints;

    bool compliance_enabled;

    double 
      breakaway_torque, 
      stop_torque;

    Eigen::VectorXd
      link_torque,
      fingertip_torque,
      breakaway_angle,
      joint_torque,
      joint_torque_max,
      joint_torque_breakaway;

    std::vector<KDL::VelocityProfile_Trap> trap_generators;
    std::vector<ros::Time> trap_start_times;
    std::vector<bool> torque_switches;

    double 
      p_gain,
      d_gain,
      velocity_gain;
  };
}

#include "hand_sim_device_impl.h"

#endif // ifnedf __ORO_BARRETT_SIM_HAND_DEVICE_H
