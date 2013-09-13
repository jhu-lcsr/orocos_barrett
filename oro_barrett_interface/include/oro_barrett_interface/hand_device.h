#ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
#define __ORO_BARRETT_INTERFACE_HAND_DEVICE_H

#include <boost/assign/std/vector.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

namespace oro_barrett_interface {

  class HandDevice {
  public:
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
    Eigen::Vector4d 
      joint_position,
      joint_position_cmd,
      joint_velocity_cmd,
      joint_effort_cmd;

    Eigen::Vector4i
      knuckle_torque;
    //\}

    //! \name Input ports
    //\{
    RTT::InputPort<Eigen::Matrix<double,8,1> >
      joint_effort_in,
      joint_position_in,
      joint_velocity_in;
    //\}
    
    //! \name Input ports
    //\{
    RTT::OutputPort<Eigen::Matrix<double,8,1> >
      joint_effort_out,
      joint_position_out,
      joint_velocity_out;
    //\}
  };

  HandDevice::HandDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix) :
    parent_service_(parent_service)
  {
    RTT::Service::shared_ptr wam_service = parent_service->provides("hand");

    using namespace boost::assign;
    joint_names.clear();
    joint_names += 
      urdf_prefix+"/finger_1/med_joint",
      urdf_prefix+"/finger_2/med_joint",
      urdf_prefix+"/finger_3/med_joint",
      urdf_prefix+"/finger_1/prox_joint",
      urdf_prefix+"/finger_1/dist_joint",
      urdf_prefix+"/finger_2/dist_joint",
      urdf_prefix+"/finger_3/dist_joint";

  }

  HandDevice::~HandDevice()
  {
    parent_service_->removeService("hand");
  }
}

#endif // ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
