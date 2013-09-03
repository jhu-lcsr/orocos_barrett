#ifndef __RTT_BARRETT_INTERFACE_WAM_DEVICE_H
#define __RTT_BARRETT_INTERFACE_WAM_DEVICE_H

#include <string>
#include <vector>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

namespace rtt_barrett_interface {

  /** \brief Base interface class for real and simulated 4- and 7-DOF WAMs.
   */
  class WamDeviceBase 
  {
  public:
    //! Zero all the joint-space values
    virtual void setZero() = 0;
    //! Read the configuration and publish it
    virtual void readConfig() = 0;
    //! Read the hardware state and publish it
    virtual void readHW(RTT::Seconds time, RTT::Seconds period) = 0;
    //! Write the command to the hardware
    virtual void writeHW(RTT::Seconds time, RTT::Seconds period) = 0;
    //! Write the calibration command 
    virtual void writeHWCalibration(RTT::Seconds time, RTT::Seconds period) = 0;
  };

  //! Class for real and simulated 4- or 7-DOF WAMs
  template<size_t DOF>
    class WamDevice : public WamDeviceBase 
  {
  public:
    /** \brief Add all input and output ports to the "wam" service of the given parent
     * service.
     */
    WamDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &tip_joint_name) :
      parent_service_(parent_service)
    {
      RTT::Service::shared_ptr wam_service = parent_service->provides("wam");

      wam_service->addPort("effort_in", joint_effort_in);
      wam_service->addPort("calibration_status_in", joint_calibration_status_in);
      wam_service->addPort("calibration_burn_offsets_in", joint_calibration_burn_offsets_in);

      wam_service->addPort("effort_out", joint_effort_out);
      wam_service->addPort("position_out", joint_position_out);
      wam_service->addPort("velocity_out", joint_velocity_out);
      wam_service->addPort("resolver_position_out", joint_resolver_position_out);

      wam_service->addPort("resolver_ranges_out", joint_resolver_ranges_out);
      wam_service->addPort("effort_limits_out", joint_effort_limits_out);
      wam_service->addPort("velocity_limits_out", joint_velocity_limits_out);
      wam_service->addPort("joint_names_out", joint_names_out);

      // Resize joint names
      joint_names.resize(DOF);

      // Get URDF links starting at product tip link
      boost::shared_ptr<const urdf::Joint> joint = urdf_model.getJoint(tip_joint_name);

      // Get joint information starting at the tip
      for(size_t i=DOF-1; i>=0; i--) 
      {
        // While the joint has been handled or the joint type isn't revolute
        while(std::find(joint_names.begin(), joint_names.end(), joint->name) != joint_names.end() 
            || joint->type != urdf::Joint::REVOLUTE)
        {
          // Get the next joint
          joint = urdf_model.getLink(joint->parent_link_name)->parent_joint;
          // Make sure we didn't run out of links
          if(!joint.get()) {
            std::ostringstream oss;
            oss << "Ran out of joints while parsing URDF starting at joint: \""
              << tip_joint_name << "\"";
            throw std::runtime_error(oss.str());
          }
        }

        // Store the joint properties
        joint_names[i] = joint->name;
        joint_effort_limits(i) = joint->limits->effort;
        joint_velocity_limits(i) = joint->limits->velocity;
      }
    }

    //! Removes the added "wam" service 
    virtual ~WamDevice() 
    {
      parent_service_->removeService("wam");
    }

    virtual void setZero() 
    {
      //joint_offsets.setZero();
      joint_position.setZero();
      joint_velocity.setZero();
      joint_effort.setZero();
      joint_resolver_position.setZero();
      joint_calibration_burn_offsets.setZero();
    }

    virtual void readConfig()
    {
      joint_resolver_ranges_out.write(joint_resolver_ranges); 
      joint_effort_limits_out.write(joint_effort_limits);
      joint_velocity_limits_out.write(joint_velocity_limits);
      joint_names_out.write(joint_names);
    }

  protected:
    //! RTT Service for WAM interfaces
    RTT::Service::shared_ptr parent_service_;

    // Configuration
    std::vector<std::string> 
      joint_names;
    Eigen::Matrix<double,DOF,1> 
      joint_resolver_ranges,
      joint_effort_limits,
      joint_velocity_limits;

    //! \name State
    //\{
    bool calibrated;
    Eigen::Matrix<double,DOF,1> 
      //joint_offsets,
      joint_position,
      joint_velocity,
      joint_effort,
      joint_resolver_position,
      joint_calibration_burn_offsets;
    Eigen::Matrix<int,DOF,1> 
      joint_calibration_status;
    //\}

    //! \name Input ports
    //\{
    RTT::InputPort<Eigen::Matrix<double,DOF,1> >
      joint_effort_in,
      joint_calibration_burn_offsets_in;
    RTT::InputPort<Eigen::Matrix<int,DOF,1> >
      joint_calibration_status_in;
    //\}
    
    //! \name State output ports
    //\{
    RTT::OutputPort<Eigen::Matrix<double,DOF,1> >
      joint_effort_out,
      joint_position_out,
      joint_velocity_out,
      joint_resolver_position_out;
    //\}

    //! Configuration output ports
    //\{
    RTT::OutputPort<Eigen::Matrix<double,DOF,1> >
      joint_resolver_ranges_out,
      joint_effort_limits_out,
      joint_velocity_limits_out;
    RTT::OutputPort<std::vector<std::string> >
      joint_names_out;
    //\}
  };
}

#endif // ifndef __RTT_BARRETT_INTERFACE_WAM_DEVICE_H
