#ifndef __ORO_BARRETT_INTERFACE_WAM_DEVICE_H
#define __ORO_BARRETT_INTERFACE_WAM_DEVICE_H

#include <string>
#include <vector>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <rtt_ros_tools/throttles.h>

#include <sensor_msgs/JointState.h>

namespace oro_barrett_interface {

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
    virtual void calibrateNearHome() = 0;
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
        const std::string &urdf_prefix) :
      parent_service_(parent_service),
      // Data members
      joint_home_position(DOF),
      joint_home_resolver_position(DOF),
      joint_resolver_ranges(DOF),
      joint_effort_limits(DOF),
      joint_velocity_limits(DOF),
      
      joint_position(DOF),
      joint_velocity(DOF),
      joint_effort(DOF),
      joint_resolver_position(DOF),
      joint_calibration_burn_offsets(DOF),

      // Throttles
      joint_state_throttle(0.01)
    {
      RTT::Service::shared_ptr wam_service = parent_service->provides("wam");
      wam_service->doc("Barrett WAM robot interface");

      // Properties
      wam_service->addProperty("home_position",joint_home_position);
      wam_service->addProperty("home_resolver_position",joint_home_resolver_position);
      wam_service->addProperty("effort",joint_effort);

      // Data ports
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

      // ROS data ports
      wam_service->addPort("joint_state_out", joint_state_out);

      // Operations
      wam_service->addOperation("calibrateNearHome", &WamDevice::calibrateNearHome, this)
        .doc("Declare the actual position of the robot to be near the home position, so that it can home to actual zero");

      // Resize joint names
      joint_names.resize(DOF);

      // Resize joint state
      joint_state.name.resize(DOF);
      joint_state.position.resize(DOF);
      joint_state.velocity.resize(DOF);
      joint_state.effort.resize(DOF);

      // Get URDF links starting at product tip link
      const std::string tip_joint_name = urdf_prefix+"/palm_yaw_joint"; 
      boost::shared_ptr<const urdf::Joint> joint = urdf_model.getJoint(tip_joint_name);

      // Get joint information starting at the tip (this way we're robust to
      // branching in the kinematic tree)
      for(size_t i=0; i<DOF; i++) 
      {
        unsigned jid = DOF-i-1;
        // While the joint has been handled or the joint type isn't revolute
        while(std::find(joint_names.begin(), joint_names.end(), joint->name) != joint_names.end() 
            || joint->type != urdf::Joint::REVOLUTE)
        {
          // Get the next joint
          joint = urdf_model.getLink(joint->parent_link_name)->parent_joint;
          // Make sure we didn't run out of links
          if(!joint) {
            std::ostringstream oss;
            RTT::log(RTT::Error) << "Ran out of joints while parsing URDF starting at joint: \""
              << tip_joint_name << "\"" << RTT::endlog();
            throw std::runtime_error(oss.str());
          }
        }

        RTT::log(RTT::Debug) << "Got joint "<<jid<<": \"" << joint->name << "\"" << RTT::endlog();

        // Store the joint properties
        joint_names[jid] = joint->name;
        joint_effort_limits(jid) = joint->limits->effort;
        joint_velocity_limits(jid) = joint->limits->velocity;
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

    //! Jointspace vector type for convenience
    typedef Eigen::VectorXd JointspaceVector;

  protected:
    //! RTT Service for WAM interfaces
    RTT::Service::shared_ptr parent_service_;

    // Configuration
    std::vector<std::string> 
      joint_names;
    JointspaceVector 
      joint_home_position,
      joint_home_resolver_position,
      joint_resolver_ranges,
      joint_effort_limits,
      joint_velocity_limits;

    //! \name State
    //\{
    bool calibrated;
    JointspaceVector 
      //joint_offsets,
      joint_position,
      joint_velocity,
      joint_effort,
      joint_resolver_position,
      joint_calibration_burn_offsets;
    Eigen::Matrix<int,DOF,1> 
      joint_calibration_status;
    sensor_msgs::JointState
      joint_state;

    //\}

    //! \name Input ports
    //\{
    RTT::InputPort<JointspaceVector >
      joint_effort_in,
      joint_calibration_burn_offsets_in;
    RTT::InputPort<Eigen::Matrix<int,DOF,1> >
      joint_calibration_status_in;
    //\}
    
    //! \name State output ports
    //\{
    RTT::OutputPort<JointspaceVector >
      joint_effort_out,
      joint_position_out,
      joint_velocity_out,
      joint_resolver_position_out;
    RTT::OutputPort<sensor_msgs::JointState >
      joint_state_out;
    //\}

    //! Configuration output ports
    //\{
    RTT::OutputPort<JointspaceVector >
      joint_resolver_ranges_out,
      joint_effort_limits_out,
      joint_velocity_limits_out;
    RTT::OutputPort<std::vector<std::string> >
      joint_names_out;
    //\}
    
    rtt_ros_tools::PeriodicThrottle joint_state_throttle;
  };
}

#endif // ifndef __ORO_BARRETT_INTERFACE_WAM_DEVICE_H
