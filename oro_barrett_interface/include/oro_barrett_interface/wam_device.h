#ifndef __ORO_BARRETT_INTERFACE_WAM_DEVICE_H
#define __ORO_BARRETT_INTERFACE_WAM_DEVICE_H

#include <string>
#include <vector>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_ros_tools/throttles.h>

#include <sensor_msgs/JointState.h>

#include <boost/scoped_ptr.hpp>

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
    //! Write the calibration command when the wam is "near" the home position
    virtual void initialize() = 0;
    //! Switch out of calibration mode
    virtual void run() = 0;
    //! Switch disable commands to the wam
    virtual void idle() = 0;
    //! The safety mode (IDLE/ACTIVE/ESTOP)
    virtual unsigned int getSafetyMode() = 0;
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

      warning_fault_ratio(0.66),
      read_resolver(false),

      // Data members
      joint_home_position(DOF),
      joint_home_resolver_offset(DOF),
      joint_resolver_ranges(DOF),
      joint_effort_limits(DOF),
      joint_velocity_limits(DOF),

      kdl_chain(),
      chain_dynamics(),
      
      joint_position(DOF),
      joint_velocity(DOF),
      joint_acceleration(DOF),
      joint_effort(DOF),
      joint_effort_raw(DOF),
      joint_effort_last(DOF),
      joint_resolver_offset(DOF),
      joint_calibration_burn_offsets(DOF),

      // Throttles
      joint_state_throttle(0.01),

      // Counters
      warning_count(DOF),

      // Modes
      safety_mode(0),
      last_safety_mode(0)
    {
      RTT::Service::shared_ptr wam_service = parent_service->provides("wam");
      wam_service->doc("Barrett WAM robot interface");

      // Properties
      wam_service->addProperty("velocity_smoothing_factor",velocity_smoothing_factor)
        .doc("The coefficient used for exponentially smoothing the velocity estimate.");
      wam_service->addProperty("warning_fault_ratio",warning_fault_ratio)
        .doc("The ratio of any given safety limit which should precipitate a warning.");
      wam_service->addProperty("home_position",joint_home_position)
        .doc("The joint-space calibration position of the robot.");
      wam_service->addProperty("home_resolver_offset",joint_home_resolver_offset)
        .doc("The motor-space encoder offsets when it is precisely at the home position.");
      wam_service->addProperty("read_resolver",read_resolver)
        .doc("Read the absolute value of the encoders (this slows down execution time).");

      // Attributes
      wam_service->addAttribute("effort_raw",joint_effort_raw);
      wam_service->addAttribute("effort",joint_effort);
      wam_service->addAttribute("warning_count",warning_count);

      // Data ports
      wam_service->addPort("effort_in", joint_effort_in);
      wam_service->addPort("calibration_burn_offsets_in", joint_calibration_burn_offsets_in);

      wam_service->addPort("effort_out", joint_effort_out);
      wam_service->addPort("position_out", joint_position_out);
      wam_service->addPort("velocity_out", joint_velocity_out);
      wam_service->addPort("resolver_offset_out", joint_resolver_offset_out);

      wam_service->addPort("resolver_ranges_out", joint_resolver_ranges_out);
      wam_service->addPort("effort_limits_out", joint_effort_limits_out);
      wam_service->addPort("velocity_limits_out", joint_velocity_limits_out);
      wam_service->addPort("joint_names_out", joint_names_out);

      // ROS data ports
      wam_service->addPort("joint_state_out", joint_state_out);
      wam_service->addPort("joint_resolver_state_out", joint_resolver_state_out);

      // Operations
      wam_service->addOperation("initialize", &WamDevice::initialize, this, RTT::OwnThread)
        .doc("Declare the actual position of the robot to be near the home position, so that it can home to actual zero");
      wam_service->addOperation("run", &WamDevice::run, this, RTT::OwnThread)
        .doc("Disable reading of additional data needed for calibration.");
      wam_service->addOperation("idle", &WamDevice::idle, this, RTT::OwnThread)
        .doc("Disable writing of commands and start reading additional data needed for calibration.");

      // Resize joint names
      joint_names.resize(DOF);

      // Resize joint state
      joint_state.name.resize(DOF);
      joint_state.position.resize(DOF);
      joint_state.velocity.resize(DOF);
      joint_state.effort.resize(DOF);

      joint_resolver_state.name.resize(DOF);
      joint_resolver_state.position.resize(DOF);

      // Get URDF links starting at product tip link
      const std::string tip_joint_name = (DOF == 7) ? (urdf_prefix+"/palm_yaw_joint") : (urdf_prefix+"/elbow_pitch_joint"); 
      boost::shared_ptr<const urdf::Joint> joint = urdf_model.getJoint(tip_joint_name);

      // Make sure we get the tip joint
      if(!joint) {
        throw std::runtime_error("Could not get tip joint for WAM!");
      }

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

      // Get a KDL tree from the urdf
      if(!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)){
        std::ostringstream oss;
        RTT::log(RTT::Error) << "Failed to construct KDL tree!" << RTT::endlog();
        throw std::runtime_error(oss.str());
      } 

      // Get a KDL chain for the arm
      if(!kdl_tree.getChain(
          kdl_tree.getRootSegment()->first, 
          urdf_model.getJoint(tip_joint_name)->child_link_name,
          kdl_chain)) {
        std::ostringstream oss;
        RTT::log(RTT::Error) << "Failed to extract KDL chain!" << RTT::endlog();
        throw std::runtime_error(oss.str());
      }

      // Construct a KDL chain dynamics solver
      chain_dynamics.reset(new KDL::ChainDynParam(kdl_chain, KDL::Vector(0,0,0)));

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
      joint_effort_last.setZero();
      joint_effort_raw.setZero();
      joint_resolver_offset.setZero();
      joint_calibration_burn_offsets.setZero();
    }

    virtual void readConfig()
    {
      joint_resolver_ranges_out.write(joint_resolver_ranges); 
      joint_effort_limits_out.write(joint_effort_limits);
      joint_velocity_limits_out.write(joint_velocity_limits);
      joint_names_out.write(joint_names);
    }

    virtual unsigned int getSafetyMode()
    {
      return safety_mode; 
    }

    //! Jointspace vector type for convenience
    typedef Eigen::VectorXd JointspaceVector;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> JointspaceFlagArray;

  protected:
    //! RTT Service for WAM interfaces
    RTT::Service::shared_ptr parent_service_;

    // Configuration
    double velocity_smoothing_factor;
    double warning_fault_ratio;
    bool read_resolver;
    std::vector<std::string> 
      joint_names;
    JointspaceVector 
      joint_home_position,
      joint_home_resolver_offset,
      joint_resolver_ranges,
      joint_effort_limits,
      joint_velocity_limits,
      joint_acceleration_limits;

    //! Joint Chain Dynamics Solver
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    boost::scoped_ptr<KDL::ChainDynParam> chain_dynamics;

    //! \name State
    //\{
    bool calibrated;
    JointspaceVector 
      //joint_offsets,
      joint_position,
      joint_velocity,
      joint_acceleration,
      joint_effort,
      joint_effort_raw,
      joint_effort_last,
      joint_resolver_offset,
      joint_calibration_burn_offsets;
    sensor_msgs::JointState
      joint_resolver_state,
      joint_state;

    JointspaceVector
      joint_effort_over_limits;
    JointspaceFlagArray
      joint_effort_limits_violated;

    //\}
    
    //! \name Input ports
    //\{
    RTT::InputPort<JointspaceVector >
      joint_effort_in,
      joint_calibration_burn_offsets_in;
    //\}
    
    //! \name State output ports
    //\{
    RTT::OutputPort<JointspaceVector >
      joint_effort_out,
      joint_position_out,
      joint_velocity_out,
      joint_resolver_offset_out;
    RTT::OutputPort<sensor_msgs::JointState >
      joint_resolver_state_out,
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

    std::vector<int> warning_count;
    unsigned int safety_mode;
    unsigned int last_safety_mode;
  };
}

#endif // ifndef __ORO_BARRETT_INTERFACE_WAM_DEVICE_H
