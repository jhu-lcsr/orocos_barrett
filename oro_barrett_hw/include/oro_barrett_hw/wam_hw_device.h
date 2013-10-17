#ifndef __ORO_BARRETT_HW_WAM_HW_DEVICE
#define __ORO_BARRETT_HW_WAM_HW_DEVICE

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <barrett/systems.h>

#include <oro_barrett_interface/wam_device.h>

#include <control_toolbox/filters.h>

#include <sensor_msgs/JointState.h>

namespace oro_barrett_hw {

  /** \brief State structure for a real WAM device
   *
   */
  template<size_t DOF>
    class WamHWDevice : public oro_barrett_interface::WamDevice<DOF>
  {
  public:
    /** \brief Construct a low-level WAM interface and extract joint information from
     * the URDF.
     */
    WamHWDevice(
        RTT::Service::shared_ptr parent_service, 
        const urdf::Model &urdf_model,
        const std::string &tip_joint_name,
        boost::shared_ptr<barrett::ProductManager> barrett_manager,
        const libconfig::Setting &wam_config) :
      oro_barrett_interface::WamDevice<DOF>(
          parent_service, 
          urdf_model, 
          tip_joint_name)
    {
      // Wait for the wam
      barrett_manager->waitForWam(false);

      // Get the wam pucks
      wam_pucks = barrett_manager->getWamPucks();
      wam_pucks.resize(DOF);

      // Construct a low-level wam
      interface.reset(
          new barrett::LowLevelWam<DOF>(
            wam_pucks, 
            barrett_manager->getSafetyModule(), 
            wam_config["low_level"]));

      // Initialize resolver ranges
      this->computeResolverRanges();
    }

    virtual void computeResolverRanges() 
    {
      Eigen::MatrixXd m_to_j_pos = interface->getMotorToJointPositionTransform();  
      this->joint_resolver_ranges = (m_to_j_pos.diagonal().array() * 2.0*M_PI).cwiseAbs().matrix();
    }

    virtual void calibrateNearHome()
    {
      const Eigen::MatrixXd & mpos2jpos = interface->getMotorToJointPositionTransform();

      // Compute actual position
      const Eigen::VectorXd actual_position =
        this->joint_home_position 
        - this->joint_home_resolver_position
        + mpos2jpos*this->joint_resolver_position;

      // Set the actual position
      interface->definePosition(actual_position);
    }

    virtual void readHW(RTT::Seconds time, RTT::Seconds period)
    {
      // Poll the hardware
      try {
        interface->update();
      } catch (const std::runtime_error& e) {
        if (interface->getSafetyModule() != NULL  &&
            interface->getSafetyModule()->getMode(true) == barrett::SafetyModule::ESTOP) 
        {
          RTT::log(RTT::Error) <<
            "systems::LowLevelWamWrapper::Source::operate(): E-stop! Cannot "
            "communicate with Pucks." << RTT::endlog();
          return;
        } else {
          throw;
        }
      }

      // Get raw state
      Eigen::Matrix<double,DOF,1> raw_joint_position = interface->getJointPositions();
      Eigen::Matrix<double,DOF,1> raw_joint_velocity = interface->getJointVelocities();

      // Smooth velocity 
      // TODO: parameterize time constant
      for(size_t i=0; i<DOF; i++) {
        this->joint_velocity(i) = filters::exponentialSmoothing(
            raw_joint_velocity(i),
            this->joint_velocity(i),
            0.5);
      }

      // Store position
      // TODO: add internal calibration offsets
      this->joint_position = raw_joint_position;

      // Write to data ports
      this->joint_position_out.write(this->joint_position);
      this->joint_velocity_out.write(this->joint_velocity);
      this->joint_resolver_position_out.write(this->joint_resolver_position);

      // Publish state to ROS 
      if(this->joint_state_throttle.ready(0.02)) {
        // Read resolver angles
        std::vector<barrett::Puck*> pucks = interface->getPucks();	
        for(size_t i=0; i<pucks.size(); i++) {
          this->joint_resolver_position(i) = interface->getMotorPucks()[i].counts2rad(pucks[i]->getProperty(barrett::Puck::MECH));
        }

        // Update the joint state message
        this->joint_state.header.stamp = ros::Time(time);
        this->joint_state.name = this->joint_names;
        Eigen::Map<Eigen::VectorXd>(this->joint_state.position.data(),DOF) = this->joint_position;
        Eigen::Map<Eigen::VectorXd>(this->joint_state.velocity.data(),DOF) = this->joint_velocity;
        Eigen::Map<Eigen::VectorXd>(this->joint_state.effort.data(),DOF) = this->joint_effort;
          
        // Publish
        this->joint_state_out.write(this->joint_state);
      }
    }

    virtual void writeHW(RTT::Seconds time, RTT::Seconds period)
    {
      static int warning_throttle = 0;

      // Check if the effort command port is connected
      if(this->joint_effort_in.connected()) {
        // Read newest command from data ports 
        if(this->joint_effort_in.readNewest(this->joint_effort) != RTT::NewData) {
          return;
        }
      } else {
        // Not connected, zero the command
        this->joint_effort.setZero();
      }

      // Make sure the device is active
      if(interface->getSafetyModule()->getMode(true) != barrett::SafetyModule::ACTIVE) { 
        // If it's not active, set the effort command to zero
        // TODO: When it changes, prevent command jumps
        this->joint_effort.setZero();
      }

      for(size_t i=0; i<DOF; i++) {
        // Check if the joint effort st
        if(std::abs(this->joint_effort(i)) > this->joint_effort_limits[i]) {
          if(warning_throttle++ % 1000) {
            RTT::log(RTT::Warning) << "Commanded torque (" << this->joint_effort(i)
              << ") of joint (" << i << ") exceeded safety limits! They have "
              "been truncated to: +/- " << this->joint_effort_limits[i] << RTT::endlog();
          }
          // Truncate this joint torque
          this->joint_effort(i) = std::max(
              std::min(this->joint_effort(i), this->joint_effort_limits[i]),
              -1.0*this->joint_effort_limits[i]);
        }
      }

      // Set the torques
      interface->setTorques(this->joint_effort);
    }

  protected:
    //! libbarrett Interface
    boost::shared_ptr<barrett::LowLevelWam<DOF> > interface;
    std::vector<barrett::Puck*> wam_pucks;
  };

}

#endif // ifndef __ORO_BARRETT_HW_WAM_DEVICE
