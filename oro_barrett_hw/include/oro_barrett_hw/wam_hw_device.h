#ifndef __ORO_BARRETT_HW_WAM_HW_DEVICE
#define __ORO_BARRETT_HW_WAM_HW_DEVICE

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <barrett/systems.h>

#include <oro_barrett_interface/wam_device.h>

#include <sensor_msgs/JointState.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <angles/angles.h>

#include <algorithm>

#include "butterworth.h"

namespace oro_barrett_hw {

  /** \brief State structure for a real WAM device
   *
   */
  template<size_t DOF>
    class WamHWDevice : public oro_barrett_interface::WamDevice<DOF>
  {
  public:

    enum RunMode {
      IDLE = 0,
      RUN
    };

    /** \brief Construct a low-level WAM interface and extract joint information from
     * the URDF.
     */
    WamHWDevice(
        RTT::Service::shared_ptr parent_service, 
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix,
        boost::shared_ptr<barrett::ProductManager> barrett_manager,
        const libconfig::Setting &wam_config) :
      oro_barrett_interface::WamDevice<DOF>(
          parent_service, 
          urdf_model, 
          urdf_prefix),
      barrett_manager_(barrett_manager),
      run_mode(IDLE),
      velocity_cutoff_(Eigen::VectorXd::Constant(DOF, 10.0)),
      torque_scales_(Eigen::VectorXd::Constant(DOF, 1.0))
    {
      parent_service->provides("wam")->addProperty("velocity_cutoff",velocity_cutoff_).doc("The velocity cutoff frequencies.");
      parent_service->provides("wam")->addProperty("torque_scales",torque_scales_).doc("The torque constant scaling factors.");

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

      // Enable resolver reading
      this->read_resolver = true;

      // Initialize resolver ranges
      this->computeResolverRanges();

      this->setFilters();
    }

    virtual ~WamHWDevice()
    {
    }

    virtual void setFilters()
    {
      velocity_filters_.resize(DOF);
      for(unsigned i=0; i<DOF; i++) {
        velocity_filters_[i] = boost::make_shared<Butterworth<double> >(2,velocity_cutoff_(i));
      }
    }

    virtual void computeResolverRanges() 
    {
      Eigen::MatrixXd m_to_j_pos = interface->getMotorToJointPositionTransform();  
      this->joint_resolver_ranges = (m_to_j_pos.diagonal().array() * 2.0*M_PI).cwiseAbs().matrix();
    }

    virtual void initialize()
    {
      const Eigen::MatrixXd & mpos2jpos = interface->getMotorToJointPositionTransform();

      // Compute actual position
      const Eigen::VectorXd actual_position =
        this->joint_home_position 
        + mpos2jpos*(this->joint_resolver_offset - this->joint_home_resolver_offset);

      // Set the actual position
      interface->definePosition(actual_position);
      
      // Disable resolver reading now that we've calibrated
      this->read_resolver = false;
    }

    virtual void run()
    {
      // Disable resolver reading 
      this->read_resolver = false;
      this->setFilters();

      if(run_mode != RUN) {
        run_mode = RUN;
      }

    }

    virtual void idle()
    {
      // Disable resolver reading 
      this->read_resolver = true;
      run_mode = IDLE;
      // The following doesn't work as intended for some reason
      //interface->getSafetyModule()->setMode(barrett::SafetyModule::IDLE);
    }

    virtual void readSim() { } 
    virtual void writeSim() { }

    virtual void readDevice(ros::Time time, RTT::Seconds period)
    {
      if(period <= 0) {
        return;
      }

      // Poll the hardware
      try {
        interface->update();
        // Get the safety module status
        if (interface->getSafetyModule() != NULL) {
          this->last_safety_mode = this->safety_mode;
          this->safety_mode = interface->getSafetyModule()->getMode(true);
        }
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

#if 0
      // Exponentially smooth velocity 
      this->joint_velocity = 
        this->velocity_smoothing_factor*this->joint_velocity 
        + (1.0 - this->velocity_smoothing_factor)*raw_joint_velocity;
#else
      // Butterworth the velocity
      for(unsigned i=0; i<DOF; i++) {
        this->joint_velocity(i) = this->velocity_filters_[i]->eval(raw_joint_velocity(i));
      }
#endif

      // Store position
      this->joint_position = raw_joint_position;


      // Write to data ports
      this->joint_position_out.write(this->joint_position);
      this->joint_velocity_out.write(this->joint_velocity);

      // Publish state to ROS 
      if(this->joint_state_throttle.ready(0.01)) {
        // Update the joint state message
        this->joint_state.header.stamp = rtt_rosclock::host_now();
        this->joint_state.name = this->joint_names;
        Eigen::Map<Eigen::VectorXd>(this->joint_state.position.data(),DOF) = this->joint_position;
        Eigen::Map<Eigen::VectorXd>(this->joint_state.velocity.data(),DOF) = this->joint_velocity;
        Eigen::Map<Eigen::VectorXd>(this->joint_state.effort.data(),DOF) = this->joint_effort;
          
        // Publish
        this->joint_state_out.write(this->joint_state);

        // Read resolver angles
        if(this->read_resolver) {
          std::vector<barrett::Puck*> pucks = interface->getPucks();	
          for(size_t i=0; i<pucks.size(); i++) {
            this->joint_resolver_offset(i) = angles::normalize_angle(
                interface->getMotorPucks()[i].counts2rad(pucks[i]->getProperty(barrett::Puck::MECH)));
          }
          
          // Update the joint state message
          this->joint_resolver_state.header.stamp = rtt_rosclock::host_now();
          this->joint_resolver_state.name = this->joint_names;
          Eigen::Map<Eigen::VectorXd>(this->joint_resolver_state.position.data(),DOF) = this->joint_resolver_offset;

          // Publish
          this->joint_resolver_offset_out.write(this->joint_resolver_offset);
          this->joint_resolver_state_out.write(this->joint_resolver_state);
        }
      }
    }

    virtual void writeDevice(ros::Time time, RTT::Seconds period)
    {
      // Check if the effort command port is connected
      if(this->joint_effort_in.connected()) {
        // Read newest command from data ports 
        Eigen::VectorXd joint_effort_tmp(DOF);
        bool new_effort_cmd = this->joint_effort_in.readNewest(joint_effort_tmp) == RTT::NewData;

        // Do nothing if there's no new command
        if(!new_effort_cmd) {
          return;
        }
      
        // Make sure the effort command is the right size
        if(joint_effort_tmp.size() == (unsigned)DOF) {
          this->joint_effort_raw = joint_effort_tmp;
        } else {
          this->joint_effort_raw.setZero();
        }

      } else {
        // Not connected, zero the command
        this->joint_effort_raw.resize(DOF);
        this->joint_effort_raw.setZero();
      }

      switch(run_mode) {
        case RUN:
          switch(this->safety_mode) {
            case barrett::SafetyModule::ACTIVE:
              // Copy the raw input to the joint effor that we'll filter
              this->joint_effort = this->joint_effort_raw;
              break;
            case barrett::SafetyModule::IDLE:
              // If the safety mode was just switched from active to idle, run a software idle
              if(this->last_safety_mode == barrett::SafetyModule::ACTIVE) {
                this->idle();
              };
              // Set the effort command to zero
              this->joint_effort.setZero();
              break;
            case barrett::SafetyModule::ESTOP:
              // Set the effort command to zero
              this->joint_effort.setZero();
              break;
          };

          break;

        case IDLE:
          switch(this->safety_mode) {
            case barrett::SafetyModule::ACTIVE:
            case barrett::SafetyModule::IDLE:
            case barrett::SafetyModule::ESTOP:
              // If it's idled or not active, set the effort command to zero
              this->joint_effort.setZero();
              break;
          };

          break;
      };


      // Check effort limits
      for(size_t i=0; i<DOF; i++) {
        // Check if the joint effort is too high
        if(std::abs(this->joint_effort(i)) > this->warning_fault_ratio*this->joint_effort_limits[i]) 
        {
          if(this->warning_count[i] % 1000 == 0) {
            // This warning can kill heartbeats
            RTT::log(RTT::Warning) << "Commanded torque (" << this->joint_effort(i)
              << ") of joint (" << i << ") is within "<<(1.0-this->warning_fault_ratio)
              <<"\% of safety limit: " << this->joint_effort_limits[i] << RTT::endlog();
          }
          this->warning_count[i]++;

          // Check for fault
          if(std::abs(this->joint_effort(i)) > this->joint_effort_limits[i]) {
            RTT::log(RTT::Error) << "Commanded torque (" << this->joint_effort(i)
              << ") of joint (" << i << ") has exceeded safety limit: "
              << this->joint_effort_limits[i] << RTT::endlog();
            this->joint_effort.setZero();
            if(interface->getSafetyModule()->getMode(true) == barrett::SafetyModule::ACTIVE) {
              this->parent_service_->getOwner()->error();
              return;
            }
          }
        }
      }

      // Set the torques
      this->joint_effort_scaled = this->torque_scales_.array() * this->joint_effort.array();
      interface->setTorques(this->joint_effort_scaled);

      // Save the last effort command
      this->joint_effort_last = this->joint_effort;

      // Pass along commanded effort for anyone who cares
      this->joint_effort_out.write(this->joint_effort);
    }


  protected:
    //! libbarrett Interface
    boost::shared_ptr<barrett::LowLevelWam<DOF> > interface;
    std::vector<barrett::Puck*> wam_pucks;
    boost::shared_ptr<barrett::ProductManager> barrett_manager_;
    RunMode run_mode;

    std::vector<boost::shared_ptr<Butterworth<double> > > velocity_filters_;
    Eigen::VectorXd velocity_cutoff_;
    Eigen::VectorXd torque_scales_;
  };

}

#endif // ifndef __ORO_BARRETT_HW_WAM_DEVICE
