#ifndef __RTT_BARRETT_HW_WAM_DEVICE
#define __RTT_BARRETT_HW_WAM_DEVICE

#include <barrett/systems.h>

namespace rtt_barrett_hw {

  /** \brief State structure for a real WAM device
   *
   */
  template<size_t DOF>
    struct WamHWDevice : public rtt_barrett_interface::WamDevice<DOF>
  {
    /** \brief Construct a low-level WAM interface and extract joint information from
     * the URDF.
     */
    WamHWDevice(
        RTT::Service::shared_ptr parent_service, 
        const urdf::Model &urdf_model,
        const std::string &tip_joint,
        boost::shared_ptr<barrett::ProductManager> barrett_manager,
        const libconfig::Setting &wam_config) :
      WamDevice<DOF>(
          parent_service, 
          urdf_model, 
          tip_joint)
    {
      // Get the wam pucks
      std::vector<barrett::Puck*> wam_pucks = barrett_manager->getWamPucks();
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
      joint_resolver_ranges = (m_to_j_pos.diagonal().array() * 2.0*M_PI).cwiseAbs().matrix();
    }

    virtual void readHW(double time, double period)
    {
      // Poll the hardware
      try {
        interface->update();
      } catch (const std::runtime_error& e) {
        if (interface->getSafetyModule() != NULL  &&
            interface->getSafetyModule()->getMode(true) == barrett::SafetyModule::ESTOP) 
        {
          RTT::Log(RTT::Error) <<
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
        joint_velocity(i) = filters::exponentialSmoothing(
            raw_joint_velocity(i),
            joint_velocity(i),
            0.5);
      }

      // Store position
      joint_position = raw_joint_position;

      // Read resolver angles
      std::vector<barrett::Puck*> pucks = interface->getPucks();	
      for(size_t i=0; i<pucks.size(); i++) {
        joint_resolver_position(i) = pucks[i]->getProperty(barrett::Puck::MECH);
      }

      // Write to data ports
      joint_position_out.write(joint_position);
      joint_velocity_out.write(joint_velocity);
      joint_resolver_position_out.write(joint_resolver_position);
    }

    virtual void writeHW(double time, double period)
    {
      // Read newest command from data ports 
      if(joint_effort_in.readNewest(joint_effort) != RTT::NewData) {
        return;
      }

      static int warning_throttle = 0;

      for(size_t i=0; i<DOF; i++) {
        // Check if the joint effort st
        if(std::abs(joint_effort(i)) > joint_effort_limits[i]) {
          if(warning++ % 1000) {
            RTT::Log(RTT::Warn) << "Commanded torque (" << joint_effort(i)
              << ") of joint (" << i << ") exceeded safety limits! They have "
              "been truncated to: +/- " << joint_effort_limits[i] << RTT::endlog();
          }
          // Truncate this joint torque
          joint_effort(i) = std::max(
              std::min(joint_effort(i), joint_effort_limits[i]),
              -1.0*joint_effort_limits[i]);
        }
      }

      // Set the torques
      interface->setTorques(joint_effort);
    }

    virtual void writeHWCalibration(double time, double period)
    {
      // Read newest calibration input
      if( joint_calibration_burn_offsets_in.readNewest(joint_calibration_burn_offsets) != RTT::NewData 
          && joint_calibration_status_in.readNewest(joint_calibration_status) != RTT::NewData)
      {
        return;
      }

      // If not calibrated, servo estimated position to calibration position
      static int calib_decimate = 0;

      if(!calibrated && calib_decimate++ % 0) {

        // Check if each joint is calibrated, if
        bool all_joints_calibrated = true;
        for(size_t i=0; i<DOF; i++) {
          if(!all_joints_calibrated) {
            joint_calibration_burn_offsets = joint_position;
          }
          all_joints_calibrated = all_joints_calibrated && joint_calibration_status[i] == 1;
        }

        if(all_joints_calibrated) {

#if 0
          // Setting the positions cannot violate the velocity limits
          // We need to update them incrementally
          double minimum_time = calibration_burn_offsets_.cwiseQuotient(velocity_limits_).cwiseAbs().maxCoeff();
          double step = std::min(1.0,std::max(period.toSec()/minimum_time,0.0));

          calibration_burn_offsets_ -= (step)*calibration_burn_offsets_;
          joint_offsets_.data += (step)*calibration_burn_offsets_;

          static int decimate =0;
          if(decimate++ > 100) {
            ROS_INFO_STREAM("Adjusting offset by: "<<step<<" minimum time: "<<minimum_time);
            decimate = 0;
          }
#endif

          // Assign the positions to the current robot configuration
          joint_calibration_burn_offsets.setZero();
          interface->definePosition(joint_calibration_burn_offsets);

          //if(std::abs(step-1.0) < 1E-4) {
          calibrated = true;
          //}

          RTT::Log(RTT::Info) << "Zeroed joints." << RTT::endlog();
        }
      }
    }

    //! libbarrett Interface
    boost::shared_ptr<barrett::LowLevelWam<DOF> > interface;
  };

}

#endif // ifndef __RTT_BARRETT_HW_WAM_DEVICE
