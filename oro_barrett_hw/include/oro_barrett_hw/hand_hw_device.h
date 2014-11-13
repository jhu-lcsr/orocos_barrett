#ifndef __ORO_BARRETT_HW_HAND_HW_DEVICE
#define __ORO_BARRETT_HW_HAND_HW_DEVICE

#include <barrett/systems.h>
#include <barrett/products/puck.h>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <oro_barrett_interface/hand_device.h>

namespace oro_barrett_hw {

  /** \brief Orocos/ROS interface for a Barrett Hand
   *
   *
   */
  class HandHWDevice : public oro_barrett_interface::HandDevice
  {
  public:
    virtual void initialize();
    virtual void idle();
    virtual void run();

    virtual void setSG(int lsg, int hsg);
    virtual void setCompliance(bool enable);
    void printState();

    virtual void setTorqueMode(unsigned int joint_index);
    virtual void setPositionMode(unsigned int joint_index);
    virtual void setVelocityMode(unsigned int joint_index);
    virtual void setTrapezoidalMode(unsigned int joint_index);
    virtual void setIdleMode(unsigned int joint_index);

    virtual void readDevice(ros::Time time, RTT::Seconds period);
    virtual void writeDevice(ros::Time time, RTT::Seconds period);

    virtual void readPorts();
    virtual void writePorts();

    virtual void open();
    virtual void close();

    HandHWDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix,
        boost::shared_ptr<barrett::ProductManager> barrett_manager);

    //! The update period during initialization
    static const double INIT_UPDATE_PERIOD = 0.1;
    //! The update period while running / idling
    static const double RUN_UPDATE_PERIOD = 1.0/30.0;

    //! Maximum allowable hand puck temperature
    static const double MAX_PUCK_TEMP = 65.0;

  protected:

    /** \brief Extension of libbarrett BHand interface to expose more capabilities
     *
     *
     */
    class HandInterface : public barrett::Hand
    {
    public:

      //! Hand Initialize Command
      static const int CMD_HI = 13;
      //! Hand Move Command
      static const int CMD_M = 19;

      HandInterface(const std::vector<barrett::Puck*>& pucks) :
        barrett::Hand(pucks)
      { }

      //! Query some hand properties
      bool isInitialized()
      {
        using namespace barrett;
        int statuses[N_PUCKS];
        group.getProperty(Puck::HSG, statuses, true);

        for(unsigned int i=0; i<N_PUCKS; i++) {
          RTT::log(RTT::Info) << "Puck " <<i<< " has status "<<statuses[i] << RTT::endlog();
        }

        return true;
      }

      //! Query some hand information
      void printState()
      {
        using namespace barrett;
        int hsg[N_PUCKS];
        group.getProperty(Puck::HSG, hsg, true);
        int sg[N_PUCKS];
        group.getProperty(Puck::SG, sg, true);

        //Eigen::VectorXd t(4);
        //this->getKnuckleTorque(t);

        for(unsigned int i=0; i<N_PUCKS; i++) {
          RTT::log(RTT::Warning) << "Puck " <<i<< " has HSG "<<hsg[i] << RTT::endlog();
          RTT::log(RTT::Warning) << "Puck " <<i<< " has SG "<<sg[i] << RTT::endlog();
        }
      }

      //! Get the torque at the distal knuckles
      void getKnuckleTorque(Eigen::VectorXd torques)
      {
        int props[N_PUCKS];

        group.getProperty<barrett::Puck::StandardParser>(barrett::Puck::SG, props, true);
        for(unsigned int i=0; i<N_PUCKS; i++) {
          torques[i] = props[i];
        }
      }

      //! Get the temperature of all the hand pucks
      void getTemp(Eigen::VectorXd &temps)
      {
        int props[N_PUCKS];
        temps.resize(N_PUCKS);

        group.getProperty(barrett::Puck::TEMP, props, true);
        for(unsigned int i=0; i<N_PUCKS; i++) {
          temps[i] = props[i];
        }
      }

      //! Enable or disable finger compliance (this does not work)
      void setCompliance(bool enable)
      {
        for(unsigned i=0; i<3; i++) {
          pucks[i]->setProperty(barrett::Puck::HSG, (enable) ? (0) : (4096), false);
          //pucks[i]->setProperty(barrett::Puck::LSG, (enable) ? (0) : (-255), false);
        }
        //pucks[3]->setProperty(barrett::Puck::HSG, (enable) ? (0) : (256), false);
      }

      void setSG(int lsg, int hsg)
      {
        for(unsigned i=0; i<3; i++) {
          pucks[i]->setProperty(barrett::Puck::LSG, lsg, false);
          pucks[i]->setProperty(barrett::Puck::HSG, hsg, false);
        }
      }

      //! Set the command mode to torque for a subset of bitmasked hand pucks
      void setTorqueMode(unsigned int digits)
      {
        setProperty
          (digits,
           barrett::Puck::MODE,
           barrett::MotorPuck::MODE_TORQUE);
      }

      //! Set the command mode to PID for a subset of bitmasked hand pucks
      void setPositionMode(unsigned int digits)
      {
        setProperty(
            digits,
            barrett::Puck::MODE,
            barrett::MotorPuck::MODE_PID);
      }

      //! Set the command mode to velocity for a subset of bitmasked hand pucks
      void setVelocityMode(unsigned int digits)
      {
        setProperty(
            digits,
            barrett::Puck::MODE,
            barrett::MotorPuck::MODE_VELOCITY);
      }

      //! Set the command mode to trapezoidal trajectory for a subset of bitmasked hand pucks
      void setTrapezoidalMode(unsigned int digits)
      {
        setProperty(
            digits,
            barrett::Puck::MODE,
            barrett::MotorPuck::MODE_TRAPEZOIDAL);
      }

      //! Set the velocity command for a subset of bitmasked hand pucks
      void setVelocityCommand(const Eigen::VectorXd& jv, unsigned int digits)
      {
        setProperty(
            digits,
            barrett::Puck::V,
            (j2pp.array() * jv.array()).matrix() / 1000.0);
      }

      //! Set the trapzeoidal command for a subset of bitmasked hand pucks
      void setTrapezoidalCommand(const Eigen::VectorXd& jp, unsigned int digits)
      {
        setProperty(
            digits,
            barrett::Puck::E,
            (j2pp.array() * jp.array()).matrix());
      }
    };

  private:
    //! Minimum execution period. This runs at 10Hz before hand initialization and 30Hz afterwards.
    RTT::Seconds min_period;
    //! Measured execution times
    ros::Time last_read_time, last_write_time;
    ros::Time last_temp_read_time;

    Eigen::Vector4d
      raw_inner_positions,
      raw_inner_positions_temp,
      raw_outer_positions,
      raw_outer_positions_temp,
      raw_inner_velocities,
      raw_outer_velocities;

    std::vector<int> raw_fingertip_torques;

    const std::vector<barrett::Puck*>& pucks;
    HandInterface *interface;

    Eigen::VectorXd temperature;
    void checkTemperature();


    Eigen::VectorXd
      joint_position_tmp,
      joint_velocity_tmp,
      joint_torque_cmd_tmp,
      joint_position_cmd_tmp,
      joint_velocity_cmd_tmp,
      joint_trapezoidal_cmd_tmp,
      center_of_mass_tmp;
    oro_barrett_msgs::BHandCmd
      joint_cmd_tmp;

    // Lock-free buffers for communicating with device thread
    RTT::base::Buffer<Eigen::VectorXd> position_buffer_;
    RTT::base::Buffer<Eigen::VectorXd> velocity_buffer_;

    RTT::base::Buffer<Eigen::VectorXd> torque_cmd_buffer_;
    RTT::base::Buffer<Eigen::VectorXd> position_cmd_buffer_;
    RTT::base::Buffer<Eigen::VectorXd> velocity_cmd_buffer_;
    RTT::base::Buffer<Eigen::VectorXd> trapezoidal_cmd_buffer_;

    RTT::base::Buffer<oro_barrett_msgs::BHandCmd> joint_cmd_buffer_;

  };

  HandHWDevice::HandHWDevice(
      RTT::Service::shared_ptr parent_service,
      const urdf::Model &urdf_model,
      const std::string &urdf_prefix,
      boost::shared_ptr<barrett::ProductManager> barrett_manager) :
    oro_barrett_interface::HandDevice(
        parent_service,
        urdf_model,
        urdf_prefix),
    min_period(RUN_UPDATE_PERIOD),
    last_read_time(0.0),
    last_write_time(0.0),
    last_temp_read_time(0.0),
    pucks(barrett_manager->getHandPucks()),
    interface(new HandHWDevice::HandInterface(pucks)),
    temperature(N_PUCKS),
    position_buffer_(1),
    velocity_buffer_(1),
    torque_cmd_buffer_(1),
    position_cmd_buffer_(1),
    velocity_cmd_buffer_(1),
    trapezoidal_cmd_buffer_(1),
    joint_cmd_buffer_(1)
  {
    parent_service->provides("hand")->addProperty("temperature",temperature);
    parent_service->provides("hand")->addOperation("printState", &HandHWDevice::printState, this, RTT::OwnThread);
    parent_service->provides("hand")->addOperation("setSG", &HandHWDevice::setSG, this, RTT::OwnThread);

    raw_inner_positions.setZero();
    raw_inner_positions_temp.setZero();
    raw_outer_positions.setZero();
    raw_outer_positions_temp.setZero();
    raw_inner_velocities.setZero();
    raw_outer_velocities.setZero();
  }

  void HandHWDevice::initialize()
  {
    min_period = INIT_UPDATE_PERIOD;
    init_state = INIT_FINGERS;
    run_mode = INITIALIZE;
  }

  void HandHWDevice::idle()
  {
    interface->open(barrett::Hand::GRASP,false);
    interface->close(barrett::Hand::SPREAD,false);
    interface->idle();
    run_mode = IDLE;
  }

  void HandHWDevice::run()
  {
    run_mode = RUN;
  }

  void HandHWDevice::setCompliance(bool enable)
  {
    interface->setCompliance(enable);
  }

  void HandHWDevice::setSG(int lsg, int hsg)
  {
    interface->setSG(lsg, hsg);
  }

  void HandHWDevice::printState()
  {
    interface->printState();
    for(unsigned int i=0; i<N_PUCKS; i++) {
      RTT::log(RTT::Warning) << "Puck " <<i<< " has SG "<<raw_fingertip_torques[i] << RTT::endlog();
    }
  }

  void HandHWDevice::readDevice(ros::Time time, RTT::Seconds period)
  {
    // Limit other operations
    if((time - last_read_time).toSec() < min_period)
    {
      return;
    }
    else if((time - last_temp_read_time).toSec() > 20.0)
    {
      // Check temperature every 20 seconds
      interface->getTemp(temperature);
      checkTemperature();
      last_temp_read_time = time;
    }

    // Poll the hardware
    try {
      interface->update(
          barrett::Hand::S_POSITION|barrett::Hand::S_FINGERTIP_TORQUE,
          true);
    } catch (const std::runtime_error& e) {
      RTT::log(RTT::Error) << "Could not read BHand state: " << e.what() << RTT::endlog();
      throw;
    }

    // Get the joint positions
    raw_inner_positions_temp = interface->getInnerLinkPosition();
    raw_outer_positions_temp = interface->getOuterLinkPosition();

    // Compute joint velocity
    raw_inner_velocities = (raw_inner_positions_temp-raw_inner_positions) / period;
    raw_outer_velocities = (raw_outer_positions_temp-raw_outer_positions) / period;

    // Save the joint positions
    raw_inner_positions = interface->getInnerLinkPosition();
    raw_outer_positions = interface->getOuterLinkPosition();

    // Get the fingertip torques
    raw_fingertip_torques = interface->getFingertipTorque();

    last_read_time = time;
  }

  void HandHWDevice::writePorts()
  {
    // Always compute and write center of mass
    this->computeCenterOfMass(center_of_mass);
    center_of_mass_out.write(center_of_mass);

    // Get the positions, and re-shape it
    joint_position(0) = raw_inner_positions(3);
    joint_position(1) = raw_inner_positions(3);
    joint_position.block<3,1>(2,0) = raw_inner_positions.block<3,1>(0,0);
    joint_position.block<3,1>(5,0) = raw_outer_positions.block<3,1>(0,0);
    
    // Get the velocity, and re-shape it
    joint_velocity(0) = raw_inner_velocities(3);
    joint_velocity(1) = raw_inner_velocities(3);
    joint_velocity.block<3,1>(2,0) = raw_inner_velocities.block<3,1>(0,0);
    joint_velocity.block<3,1>(5,0) = raw_outer_velocities.block<3,1>(0,0);

    joint_position_out.write(joint_position);
    joint_velocity_out.write(joint_velocity);

    // Publish state to ROS
    if(this->joint_state_throttle.ready(0.02)) {
      // Update the joint state message
      this->joint_state.header.stamp = rtt_rosclock::host_now();
      this->joint_state.name = this->joint_names;
      Eigen::Map<Eigen::VectorXd>(this->joint_state.position.data(),8) = this->joint_position;
      Eigen::Map<Eigen::VectorXd>(this->joint_state.velocity.data(),8) = this->joint_velocity;
      // TODO: Map knucle_torque into this
      //Eigen::Map<Eigen::VectorXd>(this->joint_state.effort.data(),8) = this->joint_torque;
      this->joint_state.effort[5] = raw_fingertip_torques[0]/2000.0;
      this->joint_state.effort[6] = raw_fingertip_torques[1]/2000.0;
      this->joint_state.effort[7] = raw_fingertip_torques[2]/2000.0;

      // Publish
      this->joint_state_out.write(this->joint_state);

      // Create a pose structure from the center of mass
      com_msg.header.stamp = rtt_rosclock::host_now();
      com_msg.pose.position.x = center_of_mass[0];
      com_msg.pose.position.y = center_of_mass[1];
      com_msg.pose.position.z = center_of_mass[2];
      this->center_of_mass_debug_out.write(com_msg);

      // Write out hand status
      this->status_msg.run_mode.value = this->run_mode;
      for(unsigned i=0; i<barrett::Hand::DOF; i++) {
        unsigned bit = 1<<i;
        if(mode_torque & bit)       this->status_msg.mode[i] = oro_barrett_msgs::BHandCmd::MODE_TORQUE;
        if(mode_position & bit)     this->status_msg.mode[i] = oro_barrett_msgs::BHandCmd::MODE_PID;
        if(mode_trapezoidal & bit)  this->status_msg.mode[i] = oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL;
        if(mode_velocity & bit)     this->status_msg.mode[i] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;
        if(mode_idle & bit)         this->status_msg.mode[i] = oro_barrett_msgs::BHandCmd::MODE_IDLE;
        this->status_msg.temperature[i] = this->temperature(i);
      }
      this->status_out.write(this->status_msg);

    }
  }

  void HandHWDevice::checkTemperature()
  {
    const double max_temp = MAX_PUCK_TEMP;
    if((temperature.array() > max_temp).any()) {
      RTT::log(RTT::Error) << "DANGER: The BHand temperature has exceeded the safe maximum of "<<max_temp<<"C. The hand has been IDLE'd. Please turn off the robot and let it cool down." << RTT::endlog();
      interface->idle();
      run_mode = IDLE;
      parent_service_->getOwner()->error();
    }
  }

  void HandHWDevice::readPorts()
  {
    // Read commands
    bool new_torque_cmd = (joint_torque_in.readNewest(joint_torque_cmd_tmp) == RTT::NewData);
    bool new_position_cmd = (joint_position_in.readNewest(joint_position_cmd_tmp) == RTT::NewData);
    bool new_velocity_cmd = (joint_velocity_in.readNewest(joint_velocity_cmd_tmp) == RTT::NewData);
    bool new_trapezoidal_cmd = (joint_trapezoidal_in.readNewest(joint_trapezoidal_cmd_tmp) == RTT::NewData);

    bool new_joint_cmd = (joint_cmd_in.readNewest(joint_cmd_tmp) == RTT::NewData);

    // Check sizes
    if(joint_torque_cmd.size() != N_PUCKS ||
       joint_position_cmd.size() != N_PUCKS ||
       joint_velocity_cmd.size() != N_PUCKS ||
       joint_trapezoidal_cmd.size() != N_PUCKS)
    {
      RTT::log(RTT::Error) << "Input command size msimatch!" << RTT::endlog();
      return;
    }

    // Put the command into the lock-free buffers
    if(new_torque_cmd) torque_cmd_buffer_.Push(joint_torque_cmd_tmp);
    if(new_position_cmd) position_cmd_buffer_.Push(joint_position_cmd_tmp);
    if(new_velocity_cmd) velocity_cmd_buffer_.Push(joint_velocity_cmd_tmp);
    if(new_trapezoidal_cmd) trapezoidal_cmd_buffer_.Push(joint_trapezoidal_cmd_tmp);
    if(new_joint_cmd) joint_cmd_buffer_.Push(joint_cmd_tmp);
  }

  void HandHWDevice::writeDevice(ros::Time time, RTT::Seconds period)
  {
    // Don't run too fast
    if((time - last_write_time).toSec() < min_period) {
      return;
    }

    switch(run_mode)
    {
      case IDLE:
        // Don't command the hand
        break;
      case INITIALIZE:
        {
          using namespace barrett;

          switch(init_state) {
            case INIT_FINGERS:
              for (size_t i = 0; i < Hand::DOF-1; ++i) {
                pucks[i]->setProperty(Puck::CMD, HandHWDevice::HandInterface::CMD_HI);
              }
              init_state = SEEK_FINGERS;
              break;
            case SEEK_FINGERS:
              if(interface->doneMoving(Hand::GRASP, true)) {
                init_state = INIT_SPREAD;
              }
              break;
            case INIT_SPREAD:
              pucks[Hand::SPREAD_INDEX]->setProperty(Puck::CMD, HandHWDevice::HandInterface::CMD_HI);
              init_state = SEEK_SPREAD;
              break;
            case SEEK_SPREAD:
              if(interface->doneMoving(Hand::SPREAD, true)) {
                init_state = INIT_CLOSE;
              }
              break;
            case INIT_CLOSE:
              // Increase loop rate
              min_period = RUN_UPDATE_PERIOD;
              interface->close(Hand::GRASP,false);
              run_mode = RUN;
              this->status_msg.initialized = true;
              break;
          };
          break;
        }
      case RUN:
        {
          bool new_idle_cmd = false;
          bool new_torque_cmd = torque_cmd_buffer_.Pop(joint_torque_cmd);
          bool new_position_cmd = position_cmd_buffer_.Pop(joint_position_cmd);
          bool new_velocity_cmd = velocity_cmd_buffer_.Pop(joint_velocity_cmd);
          bool new_trapezoidal_cmd = trapezoidal_cmd_buffer_.Pop(joint_trapezoidal_cmd);

          bool new_joint_cmd = joint_cmd_buffer_.Pop(joint_cmd);

          // Parse the ROS command into command vectors and update modes if necessary
          if(new_joint_cmd)
          {
            joint_torque_cmd.setZero();
            joint_position_cmd.setZero();
            joint_velocity_cmd.setZero();
            joint_trapezoidal_cmd.setZero();

            for(unsigned int i=0; i<N_PUCKS; i++) {
              switch(joint_cmd.mode[i]) {
                case oro_barrett_msgs::BHandCmd::MODE_TORQUE:
                  new_torque_cmd = true;
                  joint_torque_cmd[i] = joint_cmd.cmd[i];
                  this->setTorqueMode(i);
                  break;
                case oro_barrett_msgs::BHandCmd::MODE_PID:
                  new_position_cmd = true;
                  joint_position_cmd[i] = joint_cmd.cmd[i];
                  this->setPositionMode(i);
                  break;
                case oro_barrett_msgs::BHandCmd::MODE_VELOCITY:
                  new_velocity_cmd = true;
                  joint_velocity_cmd[i] = joint_cmd.cmd[i];
                  this->setVelocityMode(i);
                  break;
                case oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL:
                  new_trapezoidal_cmd = true;
                  joint_trapezoidal_cmd[i] = joint_cmd.cmd[i];
                  this->setTrapezoidalMode(i);
                  break;
                case oro_barrett_msgs::BHandCmd::MODE_IDLE:
                  new_idle_cmd = true;
                  this->setIdleMode(i);
                  break;
              };
            }
          }

          // Update the modes if they've changed
          if(modes_changed)
          {
            RTT::log(RTT::Debug) << "Hand command modes changed." <<RTT::endlog();
            interface->setTorqueMode      ( mode_torque & ~mode_position & ~mode_velocity & ~mode_trapezoidal & ~mode_idle);
            interface->setPositionMode    (~mode_torque &  mode_position & ~mode_velocity & ~mode_trapezoidal & ~mode_idle);
            interface->setVelocityMode    (~mode_torque & ~mode_position &  mode_velocity & ~mode_trapezoidal & ~mode_idle);
            interface->setTrapezoidalMode (~mode_torque & ~mode_position & ~mode_velocity &  mode_trapezoidal & ~mode_idle);
            interface->setVelocityMode    (~mode_torque & ~mode_position & ~mode_velocity & ~mode_trapezoidal &  mode_idle);
            modes_changed = false;
          }

          // Send commands
          if(new_torque_cmd) {
            if(interface->doneMoving(mode_torque, true)) {
              interface->setTorqueMode(mode_torque);
            }
            interface->setTorqueCommand(joint_torque_cmd, mode_torque);
          }
          if(new_position_cmd) {
            interface->setPositionCommand(joint_position_cmd, mode_position);
          }
          if(new_velocity_cmd) {
            //if(interface->doneMoving(mode_velocity, true)) {
              interface->setVelocityMode(mode_velocity);
            //}
            interface->setVelocityCommand(joint_velocity_cmd, mode_velocity);
          }
          if(new_trapezoidal_cmd) {
            if(interface->doneMoving(mode_trapezoidal, true)) {
              interface->setTrapezoidalMode(mode_trapezoidal);
            }
            interface->setTrapezoidalCommand(joint_trapezoidal_cmd, mode_trapezoidal);
          }
          if(new_idle_cmd) {
            interface->setVelocityMode(mode_idle);
            interface->setVelocityCommand(joint_idle_cmd, mode_idle);
          }
        }
        break;
    };

    // Store the write time for maintaining loop rate
    last_write_time = time;
  }

  void HandHWDevice::setTorqueMode(unsigned int joint_index)
  {
    unsigned int joint_bit = (1<<joint_index);

    if(mode_torque & joint_bit) {
      return;
    }

    RTT::log(RTT::Debug) << "Setting hand joint "<<joint_index<<" to TORQUE mode." <<RTT::endlog();

    mode_torque |= joint_bit;
    mode_position &= ~joint_bit;
    mode_velocity &= ~joint_bit;
    mode_trapezoidal &= ~joint_bit;
    mode_idle &= ~joint_bit;

    modes_changed = true;
  }

  void HandHWDevice::setPositionMode(unsigned int joint_index)
  {
    unsigned int joint_bit = (1<<joint_index);

    if(mode_position & joint_bit) {
      return;
    }

    RTT::log(RTT::Debug) << "Setting hand joint "<<joint_index<<" to PID mode." <<RTT::endlog();

    mode_torque &= ~joint_bit;
    mode_position |= joint_bit;
    mode_velocity &= ~joint_bit;
    mode_trapezoidal &= ~joint_bit;
    mode_idle &= ~joint_bit;

    modes_changed = true;
  }

  void HandHWDevice::setVelocityMode(unsigned int joint_index)
  {
    unsigned int joint_bit = (1<<joint_index);

    if(mode_velocity & joint_bit) {
      return;
    }

    RTT::log(RTT::Debug) << "Setting hand joint "<<joint_index<<" to VELOCITY mode." <<RTT::endlog();

    mode_torque &= ~joint_bit;
    mode_position &= ~joint_bit;
    mode_velocity |= joint_bit;
    mode_trapezoidal &= ~joint_bit;
    mode_idle &= ~joint_bit;

    modes_changed = true;
  }

  void HandHWDevice::setTrapezoidalMode(unsigned int joint_index)
  {
    unsigned int joint_bit = (1<<joint_index);

    if(mode_trapezoidal & joint_bit) {
      return;
    }

    RTT::log(RTT::Debug) << "Setting hand joint "<<joint_index<<" to TRAPEZOIDAL mode." <<RTT::endlog();

    mode_torque &= ~joint_bit;
    mode_position &= ~joint_bit;
    mode_velocity &= ~joint_bit;
    mode_trapezoidal |= joint_bit;
    mode_idle &= ~joint_bit;

    modes_changed = true;
  }

  void HandHWDevice::setIdleMode(unsigned int joint_index)
  {
    unsigned int joint_bit = (1<<joint_index);

    if(mode_idle & joint_bit) {
      return;
    }

    RTT::log(RTT::Debug) << "Setting hand joint "<<joint_index<<" to IDLE mode." <<RTT::endlog();

    mode_torque &= ~joint_bit;
    mode_position &= ~joint_bit;
    mode_velocity &= ~joint_bit;
    mode_trapezoidal &= ~joint_bit;
    mode_idle |= joint_bit;

    modes_changed = true;
  }

  void HandHWDevice::open()
  {
    interface->open(barrett::Hand::GRASP, false);
  }

  void HandHWDevice::close()
  {
    interface->close(barrett::Hand::GRASP, false);
  }
}

#endif // ifnedf __ORO_BARRETT_HW_HAND_DEVICE
