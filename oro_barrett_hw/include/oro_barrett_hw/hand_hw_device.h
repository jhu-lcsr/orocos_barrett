#ifndef __ORO_BARRETT_HW_HAND_HW_DEVICE
#define __ORO_BARRETT_HW_HAND_HW_DEVICE

#include <barrett/systems.h>
#include <barrett/products/puck.h>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <oro_barrett_interface/hand_device.h>

namespace oro_barrett_hw {
  // State structure for a Hand
  class HandHWDevice : public oro_barrett_interface::HandDevice
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

    virtual void readHW(RTT::Seconds time, RTT::Seconds period);
    virtual void writeHW(RTT::Seconds time, RTT::Seconds period);

    HandHWDevice(
        RTT::Service::shared_ptr parent_service, 
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix,
        boost::shared_ptr<barrett::ProductManager> barrett_manager);

    virtual void open();
    virtual void close();

    enum Mode {
      UNINITIALIZED = 0,
      IDLE,
      INITIALIZE,
      RUN 
    };

    enum InitState {
      INIT_FINGERS = 0,
      SEEK_FINGERS,
      INIT_SPREAD,
      SEEK_SPREAD,
      INIT_CLOSE
    };
  protected:

    class HandInterface : public barrett::Hand {
    public:

      static const unsigned int N_PUCKS = 4;

      HandInterface(const std::vector<barrett::Puck*>& pucks) :
        barrett::Hand(pucks)
      {
      }

      bool isInitialized() {
        using namespace barrett;
        int statuses[N_PUCKS];
        group.getProperty(Puck::TSTOP, statuses, true);
        
        for(unsigned int i=0; i<N_PUCKS; i++) {
          RTT::log(RTT::Info) << "Puck " <<i<< " has status "<<statuses[i] << RTT::endlog();
        }

        return true;
      }

      void getFingertipTorque(Eigen::VectorXd torques) {
        using namespace barrett;
        int props[N_PUCKS];
        group.getProperty(Puck::SG, props, true);
        for(unsigned int i=0; i<N_PUCKS; i++) {
          torques[i] = props[i];
        }
      }

      void getTemp(Eigen::VectorXd &temps) {
        using namespace barrett;
        int props[N_PUCKS];
        temps.resize(N_PUCKS);

        group.getProperty(Puck::TEMP, props, true);
        for(unsigned int i=0; i<N_PUCKS; i++) {
          temps[i] = props[i];
        }
      }

      void setCompliance(bool enable) {
        using namespace barrett;

        for(unsigned i=0; i<3; i++) {
          pucks[i]->setProperty(Puck::TSTOP, (enable) ? (50) : (0), false);
        }
        pucks[3]->setProperty(Puck::TSTOP, (enable) ? (150) : (0), false);
      }

      void setTorqueMode(unsigned int digits) 
      {
        setProperty
          (digits, 
           barrett::Puck::MODE, 
           barrett::MotorPuck::MODE_TORQUE);
      }

      void setPositionMode(unsigned int digits) 
      {
        setProperty(
            digits, 
            barrett::Puck::MODE, 
            barrett::MotorPuck::MODE_PID);
      }

      void setVelocityMode(unsigned int digits) 
      {
        setProperty(
            digits,
            barrett::Puck::MODE, 
            barrett::MotorPuck::MODE_VELOCITY);
      }

      void setTrapezoidalMode(unsigned int digits) 
      {
        setProperty(
            digits, 
            barrett::Puck::MODE, 
            barrett::MotorPuck::MODE_TRAPEZOIDAL);
      }

      void setVelocityCommand(const Eigen::VectorXd& jv, unsigned int digits) {
        setProperty(
            digits,
            barrett::Puck::V,
            (j2pp.array() * jv.array()).matrix() / 1000.0);
      }

      void setTrapezoidalCommand(const Eigen::VectorXd& jp, unsigned int digits) {
        setProperty(
            digits, 
            barrett::Puck::E, 
            (j2pp.array() * jp.array()).matrix());
      }

      static const int CMD_HI = 13;
      static const int CMD_M = 19;

    private:

    };
      
  private:
    RTT::Seconds last_read_time, last_write_time;
    RTT::Seconds min_period;
    Mode mode;
    InitState init_state;
    const std::vector<barrett::Puck*>& pucks;
    HandInterface *interface;

    // Mode bitmasks
    unsigned int 
      mode_torque,
      mode_position,
      mode_velocity,
      mode_trapezoidal;

    bool modes_changed;

    Eigen::VectorXd temperature;
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
    last_read_time(0.0),
    last_write_time(0.0),
    min_period(0.1),
    mode(UNINITIALIZED),
    pucks(barrett_manager->getHandPucks()),
    interface(new HandHWDevice::HandInterface(pucks)),
    mode_torque(0x0),
    mode_position(0x0),
    mode_velocity(0x0),
    mode_trapezoidal(0x0),
    modes_changed(false),
    temperature(4)
  { 
    parent_service->provides("hand")->addProperty("temperature",temperature);
  }

  void HandHWDevice::initialize()
  {
    if(mode == UNINITIALIZED) {
      mode = INITIALIZE;
      init_state = INIT_FINGERS;
    }
  }

  void HandHWDevice::idle()
  {
    interface->isInitialized();

    if(mode != UNINITIALIZED) {
      using namespace barrett;
      interface->open(Hand::GRASP,false);
      interface->close(Hand::SPREAD,false);
      interface->idle();
      mode = RUN;
    } else {
      RTT::log(RTT::Warning) << "You must initialize the BHand before it can be IDLEd." << RTT::endlog();
    }
  }

  void HandHWDevice::run()
  {
    mode = RUN;
  }

  void HandHWDevice::setCompliance(bool enable) {
    interface->setCompliance(enable);
  }

  void HandHWDevice::readHW(RTT::Seconds time, RTT::Seconds period)
  {
    if(time - last_read_time < min_period) {
      return;
    }

    interface->getTemp(temperature);

    if(mode == UNINITIALIZED) {
      return;
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

    // Get the state, and re-shape it
    Eigen::Vector4d raw_inner_positions = interface->getInnerLinkPosition();
    Eigen::Vector4d raw_outer_positions = interface->getOuterLinkPosition();

    joint_position(0) = raw_inner_positions(3);
    joint_position(1) = raw_inner_positions(3);
    joint_position.block<3,1>(2,0) = raw_inner_positions.block<3,1>(0,0);
    joint_position.block<3,1>(5,0) = raw_outer_positions.block<3,1>(0,0);

    joint_position_out.write(joint_position);

    // Publish state to ROS 
    if(this->joint_state_throttle.ready(0.02)) {
      // Update the joint state message
      this->joint_state.header.stamp = rtt_ros_tools::ros_rt_now();
      this->joint_state.name = this->joint_names;
      Eigen::Map<Eigen::VectorXd>(this->joint_state.position.data(),8) = this->joint_position;
      Eigen::Map<Eigen::VectorXd>(this->joint_state.velocity.data(),8) = this->joint_velocity;
      // TODO: Map knucle_torque into this
      //Eigen::Map<Eigen::VectorXd>(this->joint_state.effort.data(),8) = this->joint_torque;

      // Publish
      this->joint_state_out.write(this->joint_state);
    }

    last_read_time = time;
  }

  void HandHWDevice::writeHW(RTT::Seconds time, RTT::Seconds period)
  {
    if(time - last_write_time < min_period) {
      return;
    }

    if((temperature.array() > 65.0).any()) {
      this->idle();
      return;
    }

    switch(mode) {
      case UNINITIALIZED:
        break;
      case IDLE:
        break;
      case INITIALIZE:
        {
          using namespace barrett;

          switch(init_state) { 
            case INIT_FINGERS:
              for (size_t i = 0; i < barrett::Hand::DOF-1; ++i) {
                pucks[i]->setProperty(barrett::Puck::CMD, HandHWDevice::HandInterface::CMD_HI);
              }
              init_state = SEEK_FINGERS;
              break;
            case SEEK_FINGERS:
              if(interface->doneMoving(Hand::WHOLE_HAND, true)) {
                init_state = INIT_SPREAD;
              }
              break;
            case INIT_SPREAD:
              pucks[barrett::Hand::SPREAD_INDEX]->setProperty(barrett::Puck::CMD, HandHWDevice::HandInterface::CMD_HI);
              init_state = SEEK_SPREAD;
              break;
            case SEEK_SPREAD:
              if(interface->doneMoving(Hand::WHOLE_HAND, true)) {
                init_state = INIT_CLOSE;
              }
              break;
            case INIT_CLOSE:
              min_period = 0.03;
              interface->close(Hand::GRASP,false);
              mode = RUN;
              break;
          };
          break;
        }
      case RUN:
        {
          bool new_torque_cmd = (joint_torque_in.read(joint_torque_cmd) == RTT::NewData);
          bool new_position_cmd = (joint_position_in.read(joint_position_cmd) == RTT::NewData);
          bool new_velocity_cmd = (joint_velocity_in.read(joint_velocity_cmd) == RTT::NewData);
          bool new_trapezoidal_cmd = (joint_trapezoidal_in.read(joint_trapezoidal_cmd) == RTT::NewData);

          bool new_joint_cmd = (joint_cmd_in.read(joint_cmd) == RTT::NewData);

          // Check sizes
          if(joint_torque_cmd.size() != 4 ||
             joint_position_cmd.size() != 4 ||
             joint_velocity_cmd.size() != 4 ||
             joint_trapezoidal_cmd.size() != 4)
          {
            RTT::log(RTT::Error) << "Input command size msimatch!" << RTT::endlog();
            return;
          }

          // Parse the ROS command into command vectors and update modes if necessary
          if(new_joint_cmd) {
            joint_torque_cmd.setZero();
            joint_position_cmd.setZero();
            joint_velocity_cmd.setZero();
            joint_trapezoidal_cmd.setZero();

            for(unsigned int i=0; i<4; i++) {
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
              };
            }
          }

          // Update the modes if they've changed
          if(modes_changed) {
            RTT::log(RTT::Debug) << "Hand command modes changed." <<RTT::endlog();
            interface->setTorqueMode(mode_torque);
            interface->setPositionMode(mode_position);
            interface->setVelocityMode(mode_velocity);
            interface->setTrapezoidalMode(mode_trapezoidal);
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
            interface->setVelocityCommand(joint_velocity_cmd, mode_velocity);
          }
          if(new_trapezoidal_cmd) {
            if(interface->doneMoving(mode_trapezoidal, true)) {
              interface->setTrapezoidalMode(mode_trapezoidal);
            }
            interface->setTrapezoidalCommand(joint_trapezoidal_cmd, mode_trapezoidal); 
          }
        }
        break;
    };

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

    modes_changed = true;
  }

  void HandHWDevice::open() {
    interface->open(barrett::Hand::GRASP, false);
  }

  void HandHWDevice::close() {
    interface->close(barrett::Hand::GRASP, false);
  }
}

#endif // ifnedf __ORO_BARRETT_HW_HAND_DEVICE
