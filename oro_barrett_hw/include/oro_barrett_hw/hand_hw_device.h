#ifndef __ORO_BARRETT_HW_HAND_HW_DEVICE
#define __ORO_BARRETT_HW_HAND_HW_DEVICE

#include <barrett/systems.h>

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
    virtual void readHW(RTT::Seconds time, RTT::Seconds period);
    virtual void writeHW(RTT::Seconds time, RTT::Seconds period);

    virtual void setPositionMode(unsigned int joint_id);
    virtual void setVelocityMode(unsigned int joint_id);
    virtual void setEffortMode(unsigned int joint_id);

    HandHWDevice(
        RTT::Service::shared_ptr parent_service, 
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix,
        boost::shared_ptr<barrett::ProductManager> barrett_manager);

    virtual void open();
    virtual void close();

    enum Mode {
      UNINITIALIZED = 0,
      INITIALIZE,
      IDLE,
      RUN 
    };

    enum InitState {
      INIT_FINGERS = 0,
      SEEK_FINGERS,
      INIT_SPREAD,
      SEEK_SPREAD,
      INIT_CLOSE
    };
      
  private:
    RTT::Seconds last_read_time, last_write_time;
    RTT::Seconds min_period;
    Mode mode;
    InitState init_state;
    barrett::Hand *interface;
    const std::vector<barrett::Puck*>& pucks;

    // Mode bitmasks
    unsigned int 
      mode_pos,
      mode_vel,
      mode_eff;
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
    min_period(0.01),
    mode(UNINITIALIZED),
    interface(barrett_manager->getHand()),
    pucks(barrett_manager->getHandPucks()),
    mode_pos(0x0),
    mode_vel(0x0),
    mode_eff(0x0)
  { }

  void HandHWDevice::initialize()
  {
    if(mode == UNINITIALIZED) {
      mode = INITIALIZE;
      init_state = INIT_FINGERS;
    }
  }

  void HandHWDevice::idle()
  {
    if(mode != UNINITIALIZED) {
      using namespace barrett;
      interface->open(Hand::GRASP,false);
      interface->close(Hand::SPREAD,false);
      interface->idle();
      mode = IDLE;
    } else {
      RTT::log(RTT::Warning) << "You must initialize the BHand before it can be IDLEd." << RTT::endlog();
    }
  }

  void HandHWDevice::readHW(RTT::Seconds time, RTT::Seconds period)
  {
    if(time - last_read_time < min_period) {
      return;
    }

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
      //Eigen::Map<Eigen::VectorXd>(this->joint_state.effort.data(),8) = this->joint_effort;

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

    switch(mode) {
      case UNINITIALIZED:
        break;
      case INITIALIZE:
        {
          using namespace barrett;

          switch(init_state) { 
            case INIT_FINGERS:
              for (size_t i = 0; i < barrett::Hand::DOF-1; ++i) {
                pucks[i]->setProperty(barrett::Puck::CMD, 13);
              }
              init_state = SEEK_FINGERS;
              break;
            case SEEK_FINGERS:
              if(interface->doneMoving(Hand::WHOLE_HAND, true)) {
                init_state = INIT_SPREAD;
              }
              break;
            case INIT_SPREAD:
              pucks[barrett::Hand::SPREAD_INDEX]->setProperty(barrett::Puck::CMD, 13);
              init_state = SEEK_SPREAD;
              break;
            case SEEK_SPREAD:
              if(interface->doneMoving(Hand::WHOLE_HAND, true)) {
                init_state = INIT_CLOSE;
              }
              break;
            case INIT_CLOSE:
              interface->close(Hand::GRASP,false);
              mode = RUN;
              break;
          };
          break;
        }
      case IDLE:
        break;
      case RUN:
        {
          bool new_pos_cmd = (joint_position_in.read(joint_position_cmd) == RTT::NewData);
          bool new_vel_cmd = (joint_velocity_in.read(joint_velocity_cmd) == RTT::NewData);
          bool new_eff_cmd = (joint_effort_in.read(joint_effort_cmd) == RTT::NewData);

          bool new_joint_cmd = (joint_cmd_in.read(joint_cmd) == RTT::NewData);

          // Check sizes
          if(joint_velocity_cmd.size() != 4 ||
             joint_position_cmd.size() != 4 ||
             joint_position_cmd.size() != 4)
          {
            RTT::log(RTT::Error) << "Input command size msimatch!" << RTT::endlog();
            return;
          }

          if(new_joint_cmd) {

            joint_position_cmd.setZero();
            joint_effort_cmd.setZero();

            for(unsigned int i=0; i<4; i++) {
              switch(joint_cmd.mode[i]) {
                case oro_barrett_msgs::BHandCmd::CMD_POS:
                  this->setPositionMode(i);
                  new_pos_cmd = true;
                  joint_position_cmd[i] = joint_cmd.cmd[i];
                  break;
                case oro_barrett_msgs::BHandCmd::CMD_VEL:
                  this->setVelocityMode(i);
                  new_vel_cmd = true;
                  joint_velocity_cmd[i] = joint_cmd.cmd[i];
                  break;
                case oro_barrett_msgs::BHandCmd::CMD_EFF:
                  this->setEffortMode(i);
                  new_eff_cmd = true;
                  joint_effort_cmd[i] = joint_cmd.cmd[i];
                  break;
              };
            }
          }

          // Send commands
          if(new_pos_cmd) { interface->trapezoidalMove(joint_position_cmd, mode_pos, false); }
          if(new_vel_cmd) { RTT::log(RTT::Error) << "BHand Velocity command not supported! (sorry?)" << RTT::endlog(); }
          if(new_eff_cmd) { interface->setTorqueCommand(joint_effort_cmd, mode_eff); }
        }
        break;
    };

    last_write_time = time;
  }

  void HandHWDevice::setPositionMode(unsigned int joint_index) {
    unsigned int joint_bit = (1<<joint_index);

    if(mode_pos & joint_bit) {
      return;
    }

    mode_pos |= joint_bit;
    mode_vel &= ~joint_bit;
    mode_eff &= ~joint_bit;

    //interface->setPositionMode(mode_pos);
  }
  void HandHWDevice::setVelocityMode(unsigned int joint_index) {
    unsigned int joint_bit = (1<<joint_index);

    if(mode_vel & joint_bit) {
      return;
    }

    mode_pos &= ~joint_bit;
    mode_vel |= joint_bit;
    mode_eff &= ~joint_bit;

    //interface->setTorqueMode(mode_vel);
  }
  void HandHWDevice::setEffortMode(unsigned int joint_index) {
    unsigned int joint_bit = (1<<joint_index);

    if(mode_eff & joint_bit) {
      return;
    }

    mode_pos &= ~joint_bit;
    mode_vel &= ~joint_bit;
    mode_eff |= joint_bit;

    interface->setTorqueMode(mode_eff);
  }

  void HandHWDevice::open() {
    interface->open(barrett::Hand::GRASP, false);
  }

  void HandHWDevice::close() {
    interface->close(barrett::Hand::GRASP, false);
  }
}

#endif // ifnedf __ORO_BARRETT_HW_HAND_DEVICE
