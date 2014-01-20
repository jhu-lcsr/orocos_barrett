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

    HandHWDevice(
        RTT::Service::shared_ptr parent_service, 
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix,
        boost::shared_ptr<barrett::ProductManager> barrett_manager);

    virtual void open();
    virtual void close();

    enum Mode {
      IDLE = 0,
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
      

  private:
    RTT::Seconds last_read_time, last_write_time;
    RTT::Seconds min_period;
    Mode mode;
    InitState init_state;
    barrett::Hand *interface;
    const std::vector<barrett::Puck*>& pucks;
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
    mode(IDLE),
    interface(barrett_manager->getHand()),
    pucks(barrett_manager->getHandPucks())
  { }

  void HandHWDevice::initialize()
  {
    mode = INITIALIZE;
    init_state = INIT_FINGERS;
  }

  void HandHWDevice::idle()
  {
    using namespace barrett;
    interface->open(Hand::GRASP,false);
    interface->close(Hand::SPREAD,false);
    interface->idle();
    mode = IDLE;
  }

  void HandHWDevice::readHW(RTT::Seconds time, RTT::Seconds period)
  {
    if(time - last_read_time < min_period) {
      return;
    }

    switch(mode) {
      case IDLE:
        break;
      case INITIALIZE:
        break;
      case RUN:
        // Poll the hardware
        try {
          interface->update(barrett::Hand::S_POSITION|barrett::Hand::S_FINGERTIP_TORQUE,true);
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
        break;
    };

    last_read_time = time;
  }

  void HandHWDevice::writeHW(RTT::Seconds time, RTT::Seconds period)
  {
    if(time - last_write_time < min_period) {
      return;
    }

    switch(mode) {
      case IDLE:
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
      case RUN:
        break;
    };

    last_write_time = time;
  }

  void HandHWDevice::open() {
    interface->open(barrett::Hand::GRASP, false);
  }

  void HandHWDevice::close() {
    interface->close(barrett::Hand::GRASP, false);
  }
}

#endif // ifnedf __ORO_BARRETT_HW_HAND_DEVICE
