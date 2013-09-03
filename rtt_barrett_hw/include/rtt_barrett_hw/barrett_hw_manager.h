#ifndef __RTT_BARRETT_HW_BARRETT_HW_MANAGER
#define __RTT_BARRETT_HW_BARRETT_HW_MANAGER

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt/os/Semaphore.hpp>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/bus/can_socket.h>
#include <barrett/products/product_manager.h>

#include <libconfig.h++>

#include <urdf/model.h>

#include <rtt_barrett_interface/barrett_manager.h>

#include <rtt_barrett_hw/wam_hw_device.h>
#include <rtt_barrett_hw/hand_hw_device.h>

namespace rtt_barrett_hw {

  class BarrettHWManager : public rtt_barrett_interface::BarrettManager 
  {
  public:

    BarrettHWManager(const std::string &name);

    /** \brief Construct interfaces and connect to the CANBus
     *
     */
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    //! Send commands to robot
    void controlHook(RTT::Seconds time, RTT::Seconds period);
    //! Get state from robot
    void estimationHook(RTT::Seconds time, RTT::Seconds period);

    /** \brief Construct a BHand interface
     *
     * TODO: Add hand support
     */
    virtual bool configureHand(
        bool torque_sensors,
        const std::string &root_link_name) 
    { return false; }

    /** \brief Construct a WAM robot interface
     *
     * TODO: Add 4-DOF support
     */
    virtual bool configureWam4(const std::string &tip_joint_name)
    { return false; }

    /** \brief Construct a WAM robot interface
     *
     */
    virtual bool configureWam7(const std::string &tip_joint_name);

  private:

    /** \brief Configure a WAM robot on this bus
     *
     * This creates RTT services associated with the input and output ports
     * defined in the WamDevice struct.
     *
     */
    template <size_t DOF>
      bool configureWam(const std::string &tip_joint_name);

    //! Set the safety module mode barrett::SafetyModule::{ESTOP, IDLE, ACTIVE}
    bool setMode(barrett::SafetyModule::SafetyMode mode);

    //! Block until a given mode or timeout
    bool waitForMode(
        barrett::SafetyModule::SafetyMode mode,
        double timeout = 10.0,
        double poll_period = 0.1);

    //! \name libbarrett structures
    //\{
    int bus_id_;
    std::string config_path_;
    boost::shared_ptr<barrett::bus::CANSocket> canbus_;
    boost::shared_ptr<barrett::ProductManager> barrett_manager_;
    //\}

    //! \name Possible barrett products
    //\{
    //! WAM robot container
    boost::shared_ptr<rtt_barrett_interface::WamDeviceBase> wam_device_;
    //! Barrett hand container
    boost::shared_ptr<rtt_barrett_interface::HandDevice> hand_device_;
    //\}

    RTT::Seconds last_update_time_;

    //! An RTT timer class for polling / waiting for a given mode
    class BarrettModeTimer : public RTT::os::Timer 
    {
    public:

      //! Returns True if the desired mode is now set
      static bool WaitForMode(
          boost::shared_ptr<barrett::ProductManager> barrett_manager,
          barrett::SafetyModule::SafetyMode mode,
          RTT::Seconds timeout, 
          RTT::Seconds poll_period) 
      {
        BarrettModeTimer timer(barrett_manager,mode,poll_period);
        RTT::Seconds now = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());
        return timer.ready_sem_.waitUntil(now + timeout);
      }

      //! Called when the timer semaphore times out once per polling cycle
      virtual void timeout(RTT::os::Timer::TimerId timer_id) {
        if(barrett_manager_->getSafetyModule()->getMode() == mode_) {
          ready_sem_.signal();
        }
      }

    private:

      //! Construct a timer and start polling the status mode
      BarrettModeTimer(
          boost::shared_ptr<barrett::ProductManager> barrett_manager,
          barrett::SafetyModule::SafetyMode mode,
          RTT::Seconds poll_period) : 
        RTT::os::Timer(1), 
        barrett_manager_(barrett_manager), 
        ready_sem_(0),
        mode_(mode)
      {
        this->startTimer(0,poll_period);
      }

      boost::shared_ptr<barrett::ProductManager> barrett_manager_;
      RTT::os::Semaphore ready_sem_;
      barrett::SafetyModule::SafetyMode mode_;
    };
  };

  BarrettHWManager::BarrettHWManager(const std::string &name) :
    rtt_barrett_interface::BarrettManager(name),
    bus_id_(0),
    config_path_("")
  {
    this->addProperty("bus_id",bus_id_)
      .doc("The CANBus port [0-n].");

    this->addProperty("config_path",config_path_)
      .doc("The path to the libbarrett config file (leave blank for default path).");
  }

  bool BarrettHWManager::configureHook()
  {
    // Create a new bus
    canbus_.reset(new barrett::bus::CANSocket(bus_id_));

    // Create a new manager
    barrett_manager_.reset(
        new barrett::ProductManager(
          config_path_.length() > 0 ? config_path_.c_str() : NULL /* Use defailt config */,
          canbus_.get()));

    // Parse URDF from string
    urdf_model_.initString(urdf_str_);

    return true;
  }

  bool BarrettHWManager::startHook()
  {
    // Zero the commands before the arm is activated
    if(wam_device_) {
      wam_device_->setZero();
    }

    if(!this->waitForMode(barrett::SafetyModule::ACTIVE)) {
      RTT::log(RTT::Error) << "Could not start Barrett Hardware, the safety "
        "module took too long to switch to the ACTIVE mode." << RTT::endlog();
      return false;
    }

    // Write to the configuration ports
    if(wam_device_) {
      wam_device_->readConfig();
    }

    // Initialize the last update time
    last_update_time_ = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());

    return true;
  }

  void BarrettHWManager::updateHook()
  {
    RTT::Seconds time = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());
    RTT::Seconds period = time - last_update_time_;

    this->controlHook(time, period);
    this->estimationHook(time, period);

    last_update_time_ = time;
  }

  void BarrettHWManager::controlHook(RTT::Seconds time, RTT::Seconds period)
  {
    if(wam_device_) {
      // Write the control command
      wam_device_->writeHW(time,period);
      wam_device_->writeHWCalibration(time,period);
    }
  }

  void BarrettHWManager::estimationHook(RTT::Seconds time, RTT::Seconds period)
  {
    if(wam_device_) {
      // Read the state estimation
      wam_device_->readHW(time,period);
    }
  }

  void BarrettHWManager::stopHook()
  {
    // Set the mode to IDLE
    this->setMode(barrett::SafetyModule::IDLE);
    // Wait for the system to become active
    if(!this->waitForMode(barrett::SafetyModule::IDLE)) {
      RTT::log(RTT::Warning) << "Could not IDLE the Barrett Hardware!" <<
        RTT::endlog();
    }
  }

  void BarrettHWManager::cleanupHook()
  {
    wam_device_.reset();
    barrett_manager_.reset();
    canbus_.reset();
  }

  bool BarrettHWManager::configureWam7(const std::string &tip_joint_name)
  {
    // Make sure we 're in the configured state, and not running
    if(!this->isConfigured() || this->isRunning()) {
      RTT::log(RTT::Error) << "Cannot configure WAM while the component is "
        "unconfigured or running." << RTT::endlog();
      return false;
    }

    // Check for a 7-DOF WAM
    if(!barrett_manager_->foundWam7()) {
      RTT::log(RTT::Error) << "Could not find a requested 7-DOF WAM on bus" <<
        bus_id_ << "." << RTT::endlog();
      return false;
    }

    // Create a new 7-DOF WAM
    if(this->configureWam<7>(tip_joint_name)) {
      return false;
    }

    return true;
  }

  template<size_t DOF>
    bool BarrettHWManager::configureWam(const std::string &tip_joint_name)
    {
      using namespace rtt_barrett_interface;

      try{

        // Construct a new wam device and "wam" service (interface and state storage)
        boost::shared_ptr<WamHWDevice<DOF> > wam_device(
            new WamHWDevice<DOF>(
              this->provides(),
              urdf_model_,
              tip_joint_name,
              barrett_manager_,
              barrett_manager_->getConfig().lookup(barrett_manager_->getWamDefaultConfigPath())));

        // Store the wam device
        wam_device_ = wam_device;
      } catch(std::runtime_error &ex) {
        RTT::log(RTT::Error) << "Could not configure " << DOF << "-DOF WAM: " <<
          ex.what() << RTT::endlog();
        return false;
      }

      return true;
    }

  bool BarrettHWManager::setMode(
      barrett::SafetyModule::SafetyMode mode)
  {
    if(this->isConfigured() && barrett_manager_) {
      barrett_manager_->getSafetyModule()->setMode(mode);
      return true;
    }

    return false;
  }

  bool BarrettHWManager::waitForMode(
      barrett::SafetyModule::SafetyMode mode,
      double timeout, 
      double poll_period) 
  {
    return BarrettModeTimer::WaitForMode( barrett_manager_, mode, timeout, poll_period);
  }

}

#endif // ifndef __RTT_BARRETT_HW_BARRETT_HW
