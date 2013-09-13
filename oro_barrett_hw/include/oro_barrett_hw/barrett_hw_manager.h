#ifndef __ORO_BARRETT_HW_BARRETT_HW_MANAGER
#define __ORO_BARRETT_HW_BARRETT_HW_MANAGER

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt/os/Semaphore.hpp>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/bus/can_socket.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/products/product_manager.h>

#include <libconfig.h++>

#include <urdf/model.h>

#include <oro_barrett_interface/barrett_manager.h>

#include <oro_barrett_hw/wam_hw_device.h>
#include <oro_barrett_hw/hand_hw_device.h>

namespace oro_barrett_hw {

  class BarrettHWManager : public oro_barrett_interface::BarrettManager 
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
    boost::shared_ptr<barrett::bus::BusManager> bus_manager_;
    boost::shared_ptr<barrett::ProductManager> barrett_manager_;
    //\}

    //! \name Possible barrett products
    //\{
    //! WAM robot container
    boost::shared_ptr<oro_barrett_interface::WamDeviceBase> wam_device_;
    //! Barrett hand container
    boost::shared_ptr<oro_barrett_interface::HandDevice> hand_device_;
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
        RTT::Seconds now = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());
        BarrettModeTimer timer(barrett_manager,mode);
        return 
          timer.startTimer(0,poll_period) 
          && timer.ready_sem_.waitUntil(now + timeout);
      }

      //! Called when the timer semaphore times out once per polling cycle
      virtual void timeout(RTT::os::Timer::TimerId timer_id) {
        RTT::log(RTT::Debug) << "Checking safety module mode...." << RTT::endlog();
        try { 
          if( barrett_manager_ 
              && barrett_manager_->getSafetyModule() 
              && barrett_manager_->getSafetyModule()->getMode(true) == mode_) 
          {
            RTT::log(RTT::Debug) << "Requested mode enabled." << RTT::endlog();
            ready_sem_.signal();
          }
        } catch( std::runtime_error &err) {
          RTT::log(RTT::Error) << "Could not query safety module: " << err.what() << RTT::endlog();
          this->killTimer(0);
        }
      }

    private:

      //! Construct a timer and start polling the status mode
      BarrettModeTimer(
          boost::shared_ptr<barrett::ProductManager> barrett_manager,
          barrett::SafetyModule::SafetyMode mode) : 
        RTT::os::Timer(1,ORO_SCHED_RT), 
        barrett_manager_(barrett_manager), 
        ready_sem_(0),
        mode_(mode)
      {  }

      boost::shared_ptr<barrett::ProductManager> barrett_manager_;
      RTT::os::Semaphore ready_sem_;
      barrett::SafetyModule::SafetyMode mode_;
    };
  };

  BarrettHWManager::BarrettHWManager(const std::string &name) :
    oro_barrett_interface::BarrettManager(name),
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
    try {
      if(!bus_manager_) {
        bus_manager_.reset(new barrett::bus::BusManager(bus_id_));
      }
    } catch(std::runtime_error &err) {
      RTT::log(RTT::Error) << "Could not initialize barret CANBus interface on bus "<<bus_id_<<": " << err.what() << RTT::endlog();
      return false;
    }

    // Create a new manager
    try {
      if(!barrett_manager_) {
        barrett_manager_.reset(
            new barrett::ProductManager(
              config_path_.length() > 0 ? config_path_.c_str() : NULL /* Use defailt config */,
              bus_manager_.get()));
      }

      barrett_manager_->wakeAllPucks();
      barrett_manager_->getSafetyModule();

      // Wait for the system to be idle
      this->setMode(barrett::SafetyModule::IDLE);
      if(!this->waitForMode(barrett::SafetyModule::IDLE)) {
        RTT::log(RTT::Error) << "Could not IDLE the Barrett Hardware!" <<
          RTT::endlog();
        return false;
      }

    } catch(std::runtime_error &err) {
      RTT::log(RTT::Error) << "Could not create barrett product manager: " << err.what() << RTT::endlog();
      return false;
    }

    // Parse URDF from string
    if(!urdf_model_.initString(urdf_str_)) {
      return false;
    }

    return true;
  }

  bool BarrettHWManager::startHook()
  {
    // Zero the commands before the arm is activated
    if(wam_device_) {
      wam_device_->setZero();
    }

    /*if(!this->waitForMode(barrett::SafetyModule::ACTIVE)) {*/
    /*RTT::log(RTT::Error) << "Could not start Barrett Hardware, the safety "*/
    /*"module took too long to switch to the ACTIVE mode." << RTT::endlog();*/
    /*return false;*/
    /*}*/

    // Write to the configuration ports
    if(wam_device_) {
      wam_device_->readConfig();
    }

    // Initialize the last update time
    last_update_time_ = 
      RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());

    // Read the state estimation
    try {
      if(wam_device_) {
        wam_device_->readHW(last_update_time_,this->getPeriod());
      }
    } catch(std::runtime_error &err) {
      RTT::log(RTT::Error) << "Could not start WAM: " << err.what() << RTT::endlog();
      return false;
    }

    return true;
  }

  void BarrettHWManager::updateHook()
  {
    RTT::Seconds time = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());
    RTT::Seconds period = time - last_update_time_;

    if(wam_device_) {
      try {
        // Read the state estimation
        wam_device_->readHW(time,period);
      } catch(std::runtime_error &err) {
        RTT::log(RTT::Error) << "Could not read the WAM state: " << err.what() << RTT::endlog();
        this->error();
      }
      try {
        // Write the control command (force the write if the system is idle)
        wam_device_->writeHW(time,period);
        /*wam_device_->writeHWCalibration(time,period);*/
      } catch(std::runtime_error &err) {
        RTT::log(RTT::Error) << "Could not write the WAM command: " << err.what() << RTT::endlog();
        this->error();
      }
    }

    last_update_time_ = time;
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
    barrett_manager_->cleanUpAfterEstop();
    wam_device_.reset();
    barrett_manager_.reset();
    if(bus_manager_) {
      bus_manager_->close();
    }
    bus_manager_.reset();
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
    if(!this->configureWam<7>(tip_joint_name)) {
      return false;
    }

    return true;
  }

  template<size_t DOF>
    bool BarrettHWManager::configureWam(const std::string &tip_joint_name)
    {
      using namespace oro_barrett_interface;

      try{
        // Construct a new wam device and "wam" service (interface and state storage)
        if(!wam_device_) {
          wam_device_.reset(
              new WamHWDevice<DOF>(
                this->provides(),
                urdf_model_,
                tip_joint_name,
                barrett_manager_,
                barrett_manager_->getConfig().lookup(barrett_manager_->getWamDefaultConfigPath())));
        }
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

#endif // ifndef __ORO_BARRETT_HW_BARRETT_HW
