
#include <rtt/Component.hpp>

#include <oro_barrett_hw/barrett_hw_manager.h>

using namespace oro_barrett_hw;

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

bool BarrettHWManager::configureWam7(const std::string &urdf_prefix)
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
  if(!this->configureWam<7>(urdf_prefix)) {
    return false;
  }

  return true;
}

  template<size_t DOF>
bool BarrettHWManager::configureWam(const std::string &urdf_prefix)
{
  using namespace oro_barrett_interface;

  try{
    // Construct a new wam device and "wam" service (interface and state storage)
    if(!wam_device_) {
      wam_device_.reset(
          new WamHWDevice<DOF>(
            this->provides(),
            urdf_model_,
            urdf_prefix,
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

ORO_LIST_COMPONENT_TYPE(oro_barrett_hw::BarrettHWManager);
ORO_CREATE_COMPONENT_LIBRARY()

