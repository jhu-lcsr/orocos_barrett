
#include <rtt/Component.hpp>

#include <oro_barrett_hw/barrett_hw_manager.h>

#include <barrett/products/puck.h>

#include <rtt_rosparam/rosparam.h>

using namespace oro_barrett_hw;

BarrettHWManager::BarrettHWManager(const std::string &name) :
  oro_barrett_interface::BarrettManager(name),
  bus_id_(0),
  config_path_(""),
  device_thread_(this),
  new_state_sem_(0),
  new_cmd_sem_(0)
{
  this->addProperty("bus_id",bus_id_)
    .doc("The CANBus port [0-n].");

  this->addProperty("config_path",config_path_)
    .doc("The path to the libbarrett config file (leave blank for default path).");

  this->addProperty("real_period",period_);
  this->addProperty("read_duration",read_duration_);
  this->addProperty("write_duration",write_duration_);
}

bool BarrettHWManager::configureHook()
{
  // Get the rosparam service requester
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");

  if(!rosparam) {
    RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
    return false;
  }

  // Get the ROS parameters
  rosparam->getAllComponentPrivate();

  // Parse URDF from string
  if(!urdf_model_.initString(urdf_str_)) {
    return false;
  }

  return device_thread_.start();
}

bool BarrettHWManager::startHook()
{
  // Initialize the last update time
  last_update_time_ = rtt_rosclock::rtt_now();
  return true;
}

bool BarrettHWManager::deviceStartHook()
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
            config_path_.c_str(),
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

  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");


  // Auto-configure optional WAM
  if(auto_configure_wam_) {
    
    switch(wam_dof_) {
      case 4: 
        if(!this->configureWam4Protected(wam_urdf_prefix_)) {
          RTT::log(RTT::Error) << "Unable to auto-configure 4-DOF WAM with URDF prefix \""<<wam_urdf_prefix_<<"\"." <<RTT::endlog();
          return false;
        }
        break;
      case 7:
        if(!this->configureWam7Protected(wam_urdf_prefix_)) {
          RTT::log(RTT::Error) << "Unable to auto-configure 4-DOF WAM with URDF prefix \""<<wam_urdf_prefix_<<"\"." <<RTT::endlog();
          return false;
        }
        break;
      default:
        RTT::log(RTT::Error) << "Unable to auto-configure WAM with URDF prefix "
          "\""<<wam_urdf_prefix_<<"\". DOF should be 4 or 7, but it's "
          <<wam_dof_<<"." <<RTT::endlog();
        return false;
    };

    // Get WAM ros parameters
    rosparam->getComponentPrivate("wam");
  }

  // Auto-configure optional BHand
  if(auto_configure_hand_) {
    if(!this->configureHand(hand_urdf_prefix_)) {
      RTT::log(RTT::Error) << "Unable to auto-configure BHand with URDF prefix \""<<hand_urdf_prefix_<<"\"." <<RTT::endlog();
      return false;
    }
    
    // Get BHand ros parameters
    rosparam->getComponentPrivate("hand");
  }

  //// Initialize the last update time
  //last_update_time_ = rtt_rosclock::rtt_now();

  /*if(!this->waitForMode(barrett::SafetyModule::ACTIVE)) {*/
  /*RTT::log(RTT::Error) << "Could not start Barrett Hardware, the safety "*/
  /*"module took too long to switch to the ACTIVE mode." << RTT::endlog();*/
  /*return false;*/
  /*}*/

  // Initialize the hand
  if(hand_device_) {
    // NOTE: The barrett hand will deadlock on initialization if the robot is IDLE
  }

  // Read the state estimation
  try {
    if(wam_device_) {
      RTT::log(RTT::Debug) << "Initializing WAM..." << RTT::endlog();
      // Zero the commands before the arm is activated
      wam_device_->setZero();
      // Write to the configuration ports
      wam_device_->readConfig();
      wam_device_->readDevice(last_update_time_,this->getPeriod());
    }
  } catch(std::runtime_error &err) {
    RTT::log(RTT::Error) << "Could not start WAM: " << err.what() << RTT::endlog();
    return false;
  }

  return true;
}


void BarrettHWManager::updateHook()
{
  // This update hook is synchronized with the device thread, which uses
  // blocking IO. Note that the devide thread does not block and wait for
  // this thread to provide it with a new command. If one isn't ready, then
  // it does not send a command.
  
  {
    // Read the command from the input ports
    if(wam_device_) { wam_device_->readPorts(); }
    if(hand_device_) { hand_device_->readPorts(); }

    // Signal that a new command is ready
    RTT::os::MutexLock lock(new_cmd_mutex_);
    new_cmd_ = true;
  }
  
  {
    // Wait for new state from the device thread
    RTT::os::MutexLock lock(new_state_mutex_);
    if(!new_state_) {
      new_state_cond_.wait(new_state_mutex_);
    }
    new_state_ = false;

    // Write the state to the output ports
    if(wam_device_) { wam_device_->writePorts(); }
    if(hand_device_) { hand_device_->writePorts(); }
  }
}

void BarrettHWManager::stopHook()
{
  device_thread_.stop();
}

void BarrettHWManager::deviceStopHook()
{
  // Set the mode to IDLE
  // FIXME: This doesn't work, it just leaves the thing running!!
  // it's as if setting the mode to "IDLE" makes it so the safety module doesn't check heartbeats any more
  // maybe instead we need to set all of the motor pucks to idle?
  //this->setMode(barrett::SafetyModule::IDLE);
  // Wait for the system to become active
  if(!this->waitForMode(barrett::SafetyModule::IDLE)) {
    RTT::log(RTT::Warning) << "Could not IDLE the Barrett Hardware!" <<
      RTT::endlog();
  }
  if(hand_device_) {
    hand_device_->idle();
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

bool BarrettHWManager::configureWam4(const std::string &urdf_prefix)
{
  // Make sure we 're in the configured state, and not running
  if(!this->isConfigured() || this->isRunning()) {
    RTT::log(RTT::Error) << "Cannot configure WAM while the component is "
      "unconfigured or running." << RTT::endlog();
    return false;
  }

  return configureWam4Protected(urdf_prefix);
}

bool BarrettHWManager::configureWam4Protected(const std::string &urdf_prefix)
{
  // Check for a 4-DOF WAM
  if(!barrett_manager_->foundWam4()) {
    RTT::log(RTT::Error) << "Could not find a requested 4-DOF WAM on bus" <<
      bus_id_ << "." << RTT::endlog();
    return false;
  }

  // Create a new 7-DOF WAM
  if(!this->configureWam<4>(urdf_prefix)) {
    return false;
  }

  return true;
}

bool BarrettHWManager::configureWam7(const std::string &urdf_prefix)
{
  // Make sure we 're in the configured state, and not running
  if(!this->isConfigured() || this->isRunning()) {
    RTT::log(RTT::Error) << "Cannot configure WAM while the component is "
      "unconfigured or running." << RTT::endlog();
    return false;
  }

  return this->configureWam7Protected(urdf_prefix);
}

bool BarrettHWManager::configureWam7Protected(const std::string &urdf_prefix)
{
  // Check for a 7-DOF WAM
  if(!barrett_manager_->foundWam7()) {
    RTT::log(RTT::Error) << "Could not find a requested 7-DOF WAM on bus" <<
      bus_id_ << "." << RTT::endlog();
    return false;
  }

  // Create a new 7-DOF WAM
  if(!this->configureWam<7>(urdf_prefix)) {
    this->provides()->removeService("wam");
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
      RTT::log(RTT::Info) << "Configuring " <<DOF<<"-DOF WAM from configuration file: \""<<((config_path_.length() > 0) ? config_path_ : std::string("default.cfg") )<<"\"" << RTT::endlog();
      wam_device_.reset(
          new WamHWDevice<DOF>(
            this->provides(),
            urdf_model_,
            urdf_prefix,
            barrett_manager_,
            barrett_manager_->getConfig().lookup(barrett_manager_->getWamDefaultConfigPath())));
    }
  } catch(std::runtime_error &ex) {
    RTT::log(RTT::Error) << "Could not configure " << DOF << "-DOF WAM from configuration file : \""<<((config_path_.length() > 0) ? config_path_ : std::string("default.cfg") )<<"\"" <<
      ex.what() << RTT::endlog();
    wam_device_.reset();
    return false;
  }

  return true;
}


bool BarrettHWManager::configureHand(const std::string &urdf_prefix)
{
  using namespace oro_barrett_interface;

  try{
    // Construct a new bhand device and "hand" service (interface and state storage)
    hand_device_.reset();
    hand_device_.reset(
        new HandHWDevice(
          this->provides(),
          urdf_model_,
          urdf_prefix,
          barrett_manager_));
  } catch(std::runtime_error &ex) {
    RTT::log(RTT::Error) << "Could not configure Barrett Hand: " <<
      ex.what() << RTT::endlog();
    return false;
  }
  RTT::log(RTT::Info) << "Configured Barrett Hand." << RTT::endlog();

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

BarrettHWManager::BarrettDeviceThread::BarrettDeviceThread(BarrettHWManager *owner)
  : RTT::os::Thread(
      ORO_SCHED_RT, 
      RTT::os::HighestPriority, 
      0.001, // Run as fast as possible (1KHz)
      1 << 4, // Use bus id for cpu affinity
      owner->getName()+"-device-thread"),
    owner_(owner),
    break_loop_sem_(0),
    done_sem_(0)
{
}

bool BarrettHWManager::BarrettDeviceThread::initialize()
{ 
  return owner_->deviceStartHook();
}

void BarrettHWManager::deviceUpdateHook()
{
  ros::Time time = rtt_rosclock::rtt_now();
  RTT::Seconds period = (time - last_update_time_).toSec();
  period_ = period;

  // Read
  {
    RTT::os::MutexLock lock(new_state_mutex_);

    RTT::os::TimeService::ticks read_start = RTT::os::TimeService::Instance()->getTicks();
    if(wam_device_) {
      try {
        // Read the state estimation
        wam_device_->readDevice(time,period);
        // Flush the buffers if the safety mode switched to idle
        if(wam_device_->getSafetyMode() == barrett::SafetyModule::IDLE 
           && wam_device_->getSafetyMode() != safety_mode_) 
        {
          bus_manager_->flushBuffers();
        }
        // Store the new safety mode
        safety_mode_ = (barrett::SafetyModule::SafetyMode)wam_device_->getSafetyMode();
      } catch(std::runtime_error &err) {
        RTT::log(RTT::Error) << "Could not read the WAM state: " << err.what() << RTT::endlog();
        this->error();
      }
    }
    if(hand_device_) {
      try {
        // Read the state estimation
        hand_device_->readDevice(time,period);
      } catch(std::runtime_error &err) {
        RTT::log(RTT::Error) << "Could not read the BHand state: " << err.what() << RTT::endlog();
        this->error();
      }
    }
    read_duration_ = RTT::os::TimeService::Instance()->secondsSince(read_start);

    // Signal the new state
    new_state_ = true;
    new_state_cond_.broadcast();
  }

  // Write
  {
    RTT::os::MutexLock lock(new_cmd_mutex_);
    if(new_cmd_) {
      RTT::os::TimeService::ticks write_start = RTT::os::TimeService::Instance()->getTicks();
      if(wam_device_) {
        try {
          // Write the control command (force the write if the system is idle)
          wam_device_->writeDevice(time,period);
        } catch(std::runtime_error &err) {
          RTT::log(RTT::Error) << "Could not write the WAM command: " << err.what() << RTT::endlog();
          this->error();
        }
      }
      if(hand_device_) {
        try {
          // Write the control command (force the write if the system is idle)
          hand_device_->writeDevice(time,period);
        } catch(std::runtime_error &err) {
          RTT::log(RTT::Error) << "Could not write the BHand command: " << err.what() << RTT::endlog();
          this->error();
        }
      }
      write_duration_ = RTT::os::TimeService::Instance()->secondsSince(write_start);

      new_cmd_ = false;
    } 
  }

  last_update_time_ = time;
}

void BarrettHWManager::BarrettDeviceThread::loop()
{
}

void BarrettHWManager::BarrettDeviceThread::step()
{
  if(owner_->isRunning()) {
    owner_->deviceUpdateHook();
  }
}

bool BarrettHWManager::BarrettDeviceThread::breakLoop()
{
  return true;
}

void BarrettHWManager::BarrettDeviceThread::finalize()
{ 
  owner_->deviceStopHook();
}

ORO_LIST_COMPONENT_TYPE(oro_barrett_hw::BarrettHWManager);
ORO_CREATE_COMPONENT_LIBRARY()

