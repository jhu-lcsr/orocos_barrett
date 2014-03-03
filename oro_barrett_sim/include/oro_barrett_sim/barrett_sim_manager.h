#ifndef __ORO_BARRETT_HW_BARRETT_HW_MANAGER
#define __ORO_BARRETT_HW_BARRETT_HW_MANAGER

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt/os/Semaphore.hpp>

#include <urdf/model.h>

#include <oro_barrett_interface/barrett_manager.h>

#include <oro_barrett_sim/wam_sim_device.h>

#ifdef ORO_BARRETT_BHAND
#include <oro_barrett_sim/hand_sim_device.h>
#endif

namespace oro_barrett_sim {

  class BarrettSimManager : public oro_barrett_interface::BarrettManager 
  {
  public:

    BarrettSimManager(const std::string &name);

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
     */
    virtual bool configureHand(const std::string &urdf_prefix);

    /** \brief Construct a WAM robot interface
     *
     */
    virtual bool configureWam4(const std::string &urdf_prefix);

    /** \brief Construct a WAM robot interface
     *
     */
    virtual bool configureWam7(const std::string &urdf_prefix);

    //! Get the real update period
    double getRealPeriod();


    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);

    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  private:

    /** \brief Configure a WAM robot on this bus
     *
     * This creates RTT services associated with the input and output ports
     * defined in the WamDevice struct.
     *
     */
    template <size_t DOF>
      bool configureWam(const std::string &urdf_prefix);

    //! \name libbarrett structures
    //\{
    int bus_id_;
    std::string config_path_;
    //boost::shared_ptr<barrett::bus::BusManager> bus_manager_;
    //boost::shared_ptr<barrett::ProductManager> barrett_manager_;
    //\}

    //! \name Possible barrett products
    //\{
    //! WAM robot container
    boost::shared_ptr<oro_barrett_interface::WamDeviceBase> wam_device_;
    //! Barrett hand container
#ifdef ORO_BARRETT_BHAND
    boost::shared_ptr<oro_barrett_interface::HandDevice> hand_device_;
#endif
    //\}

    RTT::Seconds last_update_time_;
    RTT::Seconds period_;
    RTT::Seconds read_duration_;
    RTT::Seconds write_duration_;

    //barrett::SafetyModule::SafetyMode safety_mode_;

    //! The Gazebo Model
    gazebo::physics::ModelPtr gazebo_model_;

    //! An RTT timer class for polling / waiting for a given mode
/*
 *    class BarrettModeTimer : public RTT::os::Timer 
 *    {
 *    public:
 *
 *      //! Returns True if the desired mode is now set
 *      static bool WaitForMode(
 *          boost::shared_ptr<barrett::ProductManager> barrett_manager,
 *          barrett::SafetyModule::SafetyMode mode,
 *          RTT::Seconds timeout, 
 *          RTT::Seconds poll_period) 
 *      {
 *        RTT::Seconds now = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());
 *        BarrettModeTimer timer(barrett_manager,mode);
 *        return 
 *          timer.startTimer(0,poll_period) 
 *          && timer.ready_sem_.waitUntil(now + timeout);
 *      }
 *
 *      //! Called when the timer semaphore times out once per polling cycle
 *      virtual void timeout(RTT::os::Timer::TimerId timer_id) {
 *        RTT::log(RTT::Debug) << "Checking safety module mode...." << RTT::endlog();
 *        try { 
 *          if( barrett_manager_ 
 *              && barrett_manager_->getSafetyModule() 
 *              && barrett_manager_->getSafetyModule()->getMode(true) == mode_) 
 *          {
 *            RTT::log(RTT::Debug) << "Requested mode enabled." << RTT::endlog();
 *            ready_sem_.signal();
 *          }
 *        } catch( std::runtime_error &err) {
 *          RTT::log(RTT::Error) << "Could not query safety module: " << err.what() << RTT::endlog();
 *          this->killTimer(0);
 *        }
 *      }
 *
 *    private:
 *
 *      //! Construct a timer and start polling the status mode
 *      BarrettModeTimer(
 *          boost::shared_ptr<barrett::ProductManager> barrett_manager,
 *          barrett::SafetyModule::SafetyMode mode) : 
 *        RTT::os::Timer(1,ORO_SCHED_RT), 
 *        barrett_manager_(barrett_manager), 
 *        ready_sem_(0),
 *        mode_(mode)
 *      {  }
 *
 *      boost::shared_ptr<barrett::ProductManager> barrett_manager_;
 *      RTT::os::Semaphore ready_sem_;
 *      barrett::SafetyModule::SafetyMode mode_;
 *    };
 */
  };


}

#endif // ifndef __ORO_BARRETT_HW_BARRETT_HW
