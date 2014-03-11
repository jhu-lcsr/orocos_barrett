#ifndef __ORO_BARRETT_HW_BARRETT_HW_MANAGER
#define __ORO_BARRETT_HW_BARRETT_HW_MANAGER

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt/os/Semaphore.hpp>

#include <urdf/model.h>

#include <oro_barrett_interface/barrett_manager.h>

#include <oro_barrett_sim/wam_sim_device.h>

#define ORO_BARRETT_BHAND 1

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

    ros::Time last_gz_update_time_;
    RTT::Seconds gz_period_;

    ros::Time last_update_time_;
    RTT::Seconds period_;
    RTT::Seconds read_duration_;
    RTT::Seconds write_duration_;

    //barrett::SafetyModule::SafetyMode safety_mode_;

    //! The Gazebo Model
    gazebo::physics::ModelPtr gazebo_model_;
  };


}

#endif // ifndef __ORO_BARRETT_HW_BARRETT_HW
