#ifndef __ORO_BARRETT_INTERFACE_BARRETT_MANAGER_H
#define __ORO_BARRETT_INTERFACE_BARRETT_MANAGER_H

#include <rtt/TaskContext.hpp>

#include <urdf/model.h>


namespace oro_barrett_interface {

  class BarrettManager : public RTT::TaskContext {

  public:
    /** \brief Construct Barrett Manager
     *
     * This will create RTT properties and operations for populating this
     * manager with real or simulated hardware interfaces.
     */
    BarrettManager(const std::string &name);

    //! Configure a 7-DOF WAM
    virtual bool configureWam7(
        const std::string &tip_joint) = 0;

    //! Configure a 4-DOF WAM
    virtual bool configureWam4(
        const std::string &tip_joint) = 0;

    //! Configure a Barrett Hand
    virtual bool configureHand(
        const bool torque_sensors,
        const std::string &palm_link) = 0;

  protected:

    //! URDF with kinematics/dynamics information for all products on this bus
    urdf::Model urdf_model_;
    std::string urdf_str_;
  };

  BarrettManager::BarrettManager(const std::string &name) :
    RTT::TaskContext(name),
    urdf_str_("")
  {
    // Properties of of the manager
    this->addProperty("robot_description",urdf_str_)
      .doc("The URDF for all devices on barrett manager.");

    // Configuration operations
    this->addOperation("configureWam4",&BarrettManager::configureWam4, this)
      .doc("Configure a 4-DOF WAM Robot.")
      .arg("tip_joint", "The name of the tip joint in the URDF corresponding to the end of the robot.");

    this->addOperation("configureWam7",&BarrettManager::configureWam7, this)
      .doc("Configure a 7-DOF WAM Robot.")
      .arg("tip_joint", "The name of the tip joint in the URDF corresponding to the end of the robot.");

    this->addOperation("configureHand",&BarrettManager::configureHand, this)
      .arg("torque_sensors","Whether or not the hand as knuckle joint torque sensors [true,false]")
      .arg("palm_link", "The name of the link in the URDF corresponding to the base of the hand.");
  }
}

#endif // ifndef __ORO_BARRETT_INTERFACE_BARRETT_MANAGER_H

