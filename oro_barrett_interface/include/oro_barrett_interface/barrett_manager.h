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
        const std::string &urdf_prefix) = 0;

    //! Configure a 4-DOF WAM
    virtual bool configureWam4(
        const std::string &urdf_prefix) = 0;

    //! Configure a Barrett Hand
    virtual bool configureHand(
        const std::string &urdf_prefix) = 0;

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
      .arg("urdf_prefix", "The joint name prefix in the URDF corresponding to the desired WAM robot.");

    this->addOperation("configureWam7",&BarrettManager::configureWam7, this)
      .doc("Configure a 7-DOF WAM Robot.")
      .arg("urdf_prefix", "The joint name prefix in the URDF corresponding to the desired WAM robot.");

    this->addOperation("configureHand",&BarrettManager::configureHand, this)
      .arg("urdf_prefix", "The joint name prefix in the URDF corresponding to the desired BHand.");
  }
}

#endif // ifndef __ORO_BARRETT_INTERFACE_BARRETT_MANAGER_H

