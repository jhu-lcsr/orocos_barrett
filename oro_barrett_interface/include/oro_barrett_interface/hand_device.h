#ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
#define __ORO_BARRETT_INTERFACE_HAND_DEVICE_H

#include <boost/assign/std/vector.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <sensor_msgs/JointState.h>

#include <rtt_ros_tools/throttles.h>

#include <oro_barrett_msgs/BHandCmd.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

namespace oro_barrett_interface {

  void getSubtree(const KDL::Tree &tree, KDL::SegmentMap::const_iterator subroot, KDL::Tree &subtree)
  {
    const std::string element_name = subroot->first;
    const KDL::TreeElement element = subroot->second;

    // Add the children segments to the subtree
    std::vector<KDL::SegmentMap::const_iterator>::const_iterator it;
    for(it = element.children.begin();
        it != element.children.end();
        ++it)
    {
      const std::string child_name = (*it)->first;
      const KDL::TreeElement child = (*it)->second;

      RTT::log(RTT::Debug) << "Adding segment " <<child_name<< " to parent "<< element_name <<RTT::endlog();

      // Add this segment to the subtree
      subtree.addSegment(child.segment, element_name); 
      getSubtree(tree, *it, subtree);
    }
  }


  class HandDevice {
  public:
    //! Initialize the hand (get pose and hold open)
    virtual void initialize() = 0;
    //! Idle the hand (open it up and disable the motors)
    virtual void idle() = 0;
    //! Run the hand (process commands)
    virtual void run() = 0;
    //! Set finger compliance
    virtual void setCompliance(bool enable) = 0;

    //! Set joint to torque mode
    virtual void setTorqueMode(unsigned int joint_index) = 0;
    //! Set joint to position mode
    virtual void setPositionMode(unsigned int joint_index) = 0;
    //! Set joint to velocity mode
    virtual void setVelocityMode(unsigned int joint_index) = 0;
    //! Set joint to trapezoidal mode
    virtual void setTrapezoidalMode(unsigned int joint_index) = 0;

    //! Read the hardware state and publish it
    virtual void readHW(RTT::Seconds time, RTT::Seconds period) = 0;
    //! Write the command to the hardware
    virtual void writeHW(RTT::Seconds time, RTT::Seconds period) = 0;

    HandDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix);

    //! Removes the added "wam" service 
    virtual ~HandDevice();

    virtual void open() = 0;
    virtual void close() = 0;

  protected:
    //! RTT Service for BHand interfaces
    RTT::Service::shared_ptr parent_service_;

    //! \name Configuration & State
    //\{
    //! The names of all the joints
    std::vector<std::string> 
      joint_names;
    // True corresponds to an active joint, False corresponds to a passive one
    std::vector<bool>
      joint_actuation;

    KDL::Tree kdl_tree;
    KDL::Frame base_to_parent_transform;

    // Joint state for active and passive joints
    Eigen::VectorXd
      joint_position,
      joint_velocity,
      joint_torque_cmd,
      joint_position_cmd,
      joint_velocity_cmd,
      joint_trapezoidal_cmd,
      center_of_mass;

    Eigen::VectorXi
      knuckle_torque;

    sensor_msgs::JointState
      joint_state;
    oro_barrett_msgs::BHandCmd
      joint_cmd;
    //\}

    //! \name Input ports
    //\{
    RTT::InputPort<Eigen::VectorXd >
      joint_torque_in,
      joint_position_in,
      joint_velocity_in,
      joint_trapezoidal_in;

    RTT::InputPort<oro_barrett_msgs::BHandCmd>
      joint_cmd_in;
    //\}
    
    //! \name Output ports
    //\{
    RTT::OutputPort<Eigen::VectorXd >
      joint_torque_out,
      joint_position_out,
      joint_velocity_out,
      center_of_mass_out;
    RTT::OutputPort<sensor_msgs::JointState >
      joint_state_out;
    //\}

    rtt_ros_tools::PeriodicThrottle joint_state_throttle;

    //! Compute the hand center of mass
    void computeCenterOfMass(Eigen::VectorXd &xyzm);
  };

  HandDevice::HandDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix) :
    parent_service_(parent_service),

    joint_actuation(8),

    joint_position(8),
    joint_velocity(8),
    joint_torque_cmd(4),
    joint_position_cmd(4),
    joint_velocity_cmd(4),
    joint_trapezoidal_cmd(4),
    center_of_mass(4),

    knuckle_torque(4),

    // Throttles
    joint_state_throttle(0.01)
  {
    RTT::Service::shared_ptr hand_service = parent_service->provides("hand");
    hand_service->doc("Barrett Hand interface.");

    // Operations
    hand_service->addOperation("idle", &HandDevice::idle, this, RTT::OwnThread)
      .doc("Disable the hand motors, but continue reading the state.");
    hand_service->addOperation("initialize", &HandDevice::initialize, this, RTT::OwnThread)
      .doc("Initialize the hand by driving the fingers open. NOTE: This command must be performed before ACTIVATITing the WAM for the first time. It cannot be performed while the WAM is activated.");
    hand_service->addOperation("run", &HandDevice::run, this, RTT::OwnThread)
      .doc("Enable sending commands to the hand motors.");
    hand_service->addOperation("open", &HandDevice::open, this, RTT::OwnThread)
      .doc("Open the fingers.");
    hand_service->addOperation("close", &HandDevice::close, this, RTT::OwnThread)
      .doc("Close the fingers.");
    hand_service->addOperation("setCompliance", &HandDevice::setCompliance, this, RTT::OwnThread);

    hand_service->addOperation("setTorqueMode", &HandDevice::setTorqueMode, this, RTT::OwnThread)
      .doc("Set a puck to operate in torque mode.")
      .arg("index","Puck index (0,1,2) or spread (3)");
    hand_service->addOperation("setPositionMode", &HandDevice::setPositionMode, this, RTT::OwnThread)
      .doc("Set a puck to operate in instantaneous position (PID) mode.")
      .arg("index","Puck index (0,1,2) or spread (3)");
    hand_service->addOperation("setVelocityMode", &HandDevice::setVelocityMode, this, RTT::OwnThread)
      .doc("Set a puck to operate in instantaneous velocity mode.")
      .arg("index","Puck index (0,1,2) or spread (3)");
    hand_service->addOperation("setTrapezoidalMode", &HandDevice::setTrapezoidalMode, this, RTT::OwnThread)
      .doc("Set a puck to operate in trapezoidal position mode.")
      .arg("index","Puck index (0,1,2) or spread (3)");

    //hand_service->addConstant("F1",0);
    //hand_service->addConstant("F2",1);
    //hand_service->addConstant("F3",2);
    //hand_service->addConstant("SPREAD",3);

    hand_service->addOperation("computeCenterOfMass", &HandDevice::computeCenterOfMass, this, RTT::OwnThread);
    hand_service->addProperty("center_of_mass",center_of_mass)
      .doc("Center of mass as (px,py,pz,m) of the hand in the parent of root frame of the hand (bhand_palm_link).");

    // Orocos ports
    hand_service->addPort("joint_torque_in",joint_torque_in)
      .doc("4-DOF command masked by pucks in joint torque mode.");
    hand_service->addPort("joint_position_in",joint_position_in)
      .doc("4-DOF command masked by pucks in instantaneous joint position mode.");
    hand_service->addPort("joint_velocity_in",joint_velocity_in)
      .doc("4-DOF command masked by pucks in instantaneous joint velocity mode.");
    hand_service->addPort("joint_trapezoidal_in",joint_trapezoidal_in)
      .doc("4-DOF command masked by pucks in trapezoidal joint position mode.");

    hand_service->addPort("center_of_mass_out",center_of_mass_out)
      .doc("Center of mass as (px,py,pz,m) of the hand in the parent of root frame of the hand (bhand_palm_link).");

    // ROS data ports
    hand_service->addPort("joint_cmd_in", joint_cmd_in)
      .doc("4-DOF command and command modes. This command overrides the four mode input ports.");
    hand_service->addPort("joint_state_out", joint_state_out)
      .doc("The full (actuated and underactuated) 8-DOF joint state.");

    using namespace boost::assign;
    joint_names.clear();
    joint_names += 
      urdf_prefix+"/finger_1/prox_joint", // F1 spread
      urdf_prefix+"/finger_2/prox_joint", // F2 spread
      urdf_prefix+"/finger_1/med_joint", 
      urdf_prefix+"/finger_2/med_joint",
      urdf_prefix+"/finger_3/med_joint",
      urdf_prefix+"/finger_1/dist_joint",
      urdf_prefix+"/finger_2/dist_joint",
      urdf_prefix+"/finger_3/dist_joint";

    // Get all the required joints
    for(std::vector<std::string>::iterator joint_name_it = joint_names.begin();
        joint_name_it != joint_names.end();
        ++joint_name_it)
    {
      boost::shared_ptr<const urdf::Joint> joint = urdf_model.getJoint(*joint_name_it);

      if(!joint) {
        std::ostringstream oss;
        oss << "Could not get needed BHand joint: \"" << *joint_name_it << "\"";
        RTT::log(RTT::Error) << oss.str() << RTT::endlog();
        throw std::runtime_error(oss.str());
      }
    }

    // Extract the KDL tree from the URDF
    KDL::Tree full_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, full_tree)){
      std::ostringstream oss;
      oss << "Failed to construct kdl tree";
      RTT::log(RTT::Error) << oss.str() <<RTT::endlog();
      throw std::runtime_error(oss.str());
    }

    // Get the root link of the bhand
    KDL::SegmentMap::const_iterator bhand_palm_link = full_tree.getSegment(urdf_prefix+"/bhand_palm_link");
    // Get the transform from the parent of the root link to the root link
    base_to_parent_transform = bhand_palm_link->second.segment.getFrameToTip(); 
    // Create a KDL tree with the same root name as the actual hand
    kdl_tree = KDL::Tree(urdf_prefix+"/bhand_palm_link");
    // Get the bhand subtree
    getSubtree(full_tree, bhand_palm_link, kdl_tree);
    
    // Resize joint state
    joint_state.name.resize(8);
    joint_state.position.resize(8);
    joint_state.velocity.resize(8);
    joint_state.effort.resize(8);
  }

  HandDevice::~HandDevice()
  {
    parent_service_->removeService("hand");
  }

  void computeCenterOfMassOfSubtree(
      KDL::Frame frame,
      const KDL::TreeElement &tree_elem,
      const std::map<std::string, double> &q_map,
      KDL::Vector &total_xyz,
      double &total_m) 
  {
    // Get the segment
    const KDL::Segment &segment = tree_elem.segment;
    //RTT::log(RTT::Debug) << "Adding mass from segment \"" << segment.getName() << "\""<< RTT::endlog();

    // Update frame
    const std::string &joint_name = segment.getJoint().getName();

    frame = frame * segment.pose(q_map.find(joint_name)->second); 
    //RTT::log(RTT::Debug) << "Segment pose is: " << std::endl << frame << RTT::endlog();

    // Get link center of gravity
    KDL::Vector link_xyz = segment.getInertia().getCOG();
    // Transform the center of gravity in this link to the base link
    link_xyz = frame*link_xyz;
    
    // Get link mass
    const double link_m = segment.getInertia().getMass();

    // Update the center of mass of this link
    total_xyz = (total_m*total_xyz + link_m*link_xyz) / (total_m + link_m);
    // Update the total mass with the mass of this link
    total_m += link_m;

    // Recurse on each child
    std::vector<KDL::SegmentMap::const_iterator>::const_iterator it;
    for(it = tree_elem.children.begin();
        it != tree_elem.children.end();
        ++it)
    {
      // it is of type std::pair<std::string, KDL::TreeElement>**
      computeCenterOfMassOfSubtree(
          frame,
          (*it)->second,
          q_map,
          total_xyz,
          total_m);
    }
  }

  void HandDevice::computeCenterOfMass(Eigen::VectorXd &xyzm)
  {
    // Make sure xyzm is the right size
    xyzm.resize(4);

    // Construct joint name->position map;
    std::map<std::string, double> q_map;
    for(int i=0; i < 8; i++) {
      q_map[joint_names[i]] = joint_position[i];
    }

    KDL::Vector total_xyz;
    double total_mass;

    KDL::TreeElement root_element = kdl_tree.getRootSegment()->second;
    
    // Recurse through the tree
    computeCenterOfMassOfSubtree(
        KDL::Frame::Identity(), 
        root_element, 
        q_map,
        total_xyz, 
        total_mass);

    // Transform the location into the parent link
    total_xyz = base_to_parent_transform * total_xyz;

    // Store the xyz,m
    xyzm[0] = total_xyz.x();
    xyzm[1] = total_xyz.y();
    xyzm[2] = total_xyz.z();
    xyzm[3] = total_mass;
  }
}

#endif // ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
