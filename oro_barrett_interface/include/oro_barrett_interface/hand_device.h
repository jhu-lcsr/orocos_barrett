#ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
#define __ORO_BARRETT_INTERFACE_HAND_DEVICE_H

#include <boost/assign/std/vector.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <sensor_msgs/JointState.h>

#include <rtt_ros_tools/throttles.h>
#include <rtt_roscomm/rtt_rostopic.h>

#include <oro_barrett_msgs/BHandCmd.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

#include <geometry_msgs/PoseStamped.h>

namespace oro_barrett_interface {

  void getSubtree(
      const KDL::Tree &tree,
      KDL::SegmentMap::const_iterator subroot,
      KDL::Tree &subtree)
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

      RTT::log(RTT::Debug) 
        << "Adding segment " <<child_name
        << " to parent "<< element_name 
        << " with transform: " 
        << RTT::endlog();

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
    virtual void readDevice(ros::Time time, RTT::Seconds period) = 0;
    //! Write the command to the hardware
    virtual void writeDevice(ros::Time time, RTT::Seconds period) = 0;

    //! Read the simulation state
    virtual void readSim(ros::Time time, RTT::Seconds period) { }
    //! Write the simulation command
    virtual void writeSim(ros::Time time, RTT::Seconds period) { }
    
    //! Read the input ports
    //TODO: make pure virtual
    virtual void readPorts() {};
    //! Write the output ports
    //TODO: make pure virtual
    virtual void writePorts() {};

    HandDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix);

    //! Removes the added "wam" service 
    virtual ~HandDevice();

    virtual void open() = 0;
    virtual void close() = 0;

    //! Number of pucks in the hand
    static const unsigned int N_PUCKS = 4;
    static const unsigned int DOF = 8;

    //! Hand run mode
    enum RunMode {
      IDLE = 0,
      INITIALIZE,
      RUN
    };

    //! Initialization state
    enum InitState {
      INIT_FINGERS = 0,
      SEEK_FINGERS,
      INIT_SPREAD,
      SEEK_SPREAD,
      INIT_CLOSE
    };

  protected:

    //! RTT Service for BHand interfaces
    RTT::Service::shared_ptr parent_service_;

    //! The current run mode
    RunMode run_mode;

    //! The current initialization state
    InitState init_state;

    //! \name Configuration & State
    //\{
    KDL::Tree full_tree;
    //! The names of all the joints
    std::vector<std::string> 
      joint_names;
    // True corresponds to an active joint, False corresponds to a passive one
    std::vector<bool>
      joint_actuation;

    std::string urdf_prefix;
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

    //! Mode bitmasks
    unsigned int 
      mode_torque,
      mode_position,
      mode_velocity,
      mode_trapezoidal;

    //! Flag desingating any modes have changed
    bool modes_changed;

    std::map<std::string, double> q_map;
    geometry_msgs::PoseStamped com_msg;

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
    RTT::OutputPort<geometry_msgs::PoseStamped>
      center_of_mass_debug_out;
    //\}

    rtt_ros_tools::PeriodicThrottle joint_state_throttle;

    //! Compute the hand center of mass
    void computeCenterOfMass(Eigen::VectorXd &xyzm);
    void computeCenterOfMassDebug();
    void computeCenterOfMass(Eigen::VectorXd &xyzm, bool debug);
  };

  HandDevice::HandDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix_) :
    parent_service_(parent_service),

    run_mode(IDLE),

    joint_actuation(8),

    urdf_prefix(urdf_prefix_),

    joint_position(8),
    joint_velocity(8),
    joint_torque_cmd(4),
    joint_position_cmd(4),
    joint_velocity_cmd(4),
    joint_trapezoidal_cmd(4),
    center_of_mass(4),

    mode_torque(0x0),
    mode_position(0x0),
    mode_velocity(0x0),
    mode_trapezoidal(0x0),
    modes_changed(false),

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
      .doc("Initialize the hand by driving the fingers open. NOTE: This command must be performed BEFORE activatiting the WAM for the first time. It cannot be performed while the WAM is activated.");
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

    hand_service->addOperation("computeCenterOfMass", (void (HandDevice::*)(Eigen::VectorXd&))&HandDevice::computeCenterOfMass, this, RTT::OwnThread);
    hand_service->addOperation("computeCenterOfMassDebug", &HandDevice::computeCenterOfMassDebug, this, RTT::OwnThread);
    hand_service->addProperty("center_of_mass",center_of_mass)
      .doc("Center of mass as (px,py,pz,m) of the hand in the parent of the root frame of the hand (bhand_palm_link).");

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
    hand_service->addPort("center_of_mass_debug_out",center_of_mass_debug_out)
      .doc("Center of mass pose.");

    // Add the port and stream it to a ROS topic
    std::string owner_name = parent_service->getOwner()->getName();
    joint_state_out.createStream(rtt_roscomm::topic("~"+owner_name+"/hand/joint_states"));
    center_of_mass_debug_out.createStream(rtt_roscomm::topic("~"+owner_name+"/hand/center_of_mass"));
    joint_cmd_in.createStream(rtt_roscomm::topic("~"+owner_name+"/hand/cmd"));

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
    if (!kdl_parser::treeFromUrdfModel(urdf_model, full_tree)){
      std::ostringstream oss;
      oss << "Failed to construct kdl tree";
      RTT::log(RTT::Error) << oss.str() <<RTT::endlog();
      throw std::runtime_error(oss.str());
    }

    // Get the root link of the bhand
    KDL::SegmentMap::const_iterator bhand_parent_link = full_tree.getSegment(urdf_prefix+"/bhand_palm_link")->second.parent;
    // Get the transform from the parent of the root link to the root link
    base_to_parent_transform = full_tree.getSegment(urdf_prefix+"/bhand_palm_link")->second.segment.pose(0); 
    // Create a KDL tree with the same root name as the actual hand
    kdl_tree = KDL::Tree(bhand_parent_link->first);
    // Get the bhand subtree
    getSubtree(full_tree, bhand_parent_link, kdl_tree);

    com_msg.header.frame_id = bhand_parent_link->first;
    
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
      KDL::SegmentMap::const_iterator root,
      const std::map<std::string, double> &q_map,
      KDL::RigidBodyInertia &total_inertia,
      bool debug) 
  {
    // Get the segment
    const KDL::TreeElement &tree_elem = root->second;
    const KDL::Segment &segment = tree_elem.segment;
    const std::string &joint_name = segment.getJoint().getName();
    
    // Update frame
    frame = frame * segment.pose(q_map.find(joint_name)->second); 

    // Accumulate inertia
    const KDL::RigidBodyInertia &link_inertia = frame*segment.getInertia();
    total_inertia = total_inertia + link_inertia;

    if(debug) {
      RTT::log(RTT::Debug) << "Adding mass from segment \"" << segment.getName() << "\" (" << link_inertia.getCOG() << ", " << link_inertia.getMass() << ")"<< RTT::endlog();
    }

    // Recurse on each child
    std::vector<KDL::SegmentMap::const_iterator>::const_iterator it;
    for(it = tree_elem.children.begin();
        it != tree_elem.children.end();
        ++it)
    {
      // it is of type std::pair<std::string, KDL::TreeElement>**
      computeCenterOfMassOfSubtree(
          frame,
          *it,
          q_map,
          total_inertia,
          debug);
    }
  }

  void HandDevice::computeCenterOfMass(Eigen::VectorXd &xyzm)
  {
    this->computeCenterOfMass(xyzm, false);
  }

  void HandDevice::computeCenterOfMassDebug()
  {
    Eigen::VectorXd xyzm;
    this->computeCenterOfMass(xyzm, true);
    RTT::log(RTT::Debug) << "Mass (x,y,z,m): \n" <<xyzm << RTT::endlog();
  }

  void HandDevice::computeCenterOfMass(Eigen::VectorXd &xyzm, bool debug)
  {
    // Make sure xyzm is the right size
    xyzm.resize(4);

    // Update joint name->position map;
    for(int i=0; i < 8; i++) {
      this->q_map[joint_names[i]] = joint_position[i];
    }

    KDL::RigidBodyInertia total_inertia;

    KDL::SegmentMap::const_iterator root_element = kdl_tree.getRootSegment();
    
    // Recurse through the tree
    computeCenterOfMassOfSubtree(
        KDL::Frame::Identity(), 
        root_element, 
        q_map,
        total_inertia,
        debug);

    // Transform the location into the parent link
    //total_inertia = base_to_parent_transform * total_inertia;

    // Store the xyz,m
    xyzm[0] = total_inertia.getCOG().x();
    xyzm[1] = total_inertia.getCOG().y();
    xyzm[2] = total_inertia.getCOG().z();
    xyzm[3] = total_inertia.getMass();
  }
}

#endif // ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
