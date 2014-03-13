#ifndef __ORO_BARRETT_SIM_HAND_SIM_DEVICE_IMPL_H
#define __ORO_BARRETT_SIM_HAND_SIM_DEVICE_IMPL_H

namespace oro_barrett_sim {

  namespace {
    //! Get joint IDs from finger ID
    const bool fingerToJointIDs(const unsigned finger_id, unsigned &medial_id, unsigned &distal_id) 
    {
      if(finger_id < 3) {

        medial_id = finger_id + 2;
        distal_id = finger_id + 5;
      } else if(finger_id == 3) {
        medial_id = 0;
        distal_id = 1;
      } else {
        return false;
      }

      return true;
    }

    //! Get the sign of a scalar
    template <typename T> int sgn(T val) 
    {
      return (T(0) < val) - (val < T(0));
    }
  }

  HandSimDevice::HandSimDevice(
      RTT::Service::shared_ptr parent_service, 
      const urdf::Model &urdf_model,
      const std::string &urdf_prefix,
      std::vector<gazebo::physics::JointPtr> joints) :
    oro_barrett_interface::HandDevice(
        parent_service, 
        urdf_model, 
        urdf_prefix),
    gazebo_joints(joints),
    compliance_enabled(false),
    breakaway_torque(1.5),
    stop_torque(3),
    link_torque(4),
    fingertip_torque(4),
    breakaway_angle(4),
    joint_torque(8),
    joint_torque_max(8,1.5),
    joint_torque_breakaway(4),
    p_gain(5.0),
    d_gain(0.0),
    velocity_gain(0.1),
    trap_start_times(4),
    torque_switches(4,false)
  { 
    // Initialize velocity profiles
    for(unsigned i=0; i<N_PUCKS; i++) {
      trap_generators.push_back(KDL::VelocityProfile_Trap(1.0,0.1));
    }

    //using namespace gazebo::physics;

    //ODELink *prox_link_1 = joints[2]->GetParent();
    //ODELink *prox_link_2 = joints[3]->GetParent();

    parent_service->addProperty("stop_torque",stop_torque);
    parent_service->addProperty("breakaway_torque",breakaway_torque);
  }

  void HandSimDevice::initialize()
  {
    init_state = INIT_FINGERS;
    run_mode = INITIALIZE;
  }

  void HandSimDevice::idle()
  {
    run_mode = IDLE;
  }

  void HandSimDevice::run()
  {
    run_mode = RUN;
  }

  void HandSimDevice::setCompliance(bool enable)
  {
    compliance_enabled = enable;
  }

  bool HandSimDevice::withinTorqueLimits(const unsigned joint_id) {
    return std::abs(joint_torque[joint_id]) > joint_torque_max[joint_id];
  }

  void HandSimDevice::readSim(ros::Time time, RTT::Seconds period)
  {
    // Get state from ALL gazebo joints
    for(unsigned j=0; j < DOF; j++) {
      joint_position[j] = gazebo_joints[j]->GetAngle(0).Radian();
      joint_velocity[j] = 0.5*joint_velocity[j] + 0.5*gazebo_joints[j]->GetVelocity(0);
      joint_torque[j] = gazebo_joints[j]->GetForce(0u);
    }
  }


  void HandSimDevice::applyFingerTorque(const unsigned finger_id, const double torque) {
    static const double STOP_TORQUE = 5.0;
    static const double BREAKAWAY_TORQUE = 0.5;
    static const double FINGER_JOINT_RATIO = 0.0027 / 0.008;

    // Get the finger indices
    unsigned medial_id, distal_id;
    fingerToJointIDs(finger_id, medial_id, distal_id);

    // Get the torque on the medial link
    const double link_torque = gazebo_joints[medial_id]->GetForceTorque(0).body2Torque.z;
    const double fingertip_torque = gazebo_joints[distal_id]->GetForceTorque(0).body2Torque.z;

    // If the torque exeeds the breakway torque, allow the fingers to breakaway
    if(std::abs(link_torque) < BREAKAWAY_TORQUE) {
      gazebo_joints[medial_id]->SetForce(0,torque);
      gazebo_joints[distal_id]->SetForce(0,torque*FINGER_JOINT_RATIO);
    } else {
      gazebo_joints[medial_id]->SetForce(0,sgn(torque)*BREAKAWAY_TORQUE);
      gazebo_joints[distal_id]->SetForce(0,torque);
    }

    // If the torque exceeds the stop torque, stop the fingers, and hold position
    if(std::abs(link_torque) > STOP_TORQUE) {
      joint_cmd.mode[finger_id] = oro_barrett_msgs::BHandCmd::MODE_PID;
      joint_cmd.cmd[finger_id] = joint_position[medial_id];
    }
  }

  void HandSimDevice::writeSim(ros::Time time, RTT::Seconds period)
  {
    // Transmission ratio between two finger joints
    static const double FINGER_JOINT_RATIO = 45.0/140.0;

    for(unsigned i=0; i<4; i++) {
      // Joint torque
      double joint_torque = 0;

      // Get the finger indices
      unsigned mid, did;
      fingerToJointIDs(i, mid, did);

      double cmd = joint_cmd.cmd[i];
      double pos = joint_position[mid];
      double vel = joint_velocity[mid];

      // Switch the control law based on the command mode
      switch(joint_cmd.mode[i]) 
      {
        case oro_barrett_msgs::BHandCmd::MODE_IDLE:
          {
            joint_torque = 0.0;
            break;
          }
        case oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL:
          {
            const RTT::Seconds sample_secs = (rtt_rosclock::rtt_now() - trap_start_times[i]).toSec();
            joint_torque = 
              p_gain * (trap_generators[i].Pos(sample_secs) - pos) +
              d_gain * (trap_generators[i].Vel(sample_secs) - vel);
            break;
          }
        case oro_barrett_msgs::BHandCmd::MODE_PID:
          {
            joint_torque = p_gain * (cmd - pos);
            break;
          }
        case oro_barrett_msgs::BHandCmd::MODE_VELOCITY:
          {
            joint_torque = velocity_gain * (cmd - vel);
            break;
          }
        case oro_barrett_msgs::BHandCmd::MODE_TORQUE:
          {
            joint_torque = cmd;
            break;
          }
        default:
          {
            RTT::log(RTT::Error) << "Bad command mode: "<<joint_cmd.mode[i]<<RTT::endlog();
            return;
          }
      };

      if(i == 3) 
      {
        // Spread
        double spread_err = joint_position[0] - joint_position[1];
        double spread_derr = joint_velocity[0] - joint_velocity[1];
        double spread_constraint_force = spread_err + 0*spread_derr;
        gazebo_joints[0]->SetForce(0,spread_constraint_force + joint_torque);
        gazebo_joints[1]->SetForce(0,spread_constraint_force + joint_torque);
      }
      else
      {
        // Fingers

        const double FINGER_GAIN = 10.0;

        // Get link and fingertip torque
        link_torque[i] = gazebo_joints[mid]->GetForceTorque(0).body2Torque.z;
        fingertip_torque[i] = gazebo_joints[did]->GetForceTorque(0).body2Torque.z;

        //  Check for torque switch condition
        if(joint_velocity[mid] < 0.5) {
          if(link_torque[i] > breakaway_torque) {
            torque_switches[i] = true;
          } else if(link_torque[i] < -breakaway_torque) {
            torque_switches[i] = false;
          }
        }

        if(joint_torque > 0) {
          if(!torque_switches[i]) {
            gazebo_joints[mid]->SetForce(0,joint_torque);
            gazebo_joints[did]->SetForce(0,FINGER_GAIN*(FINGER_JOINT_RATIO*joint_position[mid] - joint_position[did]));
          } else {
            gazebo_joints[mid]->SetForce(0,breakaway_torque);
            gazebo_joints[did]->SetForce(0,FINGER_JOINT_RATIO*joint_torque);
            breakaway_angle[i] = joint_position[did];
          }
        } else {
          if(!torque_switches[i]) {
            gazebo_joints[mid]->SetForce(0,joint_torque);
            gazebo_joints[did]->SetForce(0,FINGER_GAIN*(FINGER_JOINT_RATIO*joint_position[mid] - joint_position[did]));
          } else {
            gazebo_joints[mid]->SetForce(0,joint_torque);
            gazebo_joints[did]->SetForce(0,breakaway_angle[i]);
          }
        }
      }
    }
  }


  void HandSimDevice::readDevice(ros::Time time, RTT::Seconds period)
  {
    // Always compute and write center of mass
    this->computeCenterOfMass(center_of_mass);
    center_of_mass_out.write(center_of_mass);

    // Get the state, and re-shape it
    joint_position_out.write(joint_position);

    // Publish state to ROS 
    if(this->joint_state_throttle.ready(0.02)) {
      // Update the joint state message
      this->joint_state.header.stamp = rtt_rosclock::host_rt_now();
      this->joint_state.name = this->joint_names;
      Eigen::Map<Eigen::VectorXd>(this->joint_state.position.data(),8) = this->joint_position;
      Eigen::Map<Eigen::VectorXd>(this->joint_state.velocity.data(),8) = this->joint_velocity;
      // TODO: Map knucle_torque into this
      //Eigen::Map<Eigen::VectorXd>(this->joint_state.effort.data(),8) = this->joint_torque;

      // Publish
      this->joint_state_out.write(this->joint_state);

      // Create a pose structure from the center of mass
      com_msg.header.stamp = rtt_rosclock::host_rt_now();
      com_msg.pose.position.x = center_of_mass[0];
      com_msg.pose.position.y = center_of_mass[1];
      com_msg.pose.position.z = center_of_mass[2];
      this->center_of_mass_debug_out.write(com_msg);
    }
  }

  void HandSimDevice::writeDevice(ros::Time time, RTT::Seconds period)
  {
    switch(run_mode) {
      case IDLE:
        // Don't command the hand
        break;
      case INITIALIZE:
        {
          switch(init_state) { 
            case INIT_FINGERS:
              init_state = SEEK_FINGERS;
              break;
            case SEEK_FINGERS:
              if(doneMoving(0) && doneMoving(1) && doneMoving(2)) {
                init_state = SEEK_SPREAD;
              }
              break;
            case SEEK_SPREAD:
              if(doneMoving(3)) {
                init_state = INIT_CLOSE;
              }
              break;
            case INIT_CLOSE:
              // Increase loop rate
              close();
              run_mode = RUN;
              break;
          };
          break;
        }
      case RUN:
        {
          // Read commands
          bool new_torque_cmd = (joint_torque_in.readNewest(joint_torque_cmd) == RTT::NewData);
          bool new_position_cmd = (joint_position_in.readNewest(joint_position_cmd) == RTT::NewData);
          bool new_velocity_cmd = (joint_velocity_in.readNewest(joint_velocity_cmd) == RTT::NewData);
          bool new_trapezoidal_cmd = (joint_trapezoidal_in.readNewest(joint_trapezoidal_cmd) == RTT::NewData);

          bool new_joint_cmd = (joint_cmd_in.readNewest(joint_cmd) == RTT::NewData);

          // Check sizes
          if(joint_torque_cmd.size() != N_PUCKS ||
             joint_position_cmd.size() != N_PUCKS ||
             joint_velocity_cmd.size() != N_PUCKS ||
             joint_trapezoidal_cmd.size() != N_PUCKS)
          {
            RTT::log(RTT::Error) << "Input command size msimatch!" << RTT::endlog();
            return;
          }

          // Parse the vectors into a ros command message
          for(int i=0; i < N_PUCKS; i++) {
            if(new_torque_cmd && joint_cmd.mode[i] == oro_barrett_msgs::BHandCmd::MODE_TORQUE) {
              joint_cmd.cmd[i] = joint_torque_cmd[i];
            } else if(new_position_cmd && joint_cmd.mode[i] == oro_barrett_msgs::BHandCmd::MODE_PID) {
              joint_cmd.cmd[i] = joint_position_cmd[i];
            } else if(new_velocity_cmd && joint_cmd.mode[i] == oro_barrett_msgs::BHandCmd::MODE_VELOCITY) {
              joint_cmd.cmd[i] = joint_velocity_cmd[i];
            } else if(new_trapezoidal_cmd && joint_cmd.mode[i] == oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL) {
              joint_cmd.cmd[i] = joint_trapezoidal_cmd[i];
              trap_generators[i].SetProfile(joint_position[i], joint_cmd.cmd[i]);
              trap_start_times[i] = rtt_rosclock::rtt_now();
            }
          }

        }
        break;
    };
  }


  void HandSimDevice::setTorqueMode(unsigned int joint_index) 
  {
    joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_TORQUE;
  }
  void HandSimDevice::setPositionMode(unsigned int joint_index) 
  {
    joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_PID;
  }
  void HandSimDevice::setVelocityMode(unsigned int joint_index) 
  {
    joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;
  }
  void HandSimDevice::setTrapezoidalMode(unsigned int joint_index) 
  {
    joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL;
  }

  bool HandSimDevice::doneMoving(const unsigned pair_index)
  {
    unsigned medial_id, distal_id;
    fingerToJointIDs(pair_index, medial_id, distal_id);

    return joint_velocity[medial_id] < 0.01 && joint_velocity[distal_id] < 0.01;
  }

  void HandSimDevice::open() 
  {
    // Open by setting velocity to open
    joint_cmd.mode[0] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;
    joint_cmd.mode[1] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;
    joint_cmd.mode[2] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;

    joint_cmd.cmd[0] = 1.0;
    joint_cmd.cmd[1] = 1.0;
    joint_cmd.cmd[2] = 1.0;
  }
  void HandSimDevice::close() 
  {
    // Close by setting negative velocity
    joint_cmd.mode[0] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;
    joint_cmd.mode[1] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;
    joint_cmd.mode[2] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;

    joint_cmd.cmd[0] = -1.0;
    joint_cmd.cmd[1] = -1.0;
    joint_cmd.cmd[2] = -1.0;
  }
}

#endif // ifnedf __ORO_BARRETT_SIM_HAND_DEVICE_IMPL_H
