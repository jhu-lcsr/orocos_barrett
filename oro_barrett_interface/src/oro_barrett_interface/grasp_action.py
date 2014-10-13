
from collections import deque

import numpy

import rospy
import actionlib

from sensor_msgs.msg import JointState
from oro_barrett_msgs.msg import BHandGraspAction, BHandStatus, BHandCmd, BHandCmdMode


class GraspAction(object):
    # Activity states
    PREGRASP = 0
    GRASPING = 1
    HOLDING = 2
    ABORTING = 3
    PREEMPTING = 4

    def __init__(self, name='grasp', parent=None):
        # Properties
        self.name = name
        self.parent = parent

        # State
        self.state = None
        self.done_moving = None
        # Buffers for determining when the fingers have stopped
        self.stamp_history = None
        self.velocity_history = None


        # ROS parameters
        self.feedback_period = rospy.Duration(rospy.get_param('~feedback_period'))
        self.static_vel_threshold = rospy.get_param('~static_vel_threshold', 0.05)
        self.min_static_duration = rospy.Duration(rospy.get_param('~min_static_duration', 0.1))
        self.max_static_duration = 2.0*self.min_static_duration

        # ROS topics
        self.joint_states_sub = rospy.Subscriber(
            'hand/joint_states',
            JointState,
            self.joint_states_cb)
        self.status_pub = rospy.Subscriber(
            'hand/status',
            BHandStatus,
            self.state_cb)
        self.cmd_pub = rospy.Publisher(
            'hand/cmd',
            BHandCmd)

        # Create ROS action server
        self.server = actionlib.SimpleActionServer(
            self.name, BHandGraspAction,
            auto_start=True)

        self.server.register_goal_callback(self.goal_cb)
        self.server.register_preempt_callback(self.preempt_cb)

    def joint_states_cb(self, msg):
        """Determine when joints are done moving"""

        # Return if not active
        if not self.server.is_active():
            return

        # Add the latest joint state
        now = rospy.Time.now()

        if now < msg.header.stamp:
            self.server.set_aborted(text="Hand state timestamp is in the future.")
            return

        # Append to the history
        self.stamp_history.append(msg.header.stamp)
        for dof, (dof_hist, new_vel) in enumerate(zip(self.velocity_history, msg.velocity)):
            dof_hist.append(abs(new_vel))

        # Pop off old data
        while len(self.stamp_history) > 0 and (now - self.stamp_history[0]) > self.max_static_duration:
            self.stamp_history.popleft()
            for dof_hist in self.velocity_history:
                dof_hist.popleft()

        # Find the index of the relevant elements
        start_index = min([i for i, s in enumerate(self.stamp_history) if (now-s) < self.min_static_duration])

        # Compute the mean velocity over the last duration
        for dof, dof_hist in enumerate(self.velocity_history):
            # Mark the dof as done moving if it's below the static velocity threshold
            if numpy.mean(dof_hist[start_index:]) < self.static_vel_threshold:
                self.done_moving[dof] = True

    def status_cb(self, msg):
        """Interpret BHand status, send appropriate commands and update activity state"""

        # Return if not active
        if not self.server.is_active():
            return

        # Get the masked modes
        marked_modes = [m.value for i, m in enumerate(msg.mode) if i < 3 and self.active_goal.grasp_mask[i]]

        # Check the modes based on the activity states:
        if self.state == self.PREGRASP:
            # Check if all joints are in velocity mode
            if not all([m == BHandCmdMode.MODE_VELOCITY for m in marked_modes]):
                self.cmd_pub.publish(self.grasp_cmd)
            else:
                self.state = self.GRASPING

        elif self.state == self.GRASPING:
            # Check if all joints are in effort mode
            if not all([m == BHandCmdMode.MODE_TORQUE for m in marked_modes]):
                # Check if the hand is done moving, and change to effort command
                if all(self.done_moving):
                    self.cmd_pub.publish(self.hold_cmd)
            else:
                self.state = self.HOLDING
                self.server.set_succeeded()

        elif self.state == self.HOLDING:
            pass

        elif self.state in [self.ABORTING, self.PREEMPTING]:
            # Check if all joints are in effort mode
            if not all([m == BHandCmdMode.MODE_IDLE for m in marked_modes]):
                self.cmd_pub.publish(self.abort_cmd)
            else:
                self.state = None
                if self.state == self.ABORTING:
                    self.server.set_aborted()
                elif self.state == self.PREEMPTING:
                    self.server.set_preempted()

    def goal_cb(self):
        """Construct a new command and reset the activity state"""
        # Tell the parent to preempt peers
        if self.parent:
            self.parent.preempt_peers_of(self.name)

        # Accept the new goal
        self.active_goal = self.server.accept_new_goal()

        # Clear the grasp state
        self.state = self.PREGRASP
        self.stamp_history = deque()
        self.velocity_history = [deque()] * 8
        self.done_moving = [False] * 8

        # Construct hand commands for grasping and holding
        self.grasp_cmd = BHandCmd()
        self.hold_cmd = BHandCmd()
        self.abort_cmd = BHandCmd()
        for f_id, use_finger in enumerate(self.active_goal.grasp_mask):
            # Grasp command
            self.grasp_cmd.mode[f_id] = BHandCmd.MODE_VELOCITY if use_finger else BHandCmd.MODE_SAME
            self.grasp_cmd.cmd[f_id] = self.active_goal.grasp_speed[f_id] if use_finger else 0.0

            # Hold command
            self.hold_cmd.mode[f_id] = BHandCmd.MODE_TORQUE if use_finger else BHandCmd.MODE_SAME
            self.hold_cmd.cmd[f_id] = self.active_goal.grasp_effort[f_id] if use_finger else 0.0

            # Abort command
            self.abort_cmd.mode[f_id] = BHandCmd.MODE_IDLE if use_finger else BHandCmd.MODE_SAME
            self.abort_cmd.cmd[f_id] = self.active_goal.grasp_effort[f_id] if use_finger else 0.0

    def preempt_cb(self):
        """Idle the hand"""
        self.state = self.PREEMPTING

    def peer_preempt_cb(self):
        """Abort the goal without idling"""
        self.server.set_aborted(text="Peer command received")
