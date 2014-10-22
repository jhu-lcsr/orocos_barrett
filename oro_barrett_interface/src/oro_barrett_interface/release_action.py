
from math import pi

import rospy
import actionlib

from sensor_msgs.msg import JointState
from oro_barrett_msgs.msg import BHandReleaseAction, BHandStatus, BHandCmd, BHandCmdMode


class ReleaseAction(object):
    """
    The Release Action aims to position the given fingers so that they form an
    opening.
    """

    # Activity states
    PRERELEASE = 0
    RELEASING = 1
    ABORTING = 2
    PREEMPTING = 3

    def __init__(self, name='release', parent=None):
        # Delegates
        self.server = None

        # Properties
        self.name = name
        self.parent = parent

        # State
        self.state = None
        self.done = None
        self.position = None
        self.active_goal = None

        # ROS parameters
        self.feedback_period = rospy.Duration(rospy.get_param('~feedback_period', 0.1))

        # ROS topics
        self.joint_states_sub = rospy.Subscriber(
            'hand/joint_states',
            JointState,
            self.joint_states_cb)
        self.status_pub = rospy.Subscriber(
            'hand/status',
            BHandStatus,
            self.status_cb)
        self.cmd_pub = rospy.Publisher(
            'hand/cmd',
            BHandCmd)

        # Create ROS action server
        self.server = actionlib.SimpleActionServer(
            self.name, BHandReleaseAction,
            auto_start=False)

        self.server.register_goal_callback(self.goal_cb)
        self.server.register_preempt_callback(self.preempt_cb)

        self.server.start()

    def joint_states_cb(self, msg):
        """Determine when joints are done moving"""

        # Return if not active
        if self.server and not self.server.is_active():
            return

        # Store finger position
        for f_id in range(3):
            self.position[f_id] = msg.position[f_id+2]

        if self.active_goal:

            def is_convex(j1, j2):
                c = j1 < pi / 2.0 and (j2 + j1) < pi / 4.0
                #if not c:
                #print('not convex: %g,  %g' % (j1, j2))
                return c

            # Check if each finger is done
            for i, inner, outer in zip([0,1,2], [2,3,4] ,[5,6,7]):
                if self.active_goal.stop_when_convex[i]:
                    self.done[i] = is_convex(msg.position[inner], msg.position[outer])
                else:
                    self.done[i] = msg.position[inner] < 0.01 and msg.position[outer] < 0.01

    def status_cb(self, msg):
        """Interpret BHand status, send appropriate commands and update activity state"""

        # Return if not active
        if self.server and not self.server.is_active():
            return

        # Get the masked modes
        masked_modes = [m for i, m in enumerate(msg.mode) if i < 3 and self.active_goal.release_mask[i]]

        # Check the modes based on the activity states:
        if self.state == self.PRERELEASE:
            rospy.loginfo("Sending release command...")
            self.cmd_pub.publish(self.release_cmd)
            # Check if all joints are in velocity mode
            if all([m == BHandCmdMode.MODE_VELOCITY for m in masked_modes]):
                self.state = self.RELEASING
                rospy.loginfo("Releasing...")

        elif self.state == self.RELEASING:
            # Disable fingers which are done
            if not all([m == BHandCmdMode.MODE_PID for m in masked_modes]):
                for f_id, done in enumerate(self.done):
                    # Stop the finger if it's done
                    if done:
                        self.release_cmd.mode[f_id] = BHandCmdMode.MODE_PID
                        self.release_cmd.cmd[f_id] = self.position[f_id]
                self.cmd_pub.publish(self.release_cmd)
            else:
                rospy.loginfo("Released.")
                self.state = None
                self.server.set_succeeded()


        elif self.state in [self.ABORTING, self.PREEMPTING]:
            # Check if all joints are in effort mode
            if not all([m == BHandCmdMode.MODE_IDLE for m in masked_modes]):
                rospy.logwarn("Aborting release.")
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
            rospy.loginfo("Preempting peers...")
            self.parent.preempt_peers_of(self.name)

        # Accept the new goal
        rospy.loginfo("Accepting new goal...")
        self.active_goal = self.server.accept_new_goal()

        # Clear the release state
        self.state = self.PRERELEASE
        self.done = [False] * 3
        self.position = [0.0] * 3

        # Construct hand commands for releasing and holding
        self.release_cmd = BHandCmd()
        self.abort_cmd = BHandCmd()
        for f_id, use_finger in enumerate(self.active_goal.release_mask):
            rospy.loginfo("Release %s finger %d" % ('using' if use_finger else 'not using', 1+f_id))
            # Release command
            self.release_cmd.mode[f_id] = BHandCmd.MODE_VELOCITY if use_finger else BHandCmd.MODE_SAME
            self.release_cmd.cmd[f_id] = -1.0 * self.active_goal.release_speed[f_id] if use_finger else 0.0

            # Abort command
            self.abort_cmd.mode[f_id] = BHandCmd.MODE_IDLE if use_finger else BHandCmd.MODE_SAME
            self.abort_cmd.cmd[f_id] = 0.0

        self.release_cmd.mode[3] = -1
        self.abort_cmd.mode[3] = -1

    def preempt_cb(self):
        """Idle the hand"""
        rospy.logwarn("Preemption requested!")
        self.state = self.PREEMPTING

    def peer_preempt_cb(self):
        """Abort the goal without idling"""
        rospy.logwarn("%s: Preemption requested by peer!" % self.name)

        if self.server and self.server.is_active():
            self.server.set_aborted(text="Peer command received")
