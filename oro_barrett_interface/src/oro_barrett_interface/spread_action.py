
from collections import deque

import numpy

import rospy
import actionlib

from sensor_msgs.msg import JointState
from oro_barrett_msgs.msg import BHandSpreadAction, BHandStatus, BHandCmd


class SpreadAction(object):
    """
    The Spread Action performs a closed loop two-stage spread. It first closes
    the fingers at a given speed until they encounter resistance and are
    stopped. Second, it applies a given effort with the desired fingers.

    Once it has completed, it will leave the hand executing an effort command.
    If it is preempted, it will idle the fingers that it is commanding.

    See the BHandSpreadAction message for more details.
    """
    # Activity states
    PRESPREAD = 0
    SPREADING = 1
    SPREADED = 2
    ABORTING = 3
    PREEMPTING = 4

    def __init__(self, name='spread', parent=None):
        # Delegates
        self.server = None

        # Properties
        self.name = name
        self.parent = parent

        # State
        self.state = None
        self.done_moving = None

        # ROS parameters
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.005)

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
            self.name, BHandSpreadAction,
            auto_start=False)

        self.server.register_goal_callback(self.goal_cb)
        self.server.register_preempt_callback(self.preempt_cb)

        self.server.start()

    def joint_states_cb(self, msg):
        """Determine when the spread joint is done moving"""

        # Return if not active
        if self.server and not self.server.is_active():
            return

        # Add the latest joint state
        now = rospy.Time.now()

        if (now-msg.header.stamp).to_sec() < -0.01:
            msg = "Hand state timestamp is in the future by %g seconds." % (now-msg.header.stamp).to_sec()
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        # Check if the goal has been reached
        errs = [abs(msg.position[i] - self.active_goal.spread_position) for i in [0, 1]]
        rospy.logdebug(str(errs))
        self.done_moving = all([e < self.goal_tolerance for e in errs])

    def status_cb(self, msg):
        """Interpret BHand status, send appropriate commands and update activity state"""

        # Return if not active
        if self.server and not self.server.is_active():
            return

        # Get the masked modes
        spread_mode = msg.mode[3]

        # Check the modes based on the activity states:
        if self.state == self.PRESPREAD:
            rospy.loginfo("Sending spread command...")
            self.cmd_pub.publish(self.spread_cmd)
            # Check if spread is in velocity mode
            if spread_mode == BHandCmd.MODE_TRAPEZOIDAL:
                self.state = self.SPREADING
                self.spread_start_time = rospy.Time.now()
                rospy.loginfo("Spreading...")

        elif self.state == self.SPREADING:
            # Check if all joints are in effort mode
            if self.done_moving:
                rospy.loginfo("Spreaded.")
                self.state = self.SPREADED
                self.server.set_succeeded()

        elif self.state == self.SPREADED:
            pass

        elif self.state in [self.ABORTING, self.PREEMPTING]:
            # Check if all joints are in effort mode
            if not all([m == BHandCmd.MODE_IDLE for m in masked_modes]):
                rospy.logwarn("Aborting spread.")
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

        # Clear the spread state
        self.state = self.PRESPREAD
        self.done_moving = False
        self.spread_start_time = rospy.Time.now()

        # Construct hand commands for spreading and holding
        self.spread_cmd = BHandCmd()
        self.abort_cmd = BHandCmd()

        self.spread_cmd.cmd[3] = self.active_goal.spread_position

        self.spread_cmd.mode = 3*[BHandCmd.MODE_SAME] + [BHandCmd.MODE_TRAPEZOIDAL]
        self.abort_cmd.mode = 3*[BHandCmd.MODE_SAME] + [BHandCmd.MODE_IDLE]

    def preempt_cb(self):
        """Idle the hand"""
        rospy.logwarn("Preemption requested!")
        self.state = self.PREEMPTING

    def peer_preempt_cb(self):
        """Abort the goal without idling"""
        rospy.logwarn("%s: Preemption requested by peer!" % self.name)
        if self.server and self.server.is_active():
            self.server.set_aborted(text="Peer command received")
