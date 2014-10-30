
from collections import deque

from math import pi, sin, cos, tan, sqrt
import numpy as np
from numpy.linalg import norm

import rospy
import actionlib
import tf

try:
    from geometer import Geometer
except ImportError as err:
    pass

from sensor_msgs.msg import JointState
from oro_barrett_msgs.msg import BHandGraspAction, BHandStatus, BHandCmd, BHandCmdMode

F1 = 0
F2 = 1
F3 = 2
FINGERS = zip([F1, F2, F3], [1,2,3])

def hat(v):
    """Skew-symmetric operator"""
    return np.array([
        [   0.0, -v[2],  v[1]],
        [  v[2],   0.0, -v[0]],
        [ -v[1],  v[0],   0.0]])

def proj(q, n):
    """Projects vector q onto plane n"""
    return q - np.dot(q, n) * n

class GraspAction(object):
    """
    The Grasp Action performs a closed loop two-stage grasp. It first closes
    the fingers at a given speed until they encounter resistance and are
    stopped. Second, it applies a given effort with the desired fingers.

    Once it has completed, it will leave the hand executing an effort command.
    If it is preempted, it will idle the fingers that it is commanding.

    See the BHandGraspAction message for more details.
    """
    # Activity states
    PREGRASP = 0
    GRASPING = 1
    HOLDING = 2
    ABORTING = 3
    PREEMPTING = 4
    DONE = 5

    def compute_finger_cage_radius(self, q1, q2):
        """Compute the circle inscribed between the fingers and the palm.

        q1: Inner joint angle
        q2: Outer joint angle

        returns: Circle parameters: (x, y, r)
        """

        # Get the two absolute finger angles
        th1 = q1
        th2 = pi/4+q2

        # Solution selection parameters
        t1 = 1
        t2 = 1
        t3 = 1

        # Select different solutions based on angles so that we select the
        # solution which is caged by the finger
        if th1+th2 > pi/2:
            t2 = -1
        if th1 > pi/2:
            t1 = -1

        try:
            # X position of the circle center
            x = ((0.07 * (t3 - t1 * sqrt(1. + pow(tan(th1),2))) * (sin(th1) - cos(th1) * tan(th1+th2))) /
                 ((-t3 + t1 * sqrt(1. + pow(tan(th1),2))) * tan(th1+th2)+tan(th1) * (t3 - t2 * sqrt(1. + pow(tan(th1+th2),2)))))

            # Y position of the circle center
            y = ((t3 * sin(th1) * (0.07 * tan(th1) - 0.07 * tan(th1+th2))) /
                 ((-t3 + t1 * sqrt(1.+pow(tan(th1),2))) * tan(th1 + th2)+ tan(th1) * (t3 - t2 * sqrt(1.+pow(tan(th1+th2),2)))))

            # Circle radius
            r = ((sin(th1) * (0.07 * tan(th1)-0.07 * tan(th1+th2))) /
                 ((-t3 + t1 * sqrt(1. +pow(tan(th1),2))) * tan(th1+th2)+tan(th1) * (t3-t2 * sqrt(1. +pow(tan(th1+th2),2)))))
        except ZeroDivisionError as err:
            rospy.logwarn("Could not compute cage radius: zero division error")
            return (0,0,0)

        return (x, y, r)

    def compute_fingertip_radius(self):
        """Compute the radius of the circle defined by the three fingertips"""

        # Max fingertip radius defined based on BHand kinematics
        MAX_FINGERTIP_RADIUS = 0.15536

        # construct frame names
        prox_links = [''] * 3
        med_links = [''] * 3
        tip_links = [''] * 3
        palm_link = '/'.join([self.tf_prefix,'bhand_palm_link'])

        tip = [[]]*3
        med = [[]]*3
        prox = [[]]*3
        bases = [[]]*3

        try:
            for f,fl in FINGERS:
                if f != F3:
                    prox_links[f] = '/'.join([self.tf_prefix,'finger_%d' % fl,'prox_link'])
                else:
                    prox_links[f] = palm_link
                med_links[f] = '/'.join([self.tf_prefix,'finger_%d' % fl,'med_link'])
                tip_links[f] = '/'.join([self.tf_prefix,'finger_%d' % fl,'tip_link'])

                # get fingertip position
                prox[f] = np.array(self.listener.lookupTransform(palm_link,  prox_links[f], rospy.Time(0))[0])
                med[f] = np.array(self.listener.lookupTransform(palm_link,  med_links[f], rospy.Time(0))[0])
                tip[f] = np.array(self.listener.lookupTransform(palm_link, tip_links[f], rospy.Time(0))[0])

                bases[f] = prox[f]-med[f]
                bases[f][2] = 0
                bases[f] = bases[f]/norm(bases[f])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn("Could not lookup finger frames!")
            return MAX_FINGERTIP_RADIUS

        # compute fingertip plane normal
        n = np.inner(hat(tip[F2] - tip[F1]), tip[F3] - tip[F2])
        n = n/norm(n)

        # project the direction of the fingertips onto the fingertip plane
        bases_proj = [None]*3
        for f,fl in FINGERS:
            g = proj(bases[f], n)
            g = g/norm(g)
            bases_proj[f] = g

        # compute projected points (for 3d offset)
        A = (Ax, Ay, Az) = proj(tip[F1],n)
        B = (Bx, By, Bz) = proj(tip[F2],n)
        C = (Cx, Cy, Cz) = proj(tip[F3],n)
        #print('A: %s, B: %s, C: %s' % (str(A), str(B), str(C)))

        # plane origin
        fingertip_plane_origin = np.inner(n, (tip[F1] - A)) * n

        a = norm(tip[F2] - tip[F1])
        b = norm(tip[F3] - tip[F2])
        c = norm(tip[F1] - tip[F3])

        # check if the fingers are no longer pointing inside the circle
        if n[2] > 0:
            # Catch zero-division error (colinear fingertips)
            try:
                radius = a * b * c / sqrt((a+b+c)*(-a+b+c)*(a-b+c)*(a+b-c))

                D = 2 * (Ax * (By - Cy) + Bx * (Cy - Ay) + Cx * (Ay - By))
                center_2d = (
                    ((Ax*Ax + Ay*Ay)*(By-Cy) + (Bx*Bx + By*By)*(Cy-Ay) + (Cx*Cx + Cy*Cy)*(Ay-By))/D,
                    ((Ax*Ax + Ay*Ay)*(Cx-Bx) + (Bx*Bx + By*By)*(Ax-Cx) + (Cx*Cx + Cy*Cy)*(Bx-Ax))/D)

            except ZeroDivisionError as err:
                rospy.logwarn("Could not compute fingertip radius: zero division error")
                return MAX_FINGERTIP_RADIUS
        else:
            radius = 0
            center_2d = (0,0)

        if self.geometer:
            self.geometer.plane(palm_link, fingertip_plane_origin, n, radius, c=center_2d, key='fingertips', draw_triad=False)

            for f,fl in FINGERS:
                self.geometer.point(palm_link, tip[f], 0.01, key='f%d'%fl)
                #self.geometer.line(palm_link, prox[f], bases[f], t=[0.0, 0.1], key='g%d'%fl)
                self.geometer.line(palm_link, tip[f], bases_proj[f], t=[0.0, 0.02], key='g%dp'%fl)
            self.geometer.publish()


        # Limit the radius if the fingertips approach colinearity
        return radius if radius < MAX_FINGERTIP_RADIUS else MAX_FINGERTIP_RADIUS


    def __init__(self, name='grasp', parent=None):
        # Delegates
        self.server = None

        # Properties
        self.name = name
        self.parent = parent

        # State
        self.state = None
        self.done_moving = None
        # Buffers for determining when the fingers have stopped
        self.stamp_history = None
        self.velocity_history = None
        self.position_history = None

        self.inner_outer_indices = zip([0,1,2], [2,3,4], [5,6,7])
        self.finger_cage_radii = [0,0,0]
        self.fingertip_radius = 0

        # ROS parameters
        self.use_geometer = rospy.get_param('~use_geometer', True)
        self.tf_prefix = rospy.get_param('~tf_prefix', '/wam/hand')
        self.feedback_period = rospy.Duration(rospy.get_param('~feedback_period', 0.1))
        self.static_vel_threshold = rospy.get_param('~static_vel_threshold', 0.1)
        self.static_pos_threshold = rospy.get_param('~static_pos_threshold', 0.05)
        self.vel_filter_cutoff = rospy.get_param('~vel_filter_cutoff', 0.5)
        self.pos_filter_cutoff = rospy.get_param('~pos_filter_cutoff', 0.5)
        self.min_static_duration = rospy.Duration(rospy.get_param('~min_static_duration', 0.1))
        self.max_static_duration = self.min_static_duration * 2.0

        # geometer
        try:
            self.geometer = Geometer(topic='~geometer') if self.use_geometer else None
        except NameError as err:
            self.geometer = None
            self.use_geometer = False
            rospy.logwarn('No geometer visualization support.')

        # TF
        self.listener = tf.TransformListener()

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
            self.name, BHandGraspAction,
            auto_start=False)

        self.server.register_goal_callback(self.goal_cb)
        self.server.register_preempt_callback(self.preempt_cb)

        self.server.start()

    def joint_states_cb(self, msg):
        """Determine when joints are done moving"""

        # Compute finger cage radii
        for i, inner, outer in self.inner_outer_indices:
            self.finger_cage_radii[i] = self.compute_finger_cage_radius(msg.position[inner], msg.position[outer])
        rospy.logdebug("Finger cage radii are: %s" % str(self.finger_cage_radii[i]))

        # Compute fingertip radius
        self.tip_radius = self.compute_fingertip_radius()
        rospy.logdebug("Fingertip radius is: %g" % self.fingertip_radius)

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

        # Append to the history
        self.stamp_history.append(msg.header.stamp)
        for dof, (dof_hist, new_vel) in enumerate(zip(self.velocity_history, msg.velocity)):
            a = self.vel_filter_cutoff
            new_vel_filtered = a * dof_hist[-1] + (1.0 - a) * new_vel if len(dof_hist) > 0 else new_vel
            dof_hist.append(new_vel_filtered)
            #print("Joint %d vel: %g" % (dof, new_vel))

        for dof, (dof_hist, new_pos) in enumerate(zip(self.position_history, msg.position)):
            a = self.pos_filter_cutoff
            new_pos_filtered = a * dof_hist[-1] + (1.0 - a) * new_pos if len(dof_hist) > 0 else new_pos
            dof_hist.append(new_pos_filtered)
            #print("Joint %d pos: %g" % (dof, new_pos))

        # Pop off old data
        while len(self.stamp_history) > 0 and (now - self.stamp_history[0]) > self.max_static_duration:
            self.stamp_history.popleft()
            for dof_hist in self.velocity_history:
                dof_hist.popleft()
            for dof_hist in self.position_history:
                dof_hist.popleft()

        # Find the index of the relevant elements
        #start_index = min([i for i, s in enumerate(self.stamp_history) if (now-s) < self.min_static_duration])
        #print("Start index: %d" % start_index)

        if len(self.stamp_history) == 0 or now - self.stamp_history[0] < self.min_static_duration:
            return

        # Check the velocity
        for dof, (vel_hist, pos_hist)  in enumerate(zip(self.velocity_history, self.position_history)):
            # Mark the dof as done moving if it's below the static velocity threshold
            if 0:
                below_thresh = [abs(v) < self.static_vel_threshold for v in vel_hist]
                print("dof %d %d%% below threshold" % (dof, 100*len([b for b in below_thresh if b])/len(below_thresh)))
                if all(below_thresh):
                    self.done_moving[dof] = True
            else:
                below_thresh = (max(pos_hist) - min(pos_hist)) < self.static_pos_threshold
                self.done_moving[dof] = below_thresh

    def is_used(self, dof):
        if dof in [2, 3, 4]:
            return self.active_goal.grasp_mask[dof - 2]
        elif dof in [5, 6, 7]:
            return self.active_goal.grasp_mask[dof - 5]
        return False

    def status_cb(self, msg):
        """Interpret BHand status, send appropriate commands and update activity state"""

        # Return if not active
        if self.server and not self.server.is_active():
            return

        # Get the masked modes
        masked_modes = [m for i, m in enumerate(msg.mode) if i < 3 and self.active_goal.grasp_mask[i]]

        # Check the modes based on the activity states:

        if self.state == self.PREGRASP:
            rospy.loginfo("Sending grasp command...")
            self.cmd_pub.publish(self.grasp_cmd)
            # Check if all joints are in velocity mode
            if all([m == BHandCmdMode.MODE_TRAPEZOIDAL for m in masked_modes]):
                self.state = self.GRASPING
                self.grasp_start_time = rospy.Time.now()
                rospy.loginfo("Grasping...")

        elif self.state == self.GRASPING:
            # Check if all joints are in effort mode
            if not all([m == BHandCmdMode.MODE_TORQUE for m in masked_modes]):
                # Check if the hand is done moving, and change to effort command
                if all([dm for dof, dm in enumerate(self.done_moving) if self.is_used(dof)]):
                    rospy.loginfo("Sending hold command...")
                    self.cmd_pub.publish(self.hold_cmd)
            else:
                self.state = self.HOLDING

        elif self.state == self.HOLDING:
            fingertips_good = self.active_goal.min_fingertip_radius < self.fingertip_radius
            finger_cages_good = all(
                [self.active_goal.min_fingertip_radius < r for r in self.finger_cage_radii])

            if not fingertips_good:
                rospy.logerr("Fingertip circle violated minimum radius.")
                self.server.set_aborted()
            elif not finger_cages_good:
                rospy.logerr("Finger cages violated minimum radii.")
                self.server.set_aborted()
            else:
                rospy.loginfo("Grasped.")
                self.server.set_succeeded()

            self.state = self.DONE

        elif self.state in [self.ABORTING, self.PREEMPTING]:
            # Check if all joints are in effort mode
            if not all([m == BHandCmdMode.MODE_IDLE for m in masked_modes]):
                rospy.logwarn("Aborting grasp.")
                self.cmd_pub.publish(self.abort_cmd)
            else:
                if self.state == self.ABORTING:
                    self.server.set_aborted()
                elif self.state == self.PREEMPTING:
                    self.server.set_preempted()
                self.state = self.PREGRASP

        elif self.state == self.DONE:
            pass

    def goal_cb(self):
        """Construct a new command and reset the activity state"""
        # Tell the parent to preempt peers
        if self.parent:
            rospy.loginfo("Preempting peers...")
            self.parent.preempt_peers_of(self.name)

        # Accept the new goal
        rospy.loginfo("Accepting new goal...")
        self.active_goal = self.server.accept_new_goal()

        # Clear the grasp state
        self.state = self.PREGRASP
        self.stamp_history = deque()
        self.velocity_history = [deque() for i in range(8)]
        self.position_history = [deque() for i in range(8)]
        self.done_moving = [False] * 8
        self.grasp_start_time = rospy.Time.now()

        # Construct hand commands for grasping and holding
        self.grasp_cmd = BHandCmd()
        self.hold_cmd = BHandCmd()
        self.abort_cmd = BHandCmd()
        for f_id, use_finger in enumerate(self.active_goal.grasp_mask):
            rospy.loginfo("Grasp %s finger %d" % ('using' if use_finger else 'not using', 1+f_id))
            # Grasp command
            self.grasp_cmd.mode[f_id] = BHandCmd.MODE_TRAPEZOIDAL if use_finger else BHandCmd.MODE_SAME
            self.grasp_cmd.cmd[f_id] = self.active_goal.grasp_speed[f_id] if use_finger else 0.0

            # Hold command
            self.hold_cmd.mode[f_id] = BHandCmd.MODE_TORQUE if use_finger else BHandCmd.MODE_SAME
            self.hold_cmd.cmd[f_id] = self.active_goal.grasp_effort[f_id] if use_finger else 0.0

            # Abort command
            self.abort_cmd.mode[f_id] = BHandCmd.MODE_IDLE if use_finger else BHandCmd.MODE_SAME
            self.abort_cmd.cmd[f_id] = 0.0

        self.grasp_cmd.mode[3] = -1
        self.hold_cmd.mode[3] = -1
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
