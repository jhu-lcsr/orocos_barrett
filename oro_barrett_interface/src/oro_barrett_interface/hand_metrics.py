
from collections import deque

from math import pi, sin, cos, tan, sqrt
import numpy as np
from numpy.linalg import norm

import PyKDL as kdl

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

# Max fingertip radius defined based on BHand kinematics
MAX_FINGERTIP_RADIUS = 0.15536


def hat(v):
    """Skew-symmetric operator"""
    return np.array([
        [   0.0, -v[2],  v[1]],
        [  v[2],   0.0, -v[0]],
        [ -v[1],  v[0],   0.0]])

def proj(q, n):
    """Projects vector q onto plane n"""
    return q - np.dot(q, n) * n

def normalize(v):
    return v / np.linalg.norm(v)

def compute_circle_parameters(points):
    for i in range(3):
        points[i] = np.array(points[i])

    # compute plane normal
    normal = np.cross(points[1] - points[0], points[2] - points[1])
    normal = normal/norm(normal)

    # Compute transform to rotate points into frame
    nz = normalize(normal)
    nx = normalize(np.cross(nz, np.array([0,0,1])))
    ny = normalize(np.cross(nz, nx))
    R = kdl.Rotation(
        kdl.Vector(*nx),
        kdl.Vector(*ny),
        kdl.Vector(*nz))

    # get coordinates of points parallel to the frame of the plane
    points_rot = [np.array(list(R.Inverse()*kdl.Vector(*p))) for p in points]

    (Ax, Ay, _) = A = points_rot[0]
    (Bx, By, _) = B = points_rot[1]
    (Cx, Cy, _) = C = points_rot[2]

    a = norm(B - A)
    b = norm(C - B)
    c = norm(A - C)

    # Catch zero-division error (colinear fingertips)
    try:
        # compute the radius of the circle
        radius = a * b * c / sqrt((a+b+c)*(-a+b+c)*(a-b+c)*(a+b-c))

        # compute the position of the center of the circle
        D = 2 * (Ax * (By - Cy) + Bx * (Cy - Ay) + Cx * (Ay - By))
        center_rot = np.array(
            [((Ax*Ax + Ay*Ay)*(By-Cy) + (Bx*Bx + By*By)*(Cy-Ay) + (Cx*Cx + Cy*Cy)*(Ay-By))/D,
             ((Ax*Ax + Ay*Ay)*(Cx-Bx) + (Bx*Bx + By*By)*(Ax-Cx) + (Cx*Cx + Cy*Cy)*(Bx-Ax))/D,
             points_rot[0][2]])

    except ZeroDivisionError as err:
        rospy.logwarn("Could not compute fingertip radius: zero division error")
        return {'n':normal, 'p':np.array([0,0,0]), 'r':0}

    # rotate the center back into the coordinates of the original points
    center = np.array(list(R*kdl.Vector(*center_rot)))

    return {'n':normal, 'p':center, 'r':radius}


def compute_finger_cage_radius(q1, q2):
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

def compute_fingertip_radius(tf_prefix, listener, geometer=None):
    """Compute the radius of the circle defined by the three fingertips"""

    # construct frame names
    prox_links = [''] * 3
    med_links = [''] * 3
    tip_links = [''] * 3
    palm_link = '/'.join([tf_prefix,'bhand_palm_link'])

    tip = [None]*3
    med = [None]*3
    prox = [None]*3
    bases = [None]*3

    try:
        for f,fl in FINGERS:
            # construct frame names
            if f != F3:
                prox_links[f] = '/'.join([tf_prefix,'finger_%d' % fl,'prox_link'])
            else:
                prox_links[f] = palm_link
            med_links[f] = '/'.join([tf_prefix,'finger_%d' % fl,'med_link'])
            tip_links[f] = '/'.join([tf_prefix,'finger_%d' % fl,'tip_link'])

            # get fingertip position
            prox[f] = np.array(listener.lookupTransform(palm_link,  prox_links[f], rospy.Time(0))[0])
            med[f] = np.array(listener.lookupTransform(palm_link,  med_links[f], rospy.Time(0))[0])
            tip[f] = np.array(listener.lookupTransform(palm_link, tip_links[f], rospy.Time(0))[0])

            # compute the base directions
            bases[f] = prox[f]-med[f]
            bases[f][2] = 0
            bases[f] = bases[f]/norm(bases[f])

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
        rospy.logwarn("Could not lookup finger frames!")
        return 0

    # compute the circle from the three points
    circle = compute_circle_parameters(tip)

    # project the direction of the fingertips onto the fingertip plane
    bases_proj = [normalize(proj(b, circle['n'])) for b in bases]

    # check if the fingers are no longer pointing inside the circle
    if circle['n'][2] < 0:
        circle['r'] = 0

    # limit radius size
    circle['r'] = min(MAX_FINGERTIP_RADIUS, circle['r'])

    # draw it if geometer is being used
    if geometer:
        geometer.plane(palm_link, key='fingertips', draw_triad=True, **circle)

        for f,fl in FINGERS:
            geometer.point(palm_link, tip[f], 0.01, key='f%d'%fl)
            geometer.line(palm_link, tip[f], bases_proj[f], t=[0.0, 0.02], key='g%dp'%fl)

        geometer.publish()

    # return the circle radius
    return circle['r']
